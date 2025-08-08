#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import openai
from dotenv import load_dotenv
import tempfile
import queue
import threading
import subprocess
import time

load_dotenv()

class OpenAITTSNode(Node):
    def __init__(self):
        super().__init__('openai_tts_node')
        # 订阅 llm_node 发布的 "llm_response" 话题
        self.subscription = self.create_subscription(
            String,
            'llm_response',
            self.listener_callback,
            10
        )
        self.get_logger().info("OpenAI TTS 节点已启动，等待 llm_response 消息...")

        # 新增：播放状态发布者（True 开始播放，False 播放结束）
        self.status_publisher = self.create_publisher(Bool, 'tts_status', 10)

        # 初始化播放队列和播放线程
        self.play_queue = queue.Queue()
        self.play_thread = threading.Thread(target=self.play_worker, daemon=True)
        self.play_thread.start()

        self.buffer = ""
        self.flush_timer = None
        self.flush_delay = 1.0  # 更短的缓冲时间，提高响应速度
        self.buffer_lock = threading.Lock()

        # 从环境变量中获取 OpenAI API 密钥
        self.api_key = os.environ.get("OPENAI_API_KEY")
        if not self.api_key:
            self.get_logger().error("请设置环境变量 OPENAI_API_KEY！")
        else:
            openai.api_key = self.api_key
            self.get_logger().info("OpenAI TTS 客户端已初始化。")

        # 新增：TTS 配置（低延时 + 友好热情语气）
        self.tts_model = os.environ.get("TTS_MODEL", "gpt-4o-mini-tts")
        self.tts_voice = os.environ.get("TTS_VOICE", "coral")
        self.tts_format = os.environ.get("TTS_FORMAT", "wav")  # wav/pcm 较低延迟；mp3 兼容性好
        try:
            self.tts_speed = float(os.environ.get("TTS_SPEED", "1.0"))  # 正常语速
        except Exception:
            self.tts_speed = 1.0
        self.tts_instructions = os.environ.get(
            "TTS_INSTRUCTIONS",
            "Speak in a friendly, polite, cheerful, and enthusiastic tone with a normal speaking pace."
        )
        self.get_logger().info(
            f"TTS配置: model={self.tts_model}, voice={self.tts_voice}, format={self.tts_format}, speed={self.tts_speed}"
        )

    def listener_callback(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info("收到 llm_response: " + text)
        with self.buffer_lock:
            self.buffer += text + " "
            words = self.buffer.strip().split()
            buffer_snapshot = self.buffer.strip()

        # 若句尾是句号、感叹号或问号，或长句（30词），且0.5秒内无新词
        if text.endswith(('.', '!', '?')) or len(words) >= 25:
            self.get_logger().info(f"满足刷新条件，当前词数: {len(words)}, 当前缓冲内容: {buffer_snapshot}")
            if self.flush_timer:
                self.flush_timer.cancel()
            self.flush_timer = threading.Timer(0.5, self.flush_buffer)  # 等待 0.5 秒无新词再刷新
            self.flush_timer.start()
        else:
            if self.flush_timer:
                self.flush_timer.cancel()
            self.flush_timer = threading.Timer(0.5, self.flush_buffer)
            self.flush_timer.start()

    def flush_buffer(self):
        with self.buffer_lock:
            text = self.buffer.strip()
            self.buffer = ""
        if not text:
            return
        self.get_logger().info("触发缓冲语音生成: " + text)
        self.call_openai_tts(text)
        time.sleep(0.3)

    def _ext_for_format(self, fmt: str) -> str:
        m = (fmt or '').lower()
        if m in ("mp3", "wav", "opus", "aac", "flac", "pcm"):
            return "." + m
        return ".mp3"

    def call_openai_tts(self, text: str):
        try:
            # 使用 OpenAI TTS，并传入 response_format + instructions
            response = openai.audio.speech.create(
                model=self.tts_model,
                voice=self.tts_voice,
                input=text,
                response_format=self.tts_format,
                speed=self.tts_speed,
                instructions=self.tts_instructions,
            )

            # 检查是否启用文件保存模式
            save_mode = os.environ.get("TTS_SAVE_MODE", "false").lower() == "true"
            ext = self._ext_for_format(self.tts_format)
            
            if save_mode:
                # 保存到持久目录
                import datetime
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                save_dir = "/workspaces/ros2_ws/audio_output"
                os.makedirs(save_dir, exist_ok=True)
                
                audio_file = os.path.join(save_dir, f"tts_{timestamp}{ext}")
                with open(audio_file, "wb") as f:
                    f.write(response.content)
                
                self.get_logger().info(f"🎵 音频文件已保存: {audio_file}")
                self.get_logger().info("💡 在宿主机上播放此文件来听取音频")
                
                # 仍然尝试播放
                self.play_queue.put(audio_file)
            else:
                # 原有的临时文件模式
                temp_file = tempfile.mktemp(suffix=ext)
                with open(temp_file, "wb") as f:
                    f.write(response.content)

                self.get_logger().info(f"已生成音频文件: {temp_file}")
                self.play_queue.put(temp_file)
                
        except Exception as e:
            self.get_logger().error("调用 OpenAI TTS API 出错: " + str(e))

    def play_worker(self):
        while rclpy.ok():
            published_status = False
            try:
                temp_file = self.play_queue.get(timeout=1)
                self.get_logger().info(f"[PLAY] 准备播放语音片段: {temp_file}")

                # 新增：发布播放开始状态
                status_msg = Bool()
                status_msg.data = True
                self.status_publisher.publish(status_msg)
                published_status = True

                # 只使用一个播放器，避免重复播放
                played = False
                try:
                    import pygame
                    pygame.mixer.pre_init(frequency=22050, size=-16, channels=2, buffer=512)
                    pygame.mixer.init()
                    self.get_logger().info(f"[PLAY] 使用 pygame 播放: {temp_file}")
                    pygame.mixer.music.load(temp_file)
                    pygame.mixer.music.play()
                    while pygame.mixer.music.get_busy():
                        pygame.time.wait(100)
                    pygame.mixer.music.stop()
                    pygame.mixer.quit()
                    self.get_logger().info(f"[PLAY] pygame 播放完成: {temp_file}")
                    played = True
                except ImportError:
                    self.get_logger().debug("[PLAY] pygame 不可用")
                except Exception as e:
                    self.get_logger().warn(f"[PLAY] pygame 播放失败: {str(e)}")

                if not played:
                    try:
                        import playsound
                        self.get_logger().info(f"[PLAY] 使用 playsound 播放: {temp_file}")
                        playsound.playsound(temp_file)
                        self.get_logger().info(f"[PLAY] playsound 播放完成: {temp_file}")
                        played = True
                    except ImportError:
                        self.get_logger().debug("[PLAY] playsound 不可用")
                    except Exception as e:
                        self.get_logger().debug(f"[PLAY] playsound 播放失败: {str(e)}")

                if not played:
                    self.get_logger().info(f"[PLAY] 音频文件已生成但无可用播放器: {temp_file}")
                    self.get_logger().info("[PLAY] 请安装音频播放器或在有音频输出的环境中运行")

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"[PLAY] 播放工作线程出错: {str(e)}")
            finally:
                # 新增：发布播放结束状态（仅当之前发布过开始）
                if published_status:
                    status_msg = Bool()
                    status_msg.data = False
                    self.status_publisher.publish(status_msg)

                # 清理临时文件 (但保留持久保存的文件)
                save_mode = os.environ.get("TTS_SAVE_MODE", "false").lower() == "true"
                try:
                    if 'temp_file' in locals() and os.path.exists(temp_file):
                        if not save_mode or temp_file.startswith('/tmp/'):
                            os.remove(temp_file)
                            self.get_logger().info(f"[CLEANUP] 删除临时文件: {temp_file}")
                        else:
                            self.get_logger().debug(f"[CLEANUP] 保留音频文件: {temp_file}")
                except Exception as e:
                    self.get_logger().warn(f"[CLEANUP] 清理临时文件失败: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = OpenAITTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()