#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
from dotenv import load_dotenv
import tempfile
import queue
import threading
import subprocess
import time
import asyncio
import io
from concurrent.futures import ThreadPoolExecutor

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
            self.client = openai.OpenAI(api_key=self.api_key)
            self.get_logger().info("OpenAI TTS 客户端已初始化。")
            
        # 创建线程池用于异步TTS调用
        self.thread_executor = ThreadPoolExecutor(max_workers=2)
        
        # TTS配置 - 使用最新模型和最快格式
        self.tts_model = os.environ.get("TTS_MODEL", "gpt-4o-mini-tts")  # 使用最新模型
        self.tts_voice = os.environ.get("TTS_VOICE", "coral")  # 推荐的新语音
        self.tts_format = os.environ.get("TTS_FORMAT", "wav")  # 使用WAV格式获得最低延迟
        self.tts_speed = float(os.environ.get("TTS_SPEED", "1.1"))  # 稍微加快语速
        
        self.get_logger().info(f"TTS 配置: 模型={self.tts_model}, 语音={self.tts_voice}, 格式={self.tts_format}, 速度={self.tts_speed}")

    def listener_callback(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info("收到 llm_response: " + text)
        with self.buffer_lock:
            self.buffer += text + " "
            words = self.buffer.strip().split()
            buffer_snapshot = self.buffer.strip()

        # 更激进的刷新策略 - 更快的响应
        if text.endswith(('.', '!', '?')) or len(words) >= 15:  # 降低词数阈值
            self.get_logger().info(f"满足刷新条件，当前词数: {len(words)}, 当前缓冲内容: {buffer_snapshot}")
            if self.flush_timer:
                self.flush_timer.cancel()
            self.flush_timer = threading.Timer(0.2, self.flush_buffer)  # 减少等待时间到0.2秒
            self.flush_timer.start()
        else:
            if self.flush_timer:
                self.flush_timer.cancel()
            self.flush_timer = threading.Timer(0.3, self.flush_buffer)  # 一般情况也减少等待时间
            self.flush_timer.start()

    def flush_buffer(self):
        with self.buffer_lock:
            text = self.buffer.strip()
            self.buffer = ""
        if not text:
            return
        self.get_logger().info("触发缓冲语音生成: " + text)
        
        # 异步调用TTS以避免阻塞
        self.thread_executor.submit(self.call_openai_tts_async, text)

    def call_openai_tts_async(self, text: str):
        """异步TTS调用，避免阻塞主线程"""
        try:
            start_time = time.time()
            self.get_logger().info(f"[TTS] 开始生成语音: {text[:50]}...")
            
            # 使用最新的模型和格式以获得最佳性能
            response = self.client.audio.speech.create(
                model=self.tts_model,  # 使用最新的 gpt-4o-mini-tts
                voice=self.tts_voice,  # 使用推荐的 coral 语音
                input=text,
                response_format=self.tts_format,  # 使用 wav 格式获得最低延迟
                speed=self.tts_speed,  # 稍微加快语速
                # 添加指令来优化语音质量
                instructions="Speak clearly and naturally with good pacing for a voice assistant."
            )
            
            generation_time = time.time() - start_time
            self.get_logger().info(f"[TTS] 语音生成完成，耗时: {generation_time:.2f}秒")

            # 检查是否启用文件保存模式
            save_mode = os.environ.get("TTS_SAVE_MODE", "false").lower() == "true"
            
            if save_mode:
                # 保存到持久目录
                import datetime
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:19]  # 包含微秒
                save_dir = "/workspaces/ros2_ws/audio_output"
                os.makedirs(save_dir, exist_ok=True)
                
                file_ext = "wav" if self.tts_format == "wav" else "mp3"
                audio_file = os.path.join(save_dir, f"tts_{timestamp}.{file_ext}")
                with open(audio_file, "wb") as f:
                    f.write(response.content)
                
                self.get_logger().info(f"🎵 音频文件已保存: {audio_file}")
                self.play_queue.put(audio_file)
            else:
                # 临时文件模式
                file_ext = "wav" if self.tts_format == "wav" else "mp3"
                temp_file = tempfile.mktemp(suffix=f".{file_ext}")
                with open(temp_file, "wb") as f:
                    f.write(response.content)

                self.get_logger().info(f"[TTS] 已生成音频文件: {temp_file}")
                self.play_queue.put(temp_file)
                
        except Exception as e:
            self.get_logger().error(f"[TTS] 调用 OpenAI TTS API 出错: {str(e)}")

    # 保留旧方法以兼容性
    def call_openai_tts(self, text: str):
        """同步TTS调用（已弃用，保留兼容性）"""
        self.call_openai_tts_async(text)

    # def play_worker(self):
    #     while rclpy.ok():
    #         try:
    #             temp_file = self.play_queue.get(timeout=1)
    #             # self.get_logger().info(f"播放队列大小: {self.play_queue.qsize()}")
    #             # self.get_logger().info(f"准备播放语音片段: {temp_file}")
    #             try:
    #                 self.get_logger().info(f"使用 mpg123 播放: {temp_file}")
    #                 process = subprocess.Popen(["mpg123", temp_file])
    #                 process.wait()
    #                 time.sleep(0.1)  # 更紧凑的播放间隔
    #                 # self.get_logger().info(f"mpg123 播放完成: {temp_file}")
    #             except Exception as e:
    #                 self.get_logger().error(f"使用 mpg123 播放失败: {str(e)}")
    #             os.remove(temp_file)
    #             self.get_logger().info(f"删除临时文件: {temp_file}")
    #         except queue.Empty:
    #             continue
    def play_worker(self):
        while rclpy.ok():
            try:
                temp_file = self.play_queue.get(timeout=1)
                start_time = time.time()
                self.get_logger().info(f"[PLAY] 准备播放语音片段: {temp_file}")
                
                # 优先使用pygame，因为它对WAV格式支持更好
                played = False
                
                try:
                    import pygame
                    # 针对WAV格式优化pygame设置
                    if self.tts_format == "wav":
                        pygame.mixer.pre_init(frequency=24000, size=-16, channels=1, buffer=512)
                    else:
                        pygame.mixer.pre_init(frequency=22050, size=-16, channels=2, buffer=512)
                    pygame.mixer.init()
                    
                    self.get_logger().info(f"[PLAY] 使用 pygame 播放: {temp_file}")
                    pygame.mixer.music.load(temp_file)
                    pygame.mixer.music.play()
                    
                    # 等待播放完成
                    while pygame.mixer.music.get_busy():
                        pygame.time.wait(50)  # 减少等待间隔
                    
                    # 完全停止并清理
                    pygame.mixer.music.stop()
                    pygame.mixer.quit()
                    
                    play_time = time.time() - start_time
                    self.get_logger().info(f"[PLAY] pygame 播放完成: {temp_file} (耗时: {play_time:.2f}秒)")
                    played = True
                    
                except ImportError:
                    self.get_logger().debug("[PLAY] pygame 不可用")
                except Exception as e:
                    self.get_logger().warn(f"[PLAY] pygame 播放失败: {str(e)}")
                
                # 备选播放方案
                if not played:
                    try:
                        import playsound
                        self.get_logger().info(f"[PLAY] 使用 playsound 播放: {temp_file}")
                        playsound.playsound(temp_file)
                        play_time = time.time() - start_time
                        self.get_logger().info(f"[PLAY] playsound 播放完成: {temp_file} (耗时: {play_time:.2f}秒)")
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
                # 清理临时文件 (但保留持久保存的文件)
                save_mode = os.environ.get("TTS_SAVE_MODE", "false").lower() == "true"
                
                try:
                    if 'temp_file' in locals() and os.path.exists(temp_file):
                        # 只有在非保存模式或者是临时文件时才删除
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
    finally:
        # 清理线程池
        if hasattr(node, 'executor'):
            node.executor.shutdown(wait=True)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()