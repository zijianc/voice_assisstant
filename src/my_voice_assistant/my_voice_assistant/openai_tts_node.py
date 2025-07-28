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
            openai.api_key = self.api_key
            self.get_logger().info("OpenAI TTS 客户端已初始化。")

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

    def call_openai_tts(self, text: str):
        try:
            # 使用 OpenAI 新接口进行 TTS
            response = openai.audio.speech.create(
                model="tts-1",  # 或使用 "tts-1-hd"
                voice="nova",  # 可选：alloy, echo, fable, nova, onyx, shimmer
                input=text,
                speed=0.9 # 可选：0.5-2.0
            )

            # 保存音频文件
            temp_file = tempfile.mktemp(suffix=".mp3")
            with open(temp_file, "wb") as f:
                f.write(response.content)

            self.get_logger().info(f"已生成音频文件: {temp_file}")
            self.play_queue.put(temp_file)
        except Exception as e:
            self.get_logger().error("调用 OpenAI TTS API 出错: " + str(e))

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
                self.get_logger().info(f"准备播放语音片段: {temp_file}")
                played = False
                
                # 尝试多种播放器，按优先级排序
                audio_players = [
                    ("paplay", self.play_with_paplay),
                    ("ffplay", self.play_with_ffplay), 
                    ("mpg123", self.play_with_mpg123),
                    ("aplay", self.play_with_aplay)
                ]
                
                for player_name, play_func in audio_players:
                    try:
                        if play_func(temp_file):
                            self.get_logger().info(f"使用 {player_name} 播放完成: {temp_file}")
                            played = True
                            break
                    except FileNotFoundError:
                        self.get_logger().debug(f"{player_name} 不可用")
                        continue
                    except Exception as e:
                        self.get_logger().warn(f"{player_name} 播放失败: {str(e)}")
                        continue
                
                if not played:
                    self.get_logger().error("所有播放器都不可用，无法播放音频")
                    
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"播放工作线程出错: {str(e)}")
            finally:
                # 清理临时文件
                try:
                    if 'temp_file' in locals() and os.path.exists(temp_file):
                        os.remove(temp_file)
                        self.get_logger().debug(f"删除临时文件: {temp_file}")
                        
                    # 清理可能的转换文件
                    if 'temp_file' in locals():
                        wav_file = temp_file.replace('.mp3', '.wav')
                        if os.path.exists(wav_file):
                            os.remove(wav_file)
                            self.get_logger().debug(f"删除转换文件: {wav_file}")
                except Exception as e:
                    self.get_logger().warn(f"清理临时文件失败: {str(e)}")

    def play_with_paplay(self, temp_file):
        """使用 paplay 播放音频（需要先转换为 WAV）"""
        try:
            # 先检查 ffmpeg 是否可用
            subprocess.run(["which", "ffmpeg"], check=True, 
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            wav_file = temp_file.replace('.mp3', '.wav')
            
            # 转换 MP3 到 WAV
            convert_cmd = ["ffmpeg", "-y", "-i", temp_file, "-ar", "44100", "-ac", "2", wav_file]
            subprocess.run(convert_cmd, check=True, 
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            # 使用 paplay 播放
            play_cmd = ["paplay", wav_file]
            process = subprocess.run(play_cmd, check=True,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return True
            
        except (subprocess.CalledProcessError, FileNotFoundError):
            return False

    def play_with_ffplay(self, temp_file):
        """使用 ffplay 播放音频"""
        try:
            cmd = ["ffplay", "-nodisp", "-autoexit", "-volume", "80", temp_file]
            process = subprocess.run(cmd, check=True,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return True
        except (subprocess.CalledProcessError, FileNotFoundError):
            return False

    def play_with_mpg123(self, temp_file):
        """使用 mpg123 播放音频"""
        try:
            cmd = ["mpg123", "-q", temp_file]
            process = subprocess.run(cmd, check=True,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return True
        except (subprocess.CalledProcessError, FileNotFoundError):
            return False

    def play_with_aplay(self, temp_file):
        """使用 aplay 播放音频（需要先转换为 WAV）"""
        try:
            # 检查 ffmpeg 是否可用
            subprocess.run(["which", "ffmpeg"], check=True,
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            wav_file = temp_file.replace('.mp3', '.wav')
            
            # 转换 MP3 到 WAV
            convert_cmd = ["ffmpeg", "-y", "-i", temp_file, "-ar", "44100", "-ac", "2", wav_file]
            subprocess.run(convert_cmd, check=True,
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            # 使用 aplay 播放
            play_cmd = ["aplay", wav_file]
            process = subprocess.run(play_cmd, check=True,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return True
            
        except (subprocess.CalledProcessError, FileNotFoundError):
            return False
    
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