#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import dashscope
from dotenv import load_dotenv
import tempfile
import queue
import threading
import requests
import time
import subprocess
import datetime

load_dotenv()

class QwenTTSNode(Node):
    def __init__(self):
        super().__init__('qwen_tts_node')
        # 订阅 llm_node 发布的 "llm_response" 话题
        self.subscription = self.create_subscription(
            String,
            'llm_response',
            self.listener_callback,
            10
        )
        self.get_logger().info("Qwen TTS 节点已启动，等待 llm_response 消息...")

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

        # 从环境变量中获取 DashScope API 密钥
        self.api_key = os.environ.get("DASHSCOPE_API_KEY")
        if not self.api_key:
            self.get_logger().error("请设置环境变量 DASHSCOPE_API_KEY！")
        else:
            dashscope.api_key = self.api_key
            self.get_logger().info("Qwen TTS 客户端已初始化。")

        # Qwen TTS 配置
        self.tts_model = os.environ.get("QWEN_TTS_MODEL", "qwen-tts-latest")
        self.tts_voice = os.environ.get("QWEN_TTS_VOICE", "Jada")  # 默认使用吴语女声
        self.tts_format = "wav"  # Qwen TTS 输出 wav 格式
        self.get_logger().info(
            f"Qwen TTS配置: model={self.tts_model}, voice={self.tts_voice}, format={self.tts_format}"
        )

    def listener_callback(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        
        # 打印识别到的语音内容
        print("\n" + "="*60)
        print("🎵 Captain 语音输出:")
        print("-"*60)
        print(f"'{text}'")
        print("="*60)
        
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
        
        # 打印准备合成的语音内容
        print("\n" + "🎵 开始语音合成:")
        print(f"📢 '{text}'")
        print("-"*40)
        
        self.get_logger().info("触发缓冲语音生成: " + text)
        self.call_qwen_tts(text)
        time.sleep(0.3)

    def call_qwen_tts(self, text: str):
        max_retries = 3
        retry_delay = 2.0  # 初始重试延迟
        
        for attempt in range(max_retries):
            try:
                self.get_logger().info(f"🔄 尝试调用Qwen TTS API (第 {attempt + 1}/{max_retries} 次)...")
                
                # 使用 Qwen TTS API - 添加超时设置
                response = dashscope.audio.qwen_tts.SpeechSynthesizer.call(
                    model=self.tts_model,
                    api_key=self.api_key,
                    text=text,
                    voice=self.tts_voice,
                    # 添加超时和重试参数
                    request_timeout=30,
                )

                if response.status_code == 200:
                    # 获取音频文件的URL
                    audio_url = response.output.audio["url"]
                    self.get_logger().info(f"✅ 获得音频URL: {audio_url}")
                    
                    # 下载音频文件
                    self._download_and_play_audio(audio_url, text)
                    return  # 成功，退出重试循环
                else:
                    self.get_logger().error(f"❌ Qwen TTS API 调用失败: {response.message}")
                    
            except Exception as e:
                error_msg = str(e)
                self.get_logger().error(f"❌ 调用 Qwen TTS API 出错 (第 {attempt + 1}/{max_retries} 次): {error_msg}")
                
                # 检查是否为网络相关错误
                if any(keyword in error_msg.lower() for keyword in ['ssl', 'connection', 'timeout', 'network', 'max retries', 'eof']):
                    if attempt < max_retries - 1:  # 还有重试机会
                        self.get_logger().info(f"🕐 网络错误，{retry_delay:.1f}秒后重试...")
                        time.sleep(retry_delay)
                        retry_delay *= 1.5  # 指数退避
                        continue
                    else:
                        self.get_logger().error(f"🚫 网络连接失败，已达到最大重试次数。请检查网络连接或稍后重试。")
                        self._handle_tts_fallback(text)
                        return
                else:
                    # 非网络错误，不重试
                    self.get_logger().error(f"🚫 TTS API 调用失败: {error_msg}")
                    self._handle_tts_fallback(text)
                    return
        
        # 如果所有重试都失败了
        self.get_logger().error("🚫 所有重试尝试都失败，启用回退模式")
        self._handle_tts_fallback(text)

    def _download_and_play_audio(self, audio_url: str, text: str):
        max_retries = 3
        retry_delay = 1.0
        
        for attempt in range(max_retries):
            try:
                self.get_logger().info(f"📥 下载音频文件 (第 {attempt + 1}/{max_retries} 次): {audio_url}")
                
                # 检查是否启用文件保存模式
                save_mode = os.environ.get("TTS_SAVE_MODE", "false").lower() == "true"
                
                if save_mode:
                    # 保存到持久目录
                    import datetime
                    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                    save_dir = "/workspaces/ros2_ws/audio_output"
                    os.makedirs(save_dir, exist_ok=True)
                    
                    audio_file = os.path.join(save_dir, f"qwen_tts_{timestamp}.wav")
                else:
                    # 使用临时文件
                    audio_file = tempfile.mktemp(suffix=".wav")

                # 下载音频文件
                response = requests.get(audio_url, timeout=30)
                response.raise_for_status()
                
                with open(audio_file, 'wb') as f:
                    f.write(response.content)
                
                if save_mode:
                    self.get_logger().info(f"🎵 Qwen TTS音频文件已保存: {audio_file}")
                    self.get_logger().info("💡 在宿主机上播放此文件来听取Jada音色（吴语）")
                else:
                    self.get_logger().info(f"✅ 已下载Qwen TTS音频文件: {audio_file}")
                
                # 将文件添加到播放队列
                self.play_queue.put(audio_file)
                return  # 成功，退出重试循环
                
            except Exception as e:
                error_msg = str(e)
                self.get_logger().error(f"❌ 下载Qwen TTS音频文件失败 (第 {attempt + 1}/{max_retries} 次): {error_msg}")
                
                # 检查是否为网络相关错误
                if any(keyword in error_msg.lower() for keyword in ['connection', 'timeout', 'network', 'ssl', 'httperror']):
                    if attempt < max_retries - 1:  # 还有重试机会
                        self.get_logger().info(f"🕐 下载失败，{retry_delay:.1f}秒后重试...")
                        time.sleep(retry_delay)
                        retry_delay *= 1.5
                        continue
                    else:
                        self.get_logger().error(f"🚫 音频下载失败，已达到最大重试次数")
                        self._handle_tts_fallback(text)
                        return
                else:
                    # 非网络错误，不重试
                    self.get_logger().error(f"🚫 音频下载失败: {error_msg}")
                    self._handle_tts_fallback(text)
                    return

    def play_worker(self):
        while rclpy.ok():
            published_status = False
            try:
                temp_file = self.play_queue.get(timeout=1)
                
                # 打印播放状态
                print("🔊 正在播放 Captain 语音...")
                
                self.get_logger().info(f"[PLAY] 准备播放Qwen TTS语音片段: {temp_file}")

                # 新增：发布播放开始状态
                status_msg = Bool()
                status_msg.data = True
                self.status_publisher.publish(status_msg)
                published_status = True

                # 检查文件是否存在
                if not os.path.exists(temp_file):
                    self.get_logger().error(f"[PLAY] 音频文件不存在: {temp_file}")
                    continue

                # 使用多种播放方法
                played = False
                
                # 方法1: 尝试使用 ffplay (ffmpeg)
                try:
                    result = subprocess.run([
                        'ffplay', '-nodisp', '-autoexit', '-v', 'quiet', temp_file
                    ], capture_output=True, timeout=30)
                    if result.returncode == 0:
                        print("✅ Captain 语音播放完成 (ffplay)\n")
                        self.get_logger().info(f"[PLAY] ffplay 播放Qwen TTS完成: {temp_file}")
                        played = True
                    else:
                        self.get_logger().debug(f"[PLAY] ffplay 失败: {result.stderr}")
                except (subprocess.TimeoutExpired, FileNotFoundError, Exception) as e:
                    self.get_logger().debug(f"[PLAY] ffplay 不可用或失败: {str(e)}")

                # 方法2: 尝试使用 aplay (ALSA)
                if not played:
                    try:
                        result = subprocess.run([
                            'aplay', '-q', temp_file
                        ], capture_output=True, timeout=30)
                        if result.returncode == 0:
                            print("✅ Captain 语音播放完成 (aplay)\n")
                            self.get_logger().info(f"[PLAY] aplay 播放Qwen TTS完成: {temp_file}")
                            played = True
                        else:
                            self.get_logger().debug(f"[PLAY] aplay 失败: {result.stderr}")
                    except (subprocess.TimeoutExpired, FileNotFoundError, Exception) as e:
                        self.get_logger().debug(f"[PLAY] aplay 不可用或失败: {str(e)}")

                # 方法3: 尝试pygame（转换格式后）
                if not played:
                    try:
                        # 先尝试转换音频格式
                        converted_file = temp_file.replace('.wav', '_converted.wav')
                        convert_result = subprocess.run([
                            'ffmpeg', '-i', temp_file, '-acodec', 'pcm_s16le', 
                            '-ar', '22050', '-ac', '1', '-y', converted_file
                        ], capture_output=True, timeout=10)
                        
                        if convert_result.returncode == 0 and os.path.exists(converted_file):
                            import pygame
                            pygame.mixer.pre_init(frequency=22050, size=-16, channels=1, buffer=512)
                            pygame.mixer.init()
                            self.get_logger().info(f"[PLAY] 使用 pygame 播放转换后的音频: {converted_file}")
                            pygame.mixer.music.load(converted_file)
                            pygame.mixer.music.play()
                            while pygame.mixer.music.get_busy():
                                pygame.time.wait(100)
                            pygame.mixer.music.stop()
                            pygame.mixer.quit()
                            print("✅ Captain 语音播放完成 (pygame)\n")
                            self.get_logger().info(f"[PLAY] pygame 播放转换后音频完成: {converted_file}")
                            played = True
                            
                            # 清理转换文件
                            try:
                                os.remove(converted_file)
                            except:
                                pass
                        else:
                            self.get_logger().debug("[PLAY] 音频格式转换失败")
                    except ImportError:
                        self.get_logger().debug("[PLAY] pygame 不可用")
                    except Exception as e:
                        self.get_logger().debug(f"[PLAY] pygame 播放失败: {str(e)}")

                # 方法4: 尝试原始pygame
                if not played:
                    try:
                        import pygame
                        pygame.mixer.pre_init(frequency=24000, size=-16, channels=1, buffer=512)
                        pygame.mixer.init()
                        self.get_logger().info(f"[PLAY] 使用原始 pygame 播放: {temp_file}")
                        pygame.mixer.music.load(temp_file)
                        pygame.mixer.music.play()
                        while pygame.mixer.music.get_busy():
                            pygame.time.wait(100)
                        pygame.mixer.music.stop()
                        pygame.mixer.quit()
                        print("✅ Captain 语音播放完成 (pygame原始)\n")
                        self.get_logger().info(f"[PLAY] pygame 原始播放完成: {temp_file}")
                        played = True
                    except Exception as e:
                        self.get_logger().debug(f"[PLAY] pygame 原始播放失败: {str(e)}")

                # 方法5: 尝试playsound
                if not played:
                    try:
                        import playsound
                        self.get_logger().info(f"[PLAY] 使用 playsound 播放: {temp_file}")
                        playsound.playsound(temp_file)
                        print("✅ Captain 语音播放完成 (playsound)\n")
                        self.get_logger().info(f"[PLAY] playsound 播放完成: {temp_file}")
                        played = True
                    except Exception as e:
                        self.get_logger().debug(f"[PLAY] playsound 播放失败: {str(e)}")

                if not played:
                    self.get_logger().info(f"[PLAY] Qwen TTS音频文件已生成但无可用播放器: {temp_file}")
                    self.get_logger().info("[PLAY] 音频文件已保存，可以手动播放或在有音频输出的环境中运行")
                    # 显示音频文件信息
                    try:
                        file_size = os.path.getsize(temp_file) / 1024
                        self.get_logger().info(f"[PLAY] 音频文件大小: {file_size:.1f} KB")
                    except:
                        pass

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

    def _handle_tts_fallback(self, text: str):
        """处理TTS失败的回退方案"""
        try:
            # 显示无法播放的文本内容
            print("\n" + "⚠️ " + "="*58)
            print("🔇 TTS 服务暂时不可用，显示文本内容:")
            print("-"*60)
            print(f"📝 '{text}'")
            print("="*60)
            print("💡 提示: 请检查网络连接或稍后重试\n")
            
            self.get_logger().warn(f"🔇 TTS不可用，仅显示文本: {text}")
            
            # 仍然发布状态以保持系统同步
            status_msg = Bool()
            status_msg.data = True
            self.status_publisher.publish(status_msg)
            
            # 模拟播放时间（基于文本长度估算）
            estimated_duration = max(1.0, len(text) * 0.15)  # 每字符约0.15秒
            time.sleep(min(estimated_duration, 10.0))  # 最多10秒
            
            status_msg.data = False
            self.status_publisher.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"回退处理失败: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = QwenTTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
