import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import pyaudio
import queue
import threading
import time
import difflib
import wave
import tempfile
import openai
import tenacity
from dotenv import load_dotenv
import numpy as np
import collections
from concurrent.futures import ThreadPoolExecutor

# 加载 .env 文件
load_dotenv()

# -----------------------------------------------------------------------------
# Audio configuration
SAMPLE_RATE = 24000          # Whisper 默认 24 kHz
CHUNK_SIZE = 1024           # 音频块大小
MODEL_NAME = os.getenv("OPENAI_STT_MODEL", "whisper-1")

# VAD 配置 - 调整这些参数来改善唤醒词检测
VAD_THRESHOLD = 0.015       # 降低阈值，使VAD更敏感
SILENCE_DURATION = 2.0      # 静音持续时间 (秒)
MIN_SPEECH_DURATION = 0.2   # 降低最小语音持续时间，更快响应
BUFFER_HISTORY = 1.5        # 增加前缓冲区时间，确保捕获完整唤醒词
# -----------------------------------------------------------------------------

class OpenAISTTNodeWithVAD(Node):
    def __init__(self):
        super().__init__('openai_stt_node_with_vad')
        
        # 初始化 OpenAI 客户端
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise RuntimeError("请设置 OPENAI_API_KEY 环境变量")
        
        self.openai_client = openai.OpenAI(api_key=api_key)
        
        # 创建发布者，发布识别结果到话题 'speech_text'
        self.publisher_ = self.create_publisher(String, 'speech_text', 10)
        # 订阅 TTS 状态消息
        self.tts_status_sub = self.create_subscription(Bool, 'tts_status', self.tts_status_callback, 10)
        self.listening = True
        # TTS 打断相关
        self.tts_active = False                       # 当前是否在播报
        self.interrupt_pub = self.create_publisher(Bool, 'tts_interrupt', 10)

        # 语言参数：空字符串表示自动检测
        self.declare_parameter('language', '')
        self.language = self.get_parameter('language').value

        # 线程池：统一限流语音转写请求
        self.transcribe_executor = ThreadPoolExecutor(max_workers=2)

        # 允许用户调整VAD参数
        self.declare_parameter('vad_threshold', VAD_THRESHOLD)
        self.declare_parameter('silence_duration', SILENCE_DURATION)
        self.declare_parameter('min_speech_duration', MIN_SPEECH_DURATION)
        self.declare_parameter('buffer_history', BUFFER_HISTORY)
        
        # 获取参数
        self.vad_threshold = self.get_parameter('vad_threshold').value
        self.silence_duration = self.get_parameter('silence_duration').value
        self.min_speech_duration = self.get_parameter('min_speech_duration').value
        self.buffer_history = self.get_parameter('buffer_history').value

        # 初始化音频输入
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=SAMPLE_RATE,
            input=True,
            frames_per_buffer=CHUNK_SIZE
        )
        self.stream.start_stream()
        self.get_logger().info("开始监听麦克风 (带VAD)...")

        # 设置唤醒词
        self.wake_words = ["hi captain", "hey captain", "hello captain"]

        # VAD 相关变量
        self.vad_state = "silence"  # "silence", "speech", "processing"
        self.speech_buffer = collections.deque(maxlen=int(SAMPLE_RATE * self.buffer_history / CHUNK_SIZE))
        self.current_speech = bytearray()
        self.silence_counter = 0
        self.speech_counter = 0
        self.last_speech_time = 0
        
        # 能量历史，用于动态阈值调整
        self.energy_history = collections.deque(maxlen=50)  # 存储最近50个能量值
        self.background_energy = 0.01  # 初始背景能量估计值

        # 创建音频处理线程
        self.audio_thread = threading.Thread(target=self.audio_processing_thread, daemon=True)
        self.audio_thread.start()

        self.get_logger().info(f"VAD 配置: 阈值={self.vad_threshold}, 静音检测={self.silence_duration}s, " +
                               f"前缓冲区={self.buffer_history}s, 最小语音={self.min_speech_duration}s")

    def calculate_rms(self, audio_data):
        """计算音频RMS (均方根) 用于VAD"""
        # 转换为numpy数组
        audio_np = np.frombuffer(audio_data, dtype=np.int16)
        # 计算RMS
        rms = np.sqrt(np.mean(audio_np.astype(np.float64) ** 2))
        # 归一化到0-1范围
        return rms / 32768.0

    def update_background_energy(self, rms):
        """更新背景噪音能量估计（指数平滑）"""
        # α = 0.95，平滑系数；可调
        self.background_energy = 0.95 * self.background_energy + 0.05 * rms
        # 声音活动阈值为背景能量的 2.5 倍或最小 vad_threshold
        return max(self.background_energy * 2.5, self.vad_threshold)

    def audio_processing_thread(self):
        """音频处理线程，包含VAD逻辑"""
        continuous_listening = True  # 连续监听模式，即使没检测到唤醒词也继续监听
        
        while rclpy.ok():
            if not self.listening:
                time.sleep(0.1)
                continue

            try:
                # 读取音频数据
                audio_data = self.stream.read(CHUNK_SIZE, exception_on_overflow=False)
                current_time = time.time()                
                # 计算音频能量
                rms = self.calculate_rms(audio_data)

                # ---------- 打断检测 ----------
                if self.tts_active:
                    # 用同一函数估算动态阈值
                    dynamic_threshold_tts = self.update_background_energy(rms)
                    if rms > dynamic_threshold_tts:        # 有人插话
                        self.speech_counter += 1
                        need_frames = int(self.min_speech_duration * SAMPLE_RATE / CHUNK_SIZE)
                        if self.speech_counter >= need_frames:
                            # 触发打断
                            interrupt_msg = Bool()
                            interrupt_msg.data = True
                            self.interrupt_pub.publish(interrupt_msg)
                            self.get_logger().info("🛑 TTS 播放中检测到用户讲话，已发送打断信号")
                            # 复位计数器，避免频繁
                            self.speech_counter = 0
                            self.silence_counter = 0
                    else:
                        self.speech_counter = 0

                    # 打断模式下跳过后续 VAD & 识别逻辑
                    continue
                
                # 更新动态阈值 (根据背景噪音自适应)
                dynamic_threshold = self.update_background_energy(rms)
                
                # VAD 状态机
                if self.vad_state == "silence":
                    # 始终保持历史缓冲区
                    self.speech_buffer.append(audio_data)
                    
                    # 检测声音活动
                    if rms > dynamic_threshold:
                        self.speech_counter += 1
                        # 记录RMS用于调试
                        if self.speech_counter == 1:
                            self.get_logger().debug(f"检测到潜在语音开始 (RMS: {rms:.4f}, 阈值: {dynamic_threshold:.4f})")
                            
                        if self.speech_counter >= int(self.min_speech_duration * SAMPLE_RATE / CHUNK_SIZE):
                            # 检测到语音开始
                            self.vad_state = "speech"
                            self.speech_counter = 0
                            self.silence_counter = 0
                            self.last_speech_time = current_time
                            
                            # 将历史缓冲区添加到当前语音
                            self.current_speech = bytearray()
                            for buffered_chunk in self.speech_buffer:
                                self.current_speech.extend(buffered_chunk)
                            
                            self.get_logger().info(f"🎤 检测到语音开始 (RMS: {rms:.4f}, 阈值: {dynamic_threshold:.4f})")
                    else:
                        self.speech_counter = 0
                
                elif self.vad_state == "speech":
                    # 添加音频到当前语音缓冲区
                    self.current_speech.extend(audio_data)
                    
                    if rms <= dynamic_threshold * 0.8:  # 静音阈值略低于动态阈值
                        self.silence_counter += 1
                        silence_duration = self.silence_counter * CHUNK_SIZE / SAMPLE_RATE
                        
                        if silence_duration >= self.silence_duration:
                            # 检测到语音结束
                            self.vad_state = "processing"
                            speech_duration = len(self.current_speech) / (SAMPLE_RATE * 2)  # 2 bytes per sample
                            self.get_logger().info(f"🔇 检测到语音结束 (时长: {speech_duration:.2f}s)")
                            
                            # 处理语音
                            if speech_duration > 0.5:  # 确保语音片段足够长
                                # 在单独线程中处理语音，避免阻塞主VAD循环
                                self.transcribe_executor.submit(self.process_speech_chunk,
                                                                bytes(self.current_speech))
                            
                            # 重置状态
                            self.current_speech = bytearray()
                            self.silence_counter = 0
                            self.vad_state = "silence"
                    else:
                        self.silence_counter = 0
                        self.last_speech_time = current_time

                # 超时保护：如果语音持续太久，强制处理
                if self.vad_state == "speech" and (current_time - self.last_speech_time) > 10.0:
                    self.get_logger().info("⏰ 语音超时，强制处理")
                    speech_data = bytes(self.current_speech)
                    self.transcribe_executor.submit(self.process_speech_chunk, speech_data)
                    
                    self.current_speech = bytearray()
                    self.vad_state = "silence"

            except Exception as e:
                self.get_logger().error(f"音频处理错误: {e}")
                time.sleep(0.05)

    def process_speech_chunk(self, speech_data):
        """处理检测到的语音片段"""
        if len(speech_data) < SAMPLE_RATE * 2 * 0.3:  # 小于0.3秒的语音忽略
            self.get_logger().debug("语音片段太短，忽略")
            return

        try:
            # 创建临时WAV文件
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=True) as tmp:
                self.write_wav(tmp.name, speech_data)
                transcript = self.transcribe_file(tmp.name)
                if transcript:
                    self.process_recognized_text(transcript)
        except Exception as e:
            self.get_logger().error(f"语音处理错误: {e}")

    def write_wav(self, path: str, pcm_bytes: bytes):
        """写 WAV 工具"""
        with wave.open(path, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(SAMPLE_RATE)
            wf.writeframes(pcm_bytes)

    @tenacity.retry(stop=tenacity.stop_after_attempt(3),
                    wait=tenacity.wait_exponential(multiplier=1, min=1, max=10))
    def transcribe_file(self, fname: str) -> str:
        """Whisper 调用，带重试"""
        try:
            with open(fname, "rb") as f:
                params = dict(
                    model=MODEL_NAME,
                    file=f,
                    prompt="The wake word is 'captain'.",
                    temperature=0,
                    response_format="text"
                )
                if self.language:           # 非空则显式指定语言
                    params['language'] = self.language
                response = self.openai_client.audio.transcriptions.create(**params)
            return response.strip() if isinstance(response, str) else response.strip()
        except Exception as e:
            self.get_logger().error(f"OpenAI API 调用失败: {e}")
            return ""

    def tts_status_callback(self, msg: Bool):
        """TTS 播放状态回调"""
        # True = 播放中；False = 播放结束
        self.tts_active = msg.data
        if self.tts_active:
            self.get_logger().info("检测到 TTS 正在播放，开启打断监听")
        else:
            self.get_logger().info("TTS 播放结束，恢复正常识别")

    def process_recognized_text(self, text):
        """处理识别的文本，检查唤醒词"""
        if not text:
            return
            
        lower_text = text.lower()
        match_found = False
        matching_word = None
        
        # 更严格的唤醒词检查
        for word in self.wake_words:
            if word in lower_text:
                match_found = True
                matching_word = word
                break
                
            # 模糊匹配，尤其针对句子开头
            # 如果句子以唤醒词的部分开头，增加匹配可能性
            if lower_text.startswith(word[:5]):  # 检查是否以唤醒词前5个字符开头
                ratio = difflib.SequenceMatcher(None, lower_text[:len(word)], word).ratio()
                if ratio > 0.7:  # 对句首更宽松的阈值
                    match_found = True
                    matching_word = word
                    break
                    
            # 一般模糊匹配
            ratio = difflib.SequenceMatcher(None, lower_text, word).ratio()
            if ratio > 0.75:
                match_found = True
                matching_word = word
                break
                
        if match_found:
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)
            self.get_logger().info(f"🔥 识别结果 (唤醒词激活 '{matching_word}'): {text}")
        else:
            self.get_logger().info(f"未检测到唤醒词: {text}")

    def destroy_node(self):
        """清理资源"""
        if hasattr(self, 'stream'):
            self.stream.stop_stream()
            self.stream.close()
        if hasattr(self, 'audio'):
            self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OpenAISTTNodeWithVAD()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()