import os
import json
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
import re

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

# 新增：门控/阈值/相似度/允许打断配置
RESUME_HANGOVER_SEC = float(os.getenv("STT_RESUME_HANGOVER_SEC", "0.8"))
TTS_VAD_THRESHOLD_BOOST = float(os.getenv("TTS_VAD_THRESHOLD_BOOST", "2.0"))
SNR_GATE = float(os.getenv("STT_WAKEWORD_SNR_GATE", "1.8"))  # 片段RMS/背景能量 比值门限
ALLOW_BARGE_IN = os.getenv("STT_ALLOW_BARGE_IN", "false").lower() == "true"
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
        # 新增：订阅 llm_response 做自回放文本过滤兜底
        self.tts_text_sub = self.create_subscription(String, 'llm_response', self.llm_response_callback, 10)
        self.listening = True  # 不再用它做门控，仅保持线程活跃

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

        # 设置唤醒词与严格正则（仅句首、词边界）
        self.wake_words = ["hi captain", "hey captain", "hello captain"]
        self.wake_regex = re.compile(r'^\s*(hi|hey|hello)\W+captain\b', re.I)

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
        self.tts_threshold_boost = TTS_VAD_THRESHOLD_BOOST

        # 新增：TTS门控状态
        self.tts_playing = False
        self.resume_at = 0.0
        self._should_drain = False
        self._freeze_noise = False
        # 最近TTS文本缓存
        self._tts_recent_text = ""
        self._tts_recent_expiry = 0.0

        # 创建音频处理线程
        self.audio_thread = threading.Thread(target=self.audio_processing_thread, daemon=True)
        self.audio_thread.start()

        self.get_logger().info(f"VAD 配置: 阈值={self.vad_threshold}, 静音检测={self.silence_duration}s, " +
                               f"前缓冲区={self.buffer_history}s, 最小语音={self.min_speech_duration}s")

    def calculate_rms(self, audio_data):
        """计算音频RMS (均方根) 用于VAD"""
        # 转换为numpy数组
        audio_np = np.frombuffer(audio_data, dtype=np.int16)
        if audio_np.size == 0:
            return 0.0
        # 计算RMS
        rms = np.sqrt(np.mean(audio_np.astype(np.float64) ** 2))
        # 归一化到0-1范围
        return rms / 32768.0

    def update_background_energy(self, rms):
        """动态阈值：非冻结时更新背景噪声；TTS期提升阈值"""
        if not self._freeze_noise:
            self.energy_history.append(rms)
            # 使用较低百分位数估计背景噪音
            if len(self.energy_history) > 10:
                self.background_energy = float(np.percentile(list(self.energy_history), 10))
        base = max(self.background_energy * 2.5, self.vad_threshold)
        if self.tts_playing:
            base *= self.tts_threshold_boost  # TTS期间提升阈值
        return base

    def audio_processing_thread(self):
        """音频处理线程，包含VAD逻辑"""
        continuous_listening = True  # 连续监听模式
        
        while rclpy.ok():
            # 读取音频数据（即便门控，也需要消耗流，便于 drain）
            try:
                audio_data = self.stream.read(CHUNK_SIZE, exception_on_overflow=False)
            except Exception as e:
                self.get_logger().error(f"音频读取错误: {e}")
                time.sleep(0.05)
                continue

            current_time = time.time()

            # 门控判定（TTS进行中或挂起期）
            gated = self.tts_playing or (self.resume_at and current_time < self.resume_at)

            # 首次进入门控/挂起后：清空缓冲，避免回放残留
            if gated and self._should_drain:
                try:
                    self.speech_buffer.clear()
                except Exception:
                    pass
                self.current_speech = bytearray()
                self.silence_counter = 0
                self.speech_counter = 0
                self._should_drain = False
                self.get_logger().debug("已清空缓冲与状态 (TTS期间/挂起)")

            # 在门控并且不允许打断时：冻结噪声估计并直接丢弃本轮数据
            if gated and not ALLOW_BARGE_IN:
                self._freeze_noise = True
                time.sleep(0.03)
                # 循环继续读取以保持 drain
                continue

            # 若已脱离门控，解除冻结
            if (not gated) and self._freeze_noise:
                self._freeze_noise = False
                self.resume_at = 0.0
                self.get_logger().debug("恢复噪声估计")

            try:
                # 计算音频能量
                rms = self.calculate_rms(audio_data)
                # 当前背景能量用于SNR判定
                bg = max(self.background_energy, 1e-6)
                snr_now = rms / bg

                # 更新/获取动态阈值
                dynamic_threshold = self.update_background_energy(rms)
                
                # VAD 状态机
                if self.vad_state == "silence":
                    # 始终保持历史缓冲区
                    self.speech_buffer.append(audio_data)
                    
                    # 检测声音活动（TTS期额外要求SNR门限）
                    if rms > dynamic_threshold and (not gated or snr_now >= SNR_GATE):
                        self.speech_counter += 1
                        if self.speech_counter == 1:
                            self.get_logger().debug(f"检测到潜在语音开始 (RMS: {rms:.4f}, 阈值: {dynamic_threshold:.4f}, SNR: {snr_now:.2f})")
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
                            
                            self.get_logger().info(f"🎤 检测到语音开始 (RMS: {rms:.4f}, 阈值: {dynamic_threshold:.4f}, SNR: {snr_now:.2f})")
                    else:
                        self.speech_counter = 0
                
                elif self.vad_state == "speech":
                    # 添加音频到当前语音缓冲区
                    self.current_speech.extend(audio_data)
                    
                    if rms <= dynamic_threshold * 0.8:
                        self.silence_counter += 1
                        silence_duration = self.silence_counter * CHUNK_SIZE / SAMPLE_RATE
                        
                        if silence_duration >= self.silence_duration:
                            # 检测到语音结束
                            self.vad_state = "processing"
                            speech_duration = len(self.current_speech) / (SAMPLE_RATE * 2)
                            self.get_logger().info(f"🔇 检测到语音结束 (时长: {speech_duration:.2f}s)")
                            
                            # 处理语音（过短忽略）
                            if speech_duration > 0.5:
                                processing_thread = threading.Thread(
                                    target=self.process_speech_chunk,
                                    args=(bytes(self.current_speech), gated),
                                    daemon=True
                                )
                                processing_thread.start()
                            
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
                    processing_thread = threading.Thread(
                        target=self.process_speech_chunk, 
                        args=(speech_data, gated),
                        daemon=True
                    )
                    processing_thread.start()
                    
                    self.current_speech = bytearray()
                    self.vad_state = "silence"

            except Exception as e:
                self.get_logger().error(f"音频处理错误: {e}")
                time.sleep(0.05)

    def process_speech_chunk(self, speech_data, gated=False):
        """处理检测到的语音片段"""
        # TTS期间且不允许打断：直接忽略
        if gated and not ALLOW_BARGE_IN:
            self.get_logger().debug("TTS期/挂起，不处理语音片段")
            return

        if len(speech_data) < SAMPLE_RATE * 2 * 0.3:  # 小于0.3秒的语音忽略
            self.get_logger().debug("语音片段太短，忽略")
            return

        # 计算片段SNR
        try:
            seg_np = np.frombuffer(speech_data, dtype=np.int16)
            seg_rms = 0.0 if seg_np.size == 0 else (np.sqrt(np.mean(seg_np.astype(np.float64) ** 2)) / 32768.0)
            bg = max(self.background_energy, 1e-6)
            snr_ratio = seg_rms / bg
        except Exception:
            snr_ratio = None

        try:
            # 创建临时WAV文件
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=True) as tmp:
                self.write_wav(tmp.name, speech_data)
                transcript = self.transcribe_file(tmp.name)
                if transcript:
                    self.process_recognized_text(transcript, snr_ratio)
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
                response = self.openai_client.audio.transcriptions.create(
                    model=MODEL_NAME,
                    file=f,
                    language="en",
                    # 提示词中强调唤醒词可能在句首
                    prompt="The captain is the wake word. Expect phrases starting with Hi Captain, Hey Captain, or Hello Captain.",
                    temperature=0,
                    response_format="text"
                )
            return response.strip() if isinstance(response, str) else response.strip()
        except Exception as e:
            self.get_logger().error(f"OpenAI API 调用失败: {e}")
            return ""

    def tts_status_callback(self, msg: Bool):
        """TTS状态回调：开启门控 + 挂起 + drain + 冻结背景能量"""
        if msg.data:
            self.get_logger().info("检测到 TTS 正在播放，暂停/门控监听")
            self.tts_playing = True
            self._freeze_noise = True
            self.resume_at = 0.0
            self._should_drain = True
        else:
            self.get_logger().info(f"TTS 播放完毕，延迟恢复监听 ({int(RESUME_HANGOVER_SEC*1000)}ms)")
            self.tts_playing = False
            self.resume_at = time.time() + RESUME_HANGOVER_SEC
            self._freeze_noise = True  # 挂起期仍冻结
            self._should_drain = True

    def llm_response_callback(self, msg: String):
        """缓存最近TTS文本，用于识别后兜底过滤"""
        text = (msg.data or '').strip()
        if not text:
            return
        # 在TTS播放或刚结束的短时间内才更新
        if self.tts_playing or (self.resume_at and time.time() < self.resume_at + 1.0):
            combined = (self._tts_recent_text + ' ' + text).strip()
            self._tts_recent_text = combined[-1000:]
            self._tts_recent_expiry = time.time() + 3.0

    def _normalize_text(self, s: str) -> str:
        s = s.lower().strip()
        s = re.sub(r'[^a-z0-9\s]+', ' ', s)
        s = re.sub(r'\s+', ' ', s)
        return s

    def process_recognized_text(self, text, snr_ratio: float | None = None):
        """处理识别的文本，检查唤醒词（仅句首、边界、SNR门限），并做TTS自回放兜底过滤"""
        if not text:
            return

        # 文本级兜底：若与最近TTS文本的前缀强包含，则丢弃
        now = time.time()
        if self._tts_recent_text and now < self._tts_recent_expiry:
            norm_ref = self._normalize_text(self._tts_recent_text)
            norm_txt = self._normalize_text(text)
            # 仅检查识别结果的前缀（最多6词）是否包含在TTS文本中，避免全句相似度误用
            words = norm_txt.split()
            prefix = ' '.join(words[:6])
            if prefix and prefix in norm_ref:
                self.get_logger().info("丢弃自回放文本(前缀包含于最近TTS)")
                return

        lower_text = text.lower().strip()

        # SNR 门限（可选但推荐）：过低则拒绝
        if snr_ratio is not None and snr_ratio < SNR_GATE:
            self.get_logger().info(f"拒绝低SNR片段 (SNR={snr_ratio:.2f} < {SNR_GATE})")
            return

        # 严格正则：句首 + 词边界
        if self.wake_regex.search(lower_text):
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)
            self.get_logger().info(f"🔥 识别结果 (唤醒词激活 - 正则): {text}")
            return

        # 仅句首模糊匹配（高阈值），不做全句比对
        for word in self.wake_words:
            n = len(word)
            candidate = lower_text[:max(n + 2, n)]  # 允许少量额外字符
            ratio = difflib.SequenceMatcher(None, candidate, word).ratio()
            if ratio >= 0.86:
                msg = String()
                msg.data = text
                self.publisher_.publish(msg)
                self.get_logger().info(f"🔥 识别结果 (唤醒词激活 - 句首模糊): {text}")
                return

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