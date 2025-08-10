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
# 允许通过环境变量覆盖，便于适配 USB 会议麦克风
SAMPLE_RATE = int(os.getenv("STT_SAMPLE_RATE", os.getenv("OPENAI_STT_SAMPLE_RATE", "24000")))
# 如果连接 48k 设备，建议 CHUNK_SIZE=960 (约20ms)；否则保持 1024 默认
_default_chunk = "960" if SAMPLE_RATE == 48000 else "1024"
CHUNK_SIZE = int(os.getenv("STT_CHUNK_SIZE", _default_chunk))
CHANNELS = int(os.getenv("STT_CHANNELS", "1"))  # 会议麦可能是双声道；下方做单声道下混
#MODEL_NAME = os.getenv("OPENAI_STT_MODEL", "whisper-1")
MODEL_NAME = os.getenv("OPENAI_STT_MODEL", "gpt-4o-mini-transcribe")

# VAD 配置 - 调整这些参数来改善唤醒词检测（支持环境变量覆盖）
VAD_THRESHOLD = float(os.getenv("STT_VAD_THRESHOLD", "0.015"))
SILENCE_DURATION = float(os.getenv("STT_SILENCE_DURATION", "2.0"))
MIN_SPEECH_DURATION = float(os.getenv("STT_MIN_SPEECH_DURATION", "0.2"))
BUFFER_HISTORY = float(os.getenv("STT_BUFFER_HISTORY", "1.5"))

# 新增：门控/阈值/相似度/允许打断配置
RESUME_HANGOVER_SEC = float(os.getenv("STT_RESUME_HANGOVER_SEC", "0.8"))
TTS_VAD_THRESHOLD_BOOST = float(os.getenv("TTS_VAD_THRESHOLD_BOOST", "2.0"))
SNR_GATE = float(os.getenv("STT_WAKEWORD_SNR_GATE", "1.8"))  # 片段RMS/背景能量 比值门限
ALLOW_BARGE_IN = os.getenv("STT_ALLOW_BARGE_IN", "false").lower() == "true"
# 可选：选择指定输入设备（名称包含匹配 或 设备索引）
INPUT_DEVICE_NAME = os.getenv("STT_INPUT_DEVICE_NAME", "").strip()
INPUT_DEVICE_INDEX = os.getenv("STT_INPUT_DEVICE_INDEX", "").strip()
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

        # 允许用户调整VAD参数（仍支持通过 ROS 参数动态设置）
        self.declare_parameter('vad_threshold', VAD_THRESHOLD)
        self.declare_parameter('silence_duration', SILENCE_DURATION)
        self.declare_parameter('min_speech_duration', MIN_SPEECH_DURATION)
        self.declare_parameter('buffer_history', BUFFER_HISTORY)
        
        # 获取参数
        self.vad_threshold = self.get_parameter('vad_threshold').value
        self.silence_duration = self.get_parameter('silence_duration').value
        self.min_speech_duration = self.get_parameter('min_speech_duration').value
        self.buffer_history = self.get_parameter('buffer_history').value

        # 设备选择与日志
        self.audio = pyaudio.PyAudio()
        chosen_index = None
        try:
            device_count = self.audio.get_device_count()
            self.get_logger().info(f"检测到音频输入设备数量: {device_count}")
            for i in range(device_count):
                info = self.audio.get_device_info_by_index(i)
                name = info.get('name', 'unknown')
                host_api = info.get('hostApi')
                max_in = int(info.get('maxInputChannels', 0))
                rate = int(info.get('defaultSampleRate', 0))
                self.get_logger().info(f"输入设备[{i}]: name='{name}', maxIn={max_in}, defaultSR={rate}")
            # 选择逻辑：优先索引，其次名称包含
            if INPUT_DEVICE_INDEX.isdigit():
                chosen_index = int(INPUT_DEVICE_INDEX)
            elif INPUT_DEVICE_NAME:
                needle = INPUT_DEVICE_NAME.lower()
                for i in range(device_count):
                    info = self.audio.get_device_info_by_index(i)
                    if needle in info.get('name', '').lower() and int(info.get('maxInputChannels', 0)) >= CHANNELS:
                        chosen_index = i
                        break
        except Exception as e:
            self.get_logger().warning(f"音频设备枚举失败: {e}")
            chosen_index = None

        # 初始化音频输入（支持多声道，随后统一下混为单声道）
        self.channels = CHANNELS
        self.sample_rate = SAMPLE_RATE
        self.chunk_size = CHUNK_SIZE
        try:
            self.stream = self.audio.open(
                format=pyaudio.paInt16,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size,
                input_device_index=chosen_index if chosen_index is not None else None,
            )
            dev_desc = f"index={chosen_index}" if chosen_index is not None else "default"
            self.get_logger().info(
                f"开始监听麦克风 (VAD), device={dev_desc}, ch={self.channels}, SR={self.sample_rate}, chunk={self.chunk_size}"
            )
        except Exception as e:
            self.get_logger().error(
                f"打开音频输入失败: {e}. 请尝试设置 STT_INPUT_DEVICE_NAME 或 STT_INPUT_DEVICE_INDEX，"
                f"并确保 STT_SAMPLE_RATE/ STT_CHANNELS 与设备支持的参数匹配（常见 SR: 16000/24000/48000）。"
            )
            raise
        self.stream.start_stream()

        # 设置唤醒词与严格正则（仅句首、词边界）
        self.wake_words = ["hi captain", "hey captain", "hello captain"]
        self.wake_regex = re.compile(r'^\s*(hi|hey|hello)\W+captain\b', re.I)

        # VAD 相关变量
        self.vad_state = "silence"  # "silence", "speech", "processing"
        self.speech_buffer = collections.deque(maxlen=int(self.sample_rate * self.buffer_history / self.chunk_size))
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

        self.get_logger().info(
            f"VAD 配置: 阈值={self.vad_threshold}, 静音检测={self.silence_duration}s, 前缓冲区={self.buffer_history}s, 最小语音={self.min_speech_duration}s"
        )

    def _to_mono(self, audio_bytes: bytes) -> bytes:
        """将多声道int16 PCM下混为单声道，以便VAD与转写。"""
        if self.channels == 1:
            return audio_bytes
        try:
            pcm = np.frombuffer(audio_bytes, dtype=np.int16)
            if pcm.size % self.channels != 0:
                pcm = pcm[: (pcm.size // self.channels) * self.channels]
            frames = pcm.reshape(-1, self.channels).astype(np.int32)
            mono = (frames.mean(axis=1)).astype(np.int16)
            return mono.tobytes()
        except Exception:
            return audio_bytes  # 失败时回退

    def calculate_rms(self, audio_data):
        """计算音频RMS (均方根) 用于VAD（假定单声道字节）"""
        audio_np = np.frombuffer(audio_data, dtype=np.int16)
        if audio_np.size == 0:
            return 0.0
        rms = np.sqrt(np.mean(audio_np.astype(np.float64) ** 2))
        return rms / 32768.0

    def update_background_energy(self, rms):
        """动态阈值：非冻结时更新背景噪声；TTS期提升阈值"""
        if not self._freeze_noise:
            self.energy_history.append(rms)
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
                raw = self.stream.read(self.chunk_size, exception_on_overflow=False)
            except Exception as e:
                self.get_logger().error(f"音频读取错误: {e}")
                time.sleep(0.05)
                continue

            # 统一转为单声道处理
            audio_data = self._to_mono(raw)

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
                    # 始终保持历史缓冲区（已为单声道）
                    self.speech_buffer.append(audio_data)
                    
                    # 检测声音活动（TTS期额外要求SNR门限）
                    if rms > dynamic_threshold and (not gated or snr_now >= SNR_GATE):
                        self.speech_counter += 1
                        if self.speech_counter == 1:
                            self.get_logger().debug(f"检测到潜在语音开始 (RMS: {rms:.4f}, 阈值: {dynamic_threshold:.4f}, SNR: {snr_now:.2f})")
                        if self.speech_counter >= int(self.min_speech_duration * self.sample_rate / self.chunk_size):
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
                    # 添加音频到当前语音缓冲区（单声道）
                    self.current_speech.extend(audio_data)
                    
                    if rms <= dynamic_threshold * 0.8:
                        self.silence_counter += 1
                        silence_duration = self.silence_counter * self.chunk_size / self.sample_rate
                        
                        if silence_duration >= self.silence_duration:
                            # 检测到语音结束
                            self.vad_state = "processing"
                            speech_duration = len(self.current_speech) / (self.sample_rate * 2)
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
        """处理检测到的语音片段（speech_data 已为单声道PCM）"""
        # TTS期间且不允许打断：直接忽略
        if gated and not ALLOW_BARGE_IN:
            self.get_logger().debug("TTS期/挂起，不处理语音片段")
            return

        if len(speech_data) < self.sample_rate * 2 * 0.3:  # 小于0.3秒的语音忽略
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
            # 创建临时WAV文件（单声道）
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=True) as tmp:
                self.write_wav(tmp.name, speech_data)
                transcript = self.transcribe_file(tmp.name)
                if transcript:
                    self.process_recognized_text(transcript, snr_ratio)
        except Exception as e:
            self.get_logger().error(f"语音处理错误: {e}")

    def write_wav(self, path: str, pcm_bytes: bytes):
        """写 WAV 工具（单声道）"""
        with wave.open(path, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(self.sample_rate)
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
                    # language="en",
                    # prompt="The captain is the wake word. Expect phrases starting with Hi Captain, Hey Captain, or Hello Captain.",
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