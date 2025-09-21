#!/usr/bin/env python3
"""
TEN VAD STT Node - 使用TEN VAD替代RMS能量VAD
====================================================

这个节点使用TEN VAD (https://github.com/TEN-framework/ten-vad) 
替代传统的RMS能量+静音检测VAD，提供更精确的语音活动检测。

TEN VAD优势：
- 比WebRTC VAD和Silero VAD更精确
- 专为对话AI设计，低延迟
- 轻量级，计算复杂度低
- 快速检测语音到非语音转换
- 能识别短暂静音

关键差异对比RMS VAD：
- RMS VAD: 基于能量阈值，容易受噪声影响
- TEN VAD: 基于深度学习，帧级语音活动检测
"""

import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import pyaudio
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

# TEN VAD import
try:
    import sys
    print(f"Python路径: {sys.executable}")
    print(f"Python版本: {sys.version}")
    print(f"模块搜索路径: {sys.path[:3]}...")  # 只显示前3个路径
    
    from ten_vad import TenVad
    TEN_VAD_AVAILABLE = True
    print("✅ TEN VAD 导入成功")
except ImportError as e:
    TEN_VAD_AVAILABLE = False
    print(f"❌ TEN VAD 导入失败: {e}")
    print("请运行: pip install git+https://github.com/TEN-framework/ten-vad.git")

# 加载环境变量
load_dotenv()

# -----------------------------------------------------------------------------
# Audio configuration - TEN VAD 要求16kHz采样率
SAMPLE_RATE = 16000  # TEN VAD 固定要求16kHz
HOP_SIZE = 256  # TEN VAD 默认帧大小 (16ms at 16kHz)
CHANNELS = int(os.getenv("STT_CHANNELS", "1"))  # 单声道
MODEL_NAME = os.getenv("OPENAI_STT_MODEL", "gpt-4o-mini-transcribe")

# TEN VAD配置 - 针对嘈杂环境优化
TEN_VAD_THRESHOLD = float(os.getenv("TEN_VAD_THRESHOLD", "0.8"))  # 提高阈值应对噪音
MIN_VOICE_FRAMES = int(os.getenv("TEN_MIN_VOICE_FRAMES", "8"))  # 增加连续帧要求
MAX_SILENCE_FRAMES = int(os.getenv("TEN_MAX_SILENCE_FRAMES", "75"))  # 最大静音帧数(约1.2秒)
BUFFER_HISTORY_FRAMES = int(os.getenv("TEN_BUFFER_HISTORY_FRAMES", "30"))  # 前缓冲帧数

# 音频质量过滤 - 加强噪音过滤
MIN_AUDIO_ENERGY = float(os.getenv("TEN_MIN_AUDIO_ENERGY", "200"))  # 提高最小能量要求
MIN_SPEECH_DURATION_MS = float(os.getenv("TEN_MIN_SPEECH_DURATION_MS", "500"))  # 增加最短语音时长
NOISE_FLOOR_ADAPTATION = float(os.getenv("TEN_NOISE_FLOOR_ADAPTATION", "0.3"))  # 噪音底噪自适应

# 唤醒词配置
WAKE_WORD_SIMILARITY = float(os.getenv("WAKE_WORD_SIMILARITY_THRESHOLD", "0.86"))

# TTS门控配置
RESUME_HANGOVER_SEC = float(os.getenv("STT_RESUME_HANGOVER_SEC", "0.8"))
ALLOW_BARGE_IN = os.getenv("STT_ALLOW_BARGE_IN", "false").lower() == "true"
TTS_ECHO_FILTER_DURATION = float(os.getenv("TTS_ECHO_FILTER_DURATION", "3.0"))

# 性能监控
PERFORMANCE_MONITOR_INTERVAL = float(os.getenv("PERFORMANCE_MONITOR_INTERVAL", "30"))
DEBUG_MODE = os.getenv("TEN_VAD_DEBUG_MODE", "false").lower() == "true"

# 设备选择
INPUT_DEVICE_NAME = os.getenv("STT_INPUT_DEVICE_NAME", "").strip()
INPUT_DEVICE_INDEX = os.getenv("STT_INPUT_DEVICE_INDEX", "").strip()
# -----------------------------------------------------------------------------

class TenVADSTTNode(Node):
    def __init__(self):
        super().__init__('ten_vad_stt_node')
        
        # 检查TEN VAD可用性
        if not TEN_VAD_AVAILABLE:
            raise RuntimeError("TEN VAD 未安装。请运行: pip install git+https://github.com/TEN-framework/ten-vad.git")
        
        # 初始化OpenAI客户端
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise RuntimeError("请设置 OPENAI_API_KEY 环境变量")
        
        self.openai_client = openai.OpenAI(api_key=api_key)
        
        # ROS2 发布者和订阅者
        self.publisher_ = self.create_publisher(String, 'speech_text', 10)
        self.tts_status_sub = self.create_subscription(Bool, 'tts_status', self.tts_status_callback, 10)
        self.tts_text_sub = self.create_subscription(String, 'llm_response', self.llm_response_callback, 10)
        
        # 初始化TEN VAD
        try:
            self.ten_vad = TenVad(hop_size=HOP_SIZE, threshold=TEN_VAD_THRESHOLD)
            self.get_logger().info(f"TEN VAD 初始化成功: hop_size={HOP_SIZE}, threshold={TEN_VAD_THRESHOLD}")
            
            # 自适应阈值管理
            self.adaptive_threshold = TEN_VAD_THRESHOLD
            self.false_positive_count = 0
            self.successful_detections = 0
            self.threshold_adjustment_window = 50  # 每50次检测调整一次
            
        except Exception as e:
            self.get_logger().error(f"TEN VAD 初始化失败: {e}")
            raise
        
        # 音频设备设置
        self.audio = pyaudio.PyAudio()
        self.stream = None
        self._setup_audio_input()
        
        # 唤醒词配置
        self.wake_words = ["nUWAy", "new way", "nu way", "en you way", "nuway", "n u way"]
        self.wake_regex = re.compile(r'^\s*(n\s*u\s*way|new\s*way|nu\s*way|en\s*you\s*way|nuway)\b', re.I)
        
        # VAD状态管理
        self.vad_state = "silence"  # "silence", "speech", "processing"
        self.speech_buffer = collections.deque(maxlen=BUFFER_HISTORY_FRAMES)
        self.current_speech_frames = []
        self.voice_frame_count = 0
        self.silence_frame_count = 0
        
        # 噪音自适应管理
        self.noise_floor_history = collections.deque(maxlen=100)  # 保存最近100帧的能量
        self.current_noise_floor = MIN_AUDIO_ENERGY
        self.noise_update_counter = 0
        
        # 优化：预分配音频缓冲区，避免频繁内存分配
        self.max_speech_frames = int(SAMPLE_RATE * 10 / HOP_SIZE)  # 最多10秒语音
        self.audio_buffer_pool = []
        
        # TTS门控状态
        self.tts_playing = False
        self.resume_at = 0.0
        self._should_reset = False
        
        # TTS回放过滤
        self._tts_recent_text = ""
        self._tts_recent_expiry = 0.0
        
        # 性能监控
        self.performance_stats = {
            'total_frames_processed': 0,
            'speech_frames_detected': 0,
            'transcription_attempts': 0,
            'successful_transcriptions': 0,
            'average_processing_time': 0.0,
            'last_reset_time': time.time()
        }
        
        # 启动音频处理线程
        self.audio_thread = threading.Thread(target=self._audio_processing_thread, daemon=True)
        self.audio_thread.start()
        
        # 启动性能监控线程
        self.stats_thread = threading.Thread(target=self._performance_monitoring_thread, daemon=True)
        self.stats_thread.start()
        
        self.get_logger().info(
            f"TEN VAD STT 节点启动完成："
            f" 采样率={SAMPLE_RATE}Hz, 帧大小={HOP_SIZE}, "
            f"最小语音帧={MIN_VOICE_FRAMES}, 最大静音帧={MAX_SILENCE_FRAMES}"
        )

    def _setup_audio_input(self):
        """设置音频输入设备"""
        chosen_index = None
        
        try:
            device_count = self.audio.get_device_count()
            self.get_logger().info(f"检测到音频输入设备数量: {device_count}")
            
            for i in range(device_count):
                info = self.audio.get_device_info_by_index(i)
                name = info.get('name', 'unknown')
                max_in = int(info.get('maxInputChannels', 0))
                rate = int(info.get('defaultSampleRate', 0))
                self.get_logger().info(f"设备[{i}]: '{name}', 输入通道={max_in}, 默认采样率={rate}")
            
            # 设备选择逻辑
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

        # 初始化音频流
        try:
            self.stream = self.audio.open(
                format=pyaudio.paInt16,
                channels=CHANNELS,
                rate=SAMPLE_RATE,
                input=True,
                frames_per_buffer=HOP_SIZE,
                input_device_index=chosen_index if chosen_index is not None else None,
            )
            
            dev_desc = f"设备索引={chosen_index}" if chosen_index is not None else "默认设备"
            self.get_logger().info(f"音频输入已开启: {dev_desc}, 采样率={SAMPLE_RATE}, 帧大小={HOP_SIZE}")
            
        except Exception as e:
            self.get_logger().error(
                f"音频输入开启失败: {e}. 这在容器环境中是正常的。"
                f"在生产环境中，请检查 STT_INPUT_DEVICE_NAME 或 STT_INPUT_DEVICE_INDEX 设置，"
                f"确保设备支持 16kHz 采样率。"
            )
            
            # 在容器环境中创建模拟流用于测试
            self.get_logger().warning("⚠️  使用模拟音频流进行测试（容器环境）")
            self.stream = None  # 标记为模拟模式
            return

    def _to_mono(self, audio_bytes: bytes) -> bytes:
        """将多声道音频转换为单声道"""
        if CHANNELS == 1:
            return audio_bytes
        
        try:
            pcm = np.frombuffer(audio_bytes, dtype=np.int16)
            if pcm.size % CHANNELS != 0:
                pcm = pcm[:(pcm.size // CHANNELS) * CHANNELS]
            frames = pcm.reshape(-1, CHANNELS).astype(np.int32)
            mono = frames.mean(axis=1).astype(np.int16)
            return mono.tobytes()
        except Exception:
            return audio_bytes

    def _audio_processing_thread(self):
        """音频处理线程 - 使用TEN VAD进行语音活动检测"""
        while rclpy.ok():
            try:
                # 读取音频数据
                raw_audio = self.stream.read(HOP_SIZE, exception_on_overflow=False)
                audio_data = self._to_mono(raw_audio)
                
                current_time = time.time()
                
                # TTS门控检查
                gated = self.tts_playing or (self.resume_at and current_time < self.resume_at)
                
                # 门控期间重置状态
                if gated and self._should_reset:
                    self.speech_buffer.clear()
                    self.current_speech_frames.clear()
                    self.voice_frame_count = 0
                    self.silence_frame_count = 0
                    self.vad_state = "silence"
                    self._should_reset = False
                    self.get_logger().debug("TTS期间重置VAD状态")
                
                # 如果门控且不允许打断，跳过处理
                if gated and not ALLOW_BARGE_IN:
                    time.sleep(0.01)
                    continue
                
                # 转换为numpy数组进行TEN VAD处理
                audio_np = np.frombuffer(audio_data, dtype=np.int16)
                
                # 使用TEN VAD处理音频帧
                voice_probability, voice_flag = self.ten_vad.process(audio_np)
                
                # 更新性能统计
                self.performance_stats['total_frames_processed'] += 1
                if voice_flag == 1:
                    self.performance_stats['speech_frames_detected'] += 1
                
                # VAD状态机处理
                self._process_vad_result(voice_probability, voice_flag, audio_data, gated)
                
            except Exception as e:
                self.get_logger().error(f"音频处理错误: {e}")
                time.sleep(0.05)

    def _process_vad_result(self, voice_prob: float, voice_flag: int, audio_data: bytes, gated: bool):
        """处理TEN VAD结果并管理语音状态"""
        
        # 更新噪音底噪估计（仅在静音期间）
        if self.vad_state == "silence" and voice_flag == 0:
            self._update_noise_floor(audio_data)
        
        # 使用自适应能量阈值进行额外过滤
        adaptive_energy_threshold = max(self.current_noise_floor * 2.0, MIN_AUDIO_ENERGY)
        
        if self.vad_state == "silence":
            # 始终维护历史缓冲区
            self.speech_buffer.append(audio_data)
            
            if voice_flag == 1:  # TEN VAD检测到语音
                # 额外的能量检查：确保音频能量足够高
                try:
                    audio_np = np.frombuffer(audio_data, dtype=np.int16)
                    current_energy = np.sqrt(np.mean(audio_np.astype(np.float32) ** 2))
                    
                    # 同时满足TEN VAD和能量条件
                    if current_energy >= adaptive_energy_threshold:
                        self.voice_frame_count += 1
                        if self.voice_frame_count == 1:
                            self.get_logger().debug(f"检测到潜在语音开始 (概率: {voice_prob:.3f}, 能量: {current_energy:.1f})")
                    else:
                        # 能量不足，重置计数
                        self.voice_frame_count = 0
                        if DEBUG_MODE:
                            self.get_logger().debug(f"能量不足，忽略VAD检测 (能量: {current_energy:.1f} < {adaptive_energy_threshold:.1f})")
                        return
                        
                except Exception:
                    # 如果能量计算失败，仍然使用TEN VAD结果
                    self.voice_frame_count += 1
                    if self.voice_frame_count == 1:
                        self.get_logger().debug(f"检测到潜在语音开始 (概率: {voice_prob:.3f})")
                
                # 连续语音帧数达到阈值，确认语音开始
                if self.voice_frame_count >= MIN_VOICE_FRAMES:
                    self.vad_state = "speech"
                    self.voice_frame_count = 0
                    self.silence_frame_count = 0
                    
                    # 将历史缓冲区添加到当前语音
                    self.current_speech_frames = list(self.speech_buffer)
                    
                    # 打印语音开始提示
                    print(f"\n🎤 开始录音... (语音概率: {voice_prob:.3f})")
                    
                    self.get_logger().info(f"🎤 TEN VAD: 语音开始 (概率: {voice_prob:.3f})")
            else:
                self.voice_frame_count = 0
                
        elif self.vad_state == "speech":
            # 添加音频帧到当前语音
            self.current_speech_frames.append(audio_data)
            
            if voice_flag == 0:  # 检测到静音
                self.silence_frame_count += 1
                silence_duration_ms = self.silence_frame_count * 16  # 每帧16ms
                
                # 静音持续时间超过阈值，确认语音结束
                if self.silence_frame_count >= MAX_SILENCE_FRAMES:
                    self.vad_state = "processing"
                    speech_duration_ms = len(self.current_speech_frames) * 16
                    
                    # 打印语音结束提示
                    print(f"🔇 录音结束，正在识别... (时长: {speech_duration_ms}ms)")
                    
                    self.get_logger().info(f"🔇 TEN VAD: 语音结束 (时长: {speech_duration_ms}ms)")
                    
                    # 异步处理语音
                    if len(self.current_speech_frames) > 10:  # 最少处理约160ms的语音
                        speech_data = b''.join(self.current_speech_frames)
                        processing_thread = threading.Thread(
                            target=self._process_speech_chunk,
                            args=(speech_data, gated),
                            daemon=True
                        )
                        processing_thread.start()
                    
                    # 重置状态
                    self.current_speech_frames.clear()
                    self.silence_frame_count = 0
                    self.vad_state = "silence"
            else:
                self.silence_frame_count = 0  # 重置静音计数

    def _update_noise_floor(self, audio_data: bytes):
        """更新噪音底噪估计"""
        try:
            audio_np = np.frombuffer(audio_data, dtype=np.int16)
            current_energy = np.sqrt(np.mean(audio_np.astype(np.float32) ** 2))
            
            # 添加到历史记录
            self.noise_floor_history.append(current_energy)
            self.noise_update_counter += 1
            
            # 每50帧更新一次噪音底噪估计
            if self.noise_update_counter >= 50 and len(self.noise_floor_history) >= 20:
                # 使用第10百分位数作为噪音底噪
                self.current_noise_floor = np.percentile(list(self.noise_floor_history), 10)
                self.noise_update_counter = 0
                
                if DEBUG_MODE:
                    self.get_logger().debug(f"更新噪音底噪: {self.current_noise_floor:.1f}")
                    
        except Exception as e:
            if DEBUG_MODE:
                self.get_logger().debug(f"噪音底噪更新失败: {e}")

    def _process_speech_chunk(self, speech_data: bytes, gated: bool = False):
        """处理检测到的语音片段"""
        start_time = time.time()
        
        # 更新性能统计
        self.performance_stats['transcription_attempts'] += 1
        
        # TTS期间且不允许打断
        if gated and not ALLOW_BARGE_IN:
            self.get_logger().debug("TTS期间跳过语音处理")
            return

        # 检查语音长度和质量
        speech_duration_ms = len(speech_data) / (SAMPLE_RATE * 2) * 1000
        if speech_duration_ms < MIN_SPEECH_DURATION_MS:
            self.get_logger().debug(f"语音片段过短 ({speech_duration_ms:.0f}ms < {MIN_SPEECH_DURATION_MS}ms)，忽略")
            return
        
        # 简单的音频质量检查（RMS能量）
        try:
            audio_np = np.frombuffer(speech_data, dtype=np.int16)
            rms_energy = np.sqrt(np.mean(audio_np.astype(np.float32) ** 2))
            if rms_energy < MIN_AUDIO_ENERGY:
                self.get_logger().debug(f"音频能量过低 (RMS: {rms_energy:.1f} < {MIN_AUDIO_ENERGY})，跳过")
                return
            
            if DEBUG_MODE:
                self.get_logger().info(f"🔊 处理语音: {speech_duration_ms:.0f}ms, RMS: {rms_energy:.1f}")
                
        except Exception:
            pass

        try:
            # 创建临时WAV文件
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=True) as tmp:
                self._write_wav(tmp.name, speech_data)
                
                # 打印正在转录提示
                print("🔄 正在转录语音...")
                
                transcript = self._transcribe_file(tmp.name)
                
                if transcript:
                    self.performance_stats['successful_transcriptions'] += 1
                    self._process_recognized_text(transcript)
                
                # 更新平均处理时间
                processing_time = time.time() - start_time
                current_avg = self.performance_stats['average_processing_time']
                total_attempts = self.performance_stats['transcription_attempts']
                self.performance_stats['average_processing_time'] = (
                    (current_avg * (total_attempts - 1) + processing_time) / total_attempts
                )
                
        except Exception as e:
            self.get_logger().error(f"语音处理错误: {e}")

    def _write_wav(self, path: str, pcm_bytes: bytes):
        """写入WAV文件"""
        with wave.open(path, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(SAMPLE_RATE)
            wf.writeframes(pcm_bytes)

    @tenacity.retry(stop=tenacity.stop_after_attempt(3),
                    wait=tenacity.wait_exponential(multiplier=1, min=1, max=10))
    def _transcribe_file(self, fname: str) -> str:
        """使用OpenAI Whisper转录音频"""
        try:
            with open(fname, "rb") as f:
                response = self.openai_client.audio.transcriptions.create(
                    model=MODEL_NAME,
                    file=f,
                    temperature=0,
                    response_format="text"
                )
            return response.strip() if isinstance(response, str) else response.strip()
        except Exception as e:
            self.get_logger().error(f"OpenAI API 调用失败: {e}")
            return ""

    def _process_recognized_text(self, text: str):
        """处理识别的文本，检查唤醒词"""
        if not text:
            return

        # TTS回放过滤
        now = time.time()
        if self._tts_recent_text and now < self._tts_recent_expiry:
            norm_ref = self._normalize_text(self._tts_recent_text)
            norm_txt = self._normalize_text(text)
            words = norm_txt.split()
            prefix = ' '.join(words[:6])
            if prefix and prefix in norm_ref:
                self.get_logger().info("丢弃TTS回放文本")
                return

        lower_text = text.lower().strip()

        # 严格正则匹配唤醒词
        if self.wake_regex.search(lower_text):
            print("\n🔥 检测到唤醒词 (正则匹配)!")
            self._publish_transcript(text)
            self.get_logger().info(f"🔥 唤醒词检测成功 (正则): {text}")
            return

        # 模糊匹配句首唤醒词
        for wake_word in self.wake_words:
            n = len(wake_word)
            candidate = lower_text[:max(n + 2, n)]
            ratio = difflib.SequenceMatcher(None, candidate, wake_word).ratio()
            if ratio >= WAKE_WORD_SIMILARITY:
                print(f"\n🔥 检测到唤醒词 (模糊匹配 {ratio:.2f})!")
                self._publish_transcript(text)
                self.get_logger().info(f"🔥 唤醒词检测成功 (模糊 {ratio:.2f}): {text}")
                return

        if DEBUG_MODE:
            self.get_logger().info(f"未检测到唤醒词: {text}")

    def _publish_transcript(self, transcript: str):
        """发布转录结果到ROS话题"""
        
        # 打印识别到的语音
        print("\n" + "="*60)
        print("🎤 用户语音识别结果:")
        print("-"*60)
        print(f"'{transcript}'")
        print("="*60)
        
        msg = String()
        msg.data = transcript
        self.publisher_.publish(msg)

    def _normalize_text(self, text: str) -> str:
        """文本规范化"""
        text = text.lower().strip()
        text = re.sub(r'[^a-z0-9\s]+', ' ', text)
        text = re.sub(r'\s+', ' ', text)
        return text

    def _performance_monitoring_thread(self):
        """性能监控线程，每30秒输出统计信息"""
        if PERFORMANCE_MONITOR_INTERVAL <= 0:
            return  # 禁用性能监控
            
        while rclpy.ok():
            try:
                time.sleep(PERFORMANCE_MONITOR_INTERVAL)
                
                stats = self.performance_stats
                current_time = time.time()
                elapsed = current_time - stats['last_reset_time']
                
                if stats['total_frames_processed'] > 0:
                    speech_ratio = stats['speech_frames_detected'] / stats['total_frames_processed']
                    success_ratio = (stats['successful_transcriptions'] / max(1, stats['transcription_attempts']))
                    
                    self.get_logger().info(
                        f"📊 性能统计 ({elapsed:.1f}s): "
                        f"语音帧率={speech_ratio:.1%}, "
                        f"转录成功率={success_ratio:.1%}, "
                        f"平均处理时间={stats['average_processing_time']:.3f}s, "
                        f"总处理帧={stats['total_frames_processed']}"
                    )
                
            except Exception as e:
                self.get_logger().warning(f"性能监控错误: {e}")

    def tts_status_callback(self, msg: Bool):
        """TTS状态回调"""
        if msg.data:
            self.get_logger().info("TTS播放开始，启用门控")
            self.tts_playing = True
            self.resume_at = 0.0
            self._should_reset = True
        else:
            self.get_logger().info(f"TTS播放结束，{int(RESUME_HANGOVER_SEC*1000)}ms后恢复")
            self.tts_playing = False
            self.resume_at = time.time() + RESUME_HANGOVER_SEC
            self._should_reset = True

    def llm_response_callback(self, msg: String):
        """缓存LLM响应文本用于回放过滤"""
        text = (msg.data or '').strip()
        if not text:
            return
        
        if self.tts_playing or (self.resume_at and time.time() < self.resume_at + 1.0):
            combined = (self._tts_recent_text + ' ' + text).strip()
            self._tts_recent_text = combined[-1000:]
            self._tts_recent_expiry = time.time() + TTS_ECHO_FILTER_DURATION

    def destroy_node(self):
        """清理资源"""
        self.get_logger().info("关闭TEN VAD STT节点...")
        
        if hasattr(self, 'stream') and self.stream:
            self.stream.stop_stream()
            self.stream.close()
        
        if hasattr(self, 'audio'):
            self.audio.terminate()
        
        # 清理TEN VAD资源
        if hasattr(self, 'ten_vad'):
            del self.ten_vad
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TenVADSTTNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点启动失败: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
