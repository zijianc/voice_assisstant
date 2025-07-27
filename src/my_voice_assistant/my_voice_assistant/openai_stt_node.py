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

# 加载 .env 文件
load_dotenv()

# -----------------------------------------------------------------------------
# Audio configuration
SAMPLE_RATE = 24000          # Whisper 默认 24 kHz 也 OK 16 kHz
CHUNK_SECONDS = 5            # 每块时长
MODEL_NAME = os.getenv("OPENAI_STT_MODEL", "whisper-1")  # 或 gpt-4o-mini-transcribe
# -----------------------------------------------------------------------------

class OpenAISTTNode(Node):
    def __init__(self):
        super().__init__('openai_stt_node')
        
        # 初始化 OpenAI 客户端
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise RuntimeError("请设置 OPENAI_API_KEY 环境变量")
        
        self.openai_client = openai.OpenAI(api_key=api_key)
        
        # 创建发布者，发布识别结果到话题 'speech_text'
        self.publisher_ = self.create_publisher(String, 'speech_text', 10)
        # 订阅 TTS 状态消息（假设 TTS 节点发布 Bool 消息，True 表示正在播放）
        self.tts_status_sub = self.create_subscription(Bool, 'tts_status', self.tts_status_callback, 10)
        self.listening = True  # 用于控制是否监听

        # 初始化音频输入
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=SAMPLE_RATE,
            input=True,
            frames_per_buffer=2048  # ≈85 ms at 24 kHz
        )
        self.stream.start_stream()
        self.get_logger().info("开始监听麦克风...")

        # 设置唤醒词（转换为小写用于匹配）
        self.wake_words = ["hi captain", "hey captain", "hello captain"]

        # 创建音频队列和线程
        self.audio_queue = queue.Queue()
        self.audio_thread = threading.Thread(target=self.audio_reader_thread, daemon=True)
        self.audio_thread.start()

        self.recognition_thread = threading.Thread(target=self.recognition_loop, daemon=True)
        self.recognition_thread.start()

    def tts_status_callback(self, msg: Bool):
        # 假设 TTS 节点发布 True 表示正在播放，False 表示播放完毕
        if msg.data:
            self.get_logger().info("检测到 TTS 正在播放，暂停监听")
            self.listening = False
        else:
            self.get_logger().info("TTS 播放完毕，恢复监听")
            self.listening = True

    def process_recognized_text(self, text):
        lower_text = text.lower()
        match_found = False
        for word in self.wake_words:
            if word in lower_text:
                match_found = True
                break
            ratio = difflib.SequenceMatcher(None, lower_text, word).ratio()
            if ratio > 0.75:
                match_found = True
                break
        if match_found:
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)
            self.get_logger().info("识别结果 (唤醒词激活): " + text)
        else:
            self.get_logger().info("未检测到唤醒词: " + text)

    def audio_reader_thread(self):
        while rclpy.ok():
            try:
                data = self.stream.read(1024, exception_on_overflow=False)
                self.audio_queue.put(data)
            except Exception as e:
                self.get_logger().error("读取音频流错误: {}".format(e))
                time.sleep(0.05)

    def recognition_loop(self):
        pcm_buf = bytearray()
        last_publish = time.time()

        while rclpy.ok():
            # 检查暂停
            if not self.listening:
                time.sleep(0.1)
                continue

            # 从队列拿数据
            try:
                data = self.audio_queue.get(timeout=1)
            except queue.Empty:
                continue
            
            pcm_buf.extend(data)

            # 到达块大小就发送
            current_time = time.time()
            if len(pcm_buf) >= SAMPLE_RATE * 2 * CHUNK_SECONDS or (current_time - last_publish) >= CHUNK_SECONDS:
                if len(pcm_buf) > 0:
                    try:
                        with tempfile.NamedTemporaryFile(suffix=".wav", delete=True) as tmp:
                            self.write_wav(tmp.name, pcm_buf)
                            transcript = self.transcribe_file(tmp.name)
                            if transcript:
                                self.process_recognized_text(transcript)
                    except Exception as e:
                        self.get_logger().error("转录错误: {}".format(e))
                    
                    pcm_buf.clear()
                    last_publish = current_time

    def write_wav(self, path: str, pcm_bytes: bytes):
        """写 WAV 工具"""
        with wave.open(path, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)          # paInt16
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
                    prompt="The captain is the wakeword.",
                    temperature=0,
                    response_format="text"   # 纯文本
                )
            return response.strip() if isinstance(response, str) else response.strip()
        except Exception as e:
            self.get_logger().error("OpenAI API 调用失败: {}".format(e))
            return ""

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
    node = OpenAISTTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
