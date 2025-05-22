import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import pyaudio
import vosk
import queue
import threading
import time
import difflib
from ament_index_python.packages import get_package_share_directory

class VoskSTTNode(Node):
    def __init__(self):
        super().__init__('vosk_stt_node')
        # 创建发布者，发布识别结果到话题 'speech_text'
        self.publisher_ = self.create_publisher(String, 'speech_text', 10)
        # 订阅 TTS 状态消息（假设 TTS 节点发布 Bool 消息，True 表示正在播放）
        self.tts_status_sub = self.create_subscription(Bool, 'tts_status', self.tts_status_callback, 10)
        self.listening = True  # 用于控制是否监听

        # 获取当前文件所在目录，设置英语模型路径
        model_path_en = os.environ.get("VOSK_MODEL_EN_PATH",
                                       "/workspaces/ros2_ws/src/my_voice_assistant/models/vosk-model-en-us-0.22")
        self.get_logger().info("加载英语模型：{}".format(model_path_en))
        self.model_en = vosk.Model(model_path_en)
        self.recognizer_en = vosk.KaldiRecognizer(self.model_en, 44100)

        # 初始化音频输入
        self.last_partial = ""
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=44100,
            input=True,
            frames_per_buffer=4096
        )
        self.stream.start_stream()
        self.get_logger().info("开始监听麦克风...")

        # 设置唤醒词（转换为小写用于匹配）
        self.wake_word = "Hi captain"

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
        wake_words = ["uh captain", "hey captain", "hello,captain"]
        lower_text = text.lower()
        match_found = False
        for word in wake_words:
            if word in lower_text:
                match_found = True
                break
            ratio = difflib.SequenceMatcher(None, lower_text, word).ratio()
            if ratio > 0.8:
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
        while rclpy.ok():
            if not self.listening:
                # 暂停识别时，短暂等待
                time.sleep(0.1)
                continue
            try:
                data = self.audio_queue.get(timeout=1)
            except queue.Empty:
                continue

            if self.recognizer_en.AcceptWaveform(data):
                result = json.loads(self.recognizer_en.Result())
                text = result.get("text", "").strip()
                if text:
                    self.process_recognized_text(text)
            else:
                partial = json.loads(self.recognizer_en.PartialResult())
                partial_text = partial.get("partial", "").strip()
                if partial_text and partial_text != self.last_partial:
                    self.last_partial = partial_text
                    self.get_logger().info("部分识别: " + partial_text)
            time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = VoskSTTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()