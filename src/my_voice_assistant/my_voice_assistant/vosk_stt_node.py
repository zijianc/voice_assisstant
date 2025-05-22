#!/usr/bin/env python3
import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import vosk


class VoskSTTNode(Node):
    def __init__(self):
        super().__init__('vosk_stt_node')
        # 创建发布者，发布识别结果到话题 'speech_text'
        self.publisher_ = self.create_publisher(String, 'speech_text', 10)

        # 获取当前文件所在目录，设置英语模型路径（相对于该文件所在位置）
        this_dir = os.path.dirname(os.path.realpath(__file__))
        default_model_path = os.path.join(this_dir, "..", "models", "vosk-model-en-us-0.22")
        # 优先使用环境变量指定的路径
        model_path_en = os.environ.get("VOSK_MODEL_EN_PATH",
                                       "/workspaces/ros2_ws/src/my_voice_assistant/models/vosk-model-en-us-0.22")
        self.get_logger().info("加载英语模型：{}".format(model_path_en))
        self.model_en = vosk.Model(model_path_en)

        self.get_logger().info("加载英语模型：{}".format(model_path_en))

        # 加载英语模型
        self.model_en = vosk.Model(model_path_en)
        self.recognizer_en = vosk.KaldiRecognizer(self.model_en, 44100)

        # 初始化音频输入
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=44100,
            input=True,
            frames_per_buffer=4000
        )
        self.stream.start_stream()
        self.get_logger().info("开始监听麦克风...")

        # 设置唤醒词（转换为小写用于匹配）
        self.wake_word = "good day mate"

        # 创建定时器，每0.1秒读取音频数据
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            data = self.stream.read(4000, exception_on_overflow=False)
        except Exception as e:
            self.get_logger().error("读取音频流错误: {}".format(e))
            return

        if self.recognizer_en.AcceptWaveform(data):
            result = json.loads(self.recognizer_en.Result())
            text = result.get("text", "").strip()
            if text:
                # 检查识别结果是否包含唤醒词（不区分大小写）
                if self.wake_word in text.lower():
                    msg = String()
                    msg.data = text
                    self.publisher_.publish(msg)
                    self.get_logger().info("识别结果 (唤醒词激活): " + text)
                else:
                    self.get_logger().info("未检测到唤醒词: " + text)


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