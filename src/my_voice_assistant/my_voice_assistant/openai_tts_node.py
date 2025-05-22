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

    def play_worker(self):
        while rclpy.ok():
            try:
                temp_file = self.play_queue.get(timeout=1)
                # self.get_logger().info(f"播放队列大小: {self.play_queue.qsize()}")
                # self.get_logger().info(f"准备播放语音片段: {temp_file}")
                try:
                    self.get_logger().info(f"使用 mpg123 播放: {temp_file}")
                    process = subprocess.Popen(["mpg123", temp_file])
                    process.wait()
                    time.sleep(0.1)  # 更紧凑的播放间隔
                    # self.get_logger().info(f"mpg123 播放完成: {temp_file}")
                except Exception as e:
                    self.get_logger().error(f"使用 mpg123 播放失败: {str(e)}")
                os.remove(temp_file)
                self.get_logger().info(f"删除临时文件: {temp_file}")
            except queue.Empty:
                continue
 
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