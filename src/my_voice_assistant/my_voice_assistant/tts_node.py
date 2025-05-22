#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
import os
import tempfile
import queue
import threading
import subprocess
import time

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        # 订阅 llm 节点发布的 "llm_response" 话题
        self.subscription = self.create_subscription(
            String,
            'llm_response',
            self.listener_callback,
            10
        )
        self.get_logger().info("TTS 节点已启动，等待 llm_response 消息...")
        
        # 初始化文本缓冲区和定时器
        self.buffer = ""
        self.flush_timer = None
        # 设置等待时长（秒），在这段时间内没有接收到新的文本时刷新缓冲区
        self.flush_delay = 1.0
        self.play_queue = queue.Queue()
        self.play_thread = threading.Thread(target=self.play_worker, daemon=True)
        self.play_thread.start()

    def listener_callback(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info("收到文本: " + text)
        
        # 累积文本到缓冲区
        self.buffer += text + " "
        
        # 始终使用计时器控制刷新，不再依赖句号等触发
        if self.flush_timer is not None:
            self.flush_timer.cancel()
        self.flush_timer = self.create_timer(self.flush_delay, self.flush_buffer)

    def flush_buffer(self):
        # 当缓冲区不为空时，进行TTS转换
        if self.buffer.strip():
            buffered_text = self.buffer.strip()
            self.get_logger().info("缓冲区文本: " + buffered_text)
            self.speak(buffered_text)
            self.buffer = ""
        # 如果定时器存在，则取消并清空
        if self.flush_timer is not None:
            self.flush_timer.cancel()
            self.flush_timer = None

    def speak(self, text: str):
        try:
            # 使用 gTTS 将文本转换为语音（这里选择中文，可以根据需要调整 lang 参数）
            tts = gTTS(text, lang='en-us', slow=False)
            # 创建一个临时文件保存 MP3 文件
            temp_file = tempfile.mktemp(suffix=".mp3")
            tts.save(temp_file)
            self.get_logger().info(f"已生成 MP3 文件: {temp_file}")
            self.play_queue.put(temp_file)
        except Exception as e:
            self.get_logger().error("TTS 错误: " + str(e))

    def play_worker(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f"播放队列大小: {self.play_queue.qsize()}")
                temp_file = self.play_queue.get(timeout=1)
                self.get_logger().info(f"准备播放语音片段: {temp_file}")

                if not os.path.exists(temp_file):
                    self.get_logger().error(f"MP3 文件未找到: {temp_file}")
                    continue

                try:
                    self.get_logger().info(f"使用 mpg123 播放: {temp_file}")
                    process = subprocess.Popen(["mpg123", temp_file])
                    process.wait()  # **确保进程完全执行**
                    time.sleep(1.0)  # **适当增加延迟，避免切换过快**
                    self.get_logger().info(f"mpg123 播放完成: {temp_file}")
                except Exception as e:
                    self.get_logger().error(f"使用 mpg123 播放失败: {str(e)}")

                os.remove(temp_file)
                self.get_logger().info(f"删除临时文件: {temp_file}")
            except queue.Empty:
                continue

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()