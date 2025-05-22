#!/usr/bin/env python3
import os
import pty
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading

class DeepSeekLLMNode(Node):
    def __init__(self):
        super().__init__('deepseek_llm_node')
        # 订阅来自 Vosk 节点的语音文本，假设话题名称为 "speech_text"
        self.subscription = self.create_subscription(
            String,
            'speech_text',
            self.voice_callback,
            10
        )
        # 发布模型生成的响应到话题 "llm_response"
        self.publisher_ = self.create_publisher(String, 'llm_response', 10)

        self.get_logger().info("启动 DeepSeek-R1:8b 模型（通过 Ollama）...")
        # 使用 pty 创建一个伪终端，并用 subprocess.Popen 启动进程
        self.master_fd, slave_fd = pty.openpty()
        self.process = subprocess.Popen(
            ['ollama', 'run', 'deepseek-r1:8b'],
            stdin=slave_fd,
            stdout=slave_fd,
            stderr=slave_fd,
            universal_newlines=True,  # 文本模式
            bufsize=1  # 行缓冲
        )
        # 关闭 slave_fd 以便子进程独占
        os.close(slave_fd)

        # 开启后台线程读取模型输出
        self.output_thread = threading.Thread(target=self.read_output, daemon=True)
        self.output_thread.start()

    def voice_callback(self, msg: String):
        prompt = msg.data.strip()
        if not prompt:
            return
        self.get_logger().info("收到语音文本: " + prompt)
        try:
            # 写入 prompt 到伪终端
            os.write(self.master_fd, (prompt + "\n").encode())
        except Exception as e:
            self.get_logger().error("发送 prompt 时出错: " + str(e))

    def read_output(self):
        try:
            while True:
                # 从伪终端中读取输出（一次读取1024字节）
                output = os.read(self.master_fd, 1024)
                if not output:
                    break
                text = output.decode(errors='ignore').strip()
                if text:
                    self.get_logger().info("模型输出: " + text)
                    # 逐行发布模型输出
                    msg = String()
                    msg.data = text
                    self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error("读取模型输出时出错: " + str(e))

    def destroy_node(self):
        self.get_logger().info("关闭节点，终止 DeepSeek 模型进程...")
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.get_logger().warning("子进程终止超时，强制杀掉")
                self.process.kill()
        # 关闭伪终端的 master_fd
        try:
            os.close(self.master_fd)
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DeepSeekLLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()