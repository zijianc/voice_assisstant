#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class TtsTestNode(Node):
    def __init__(self):
        super().__init__('tts_test_node')
        self.publisher = self.create_publisher(String, 'llm_response', 10)
        
    def send_test_message(self, text):
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        self.get_logger().info(f"已发送测试消息: {text}")

def main():
    rclpy.init()
    node = TtsTestNode()
    
    # 等待连接建立
    time.sleep(2)
    
    # 发送一个简单的测试消息
    node.send_test_message("Hello, this is a test message.")
    
    # 等待处理完成
    time.sleep(5)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
