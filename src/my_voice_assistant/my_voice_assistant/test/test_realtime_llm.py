#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestRealtimeLLMNode(Node):
    def __init__(self):
        super().__init__('test_realtime_llm_node')
        # 发布测试文本到 "speech_text"
        self.publisher_ = self.create_publisher(String, "speech_text", 10)
        # 订阅 "llm_response" 获取模型回复
        self.subscription = self.create_subscription(
            String,
            "llm_response",
            self.response_callback,
            10
        )
        self.timer = self.create_timer(2.0, self.publish_test_message)
        self.message_sent = False

    def publish_test_message(self):
        if not self.message_sent:
            msg = String()
            msg.data = "Hello, this is a test input for realtime LLM."
            self.publisher_.publish(msg)
            self.get_logger().info("发布测试消息: " + msg.data)
            self.message_sent = True
            self.timer.cancel()

    def response_callback(self, msg: String):
        self.get_logger().info("收到模型回复: " + msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = TestRealtimeLLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()