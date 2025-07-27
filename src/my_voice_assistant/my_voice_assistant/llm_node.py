#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI
from dotenv import load_dotenv

load_dotenv()


class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')
        # 订阅来自 Vosk 节点的文本，假设话题名称为 "speech_text"
        self.subscription = self.create_subscription(
            String,
            'speech_text',
            self.listener_callback,
            10
        )
        # 发布 ChatGPT 返回结果到话题 "llm_response"
        self.publisher_ = self.create_publisher(String, 'llm_response', 10)
        # 从环境变量中获取 OpenAI API 密钥
        self.api_key = os.environ.get("OPENAI_API_KEY")
        if self.api_key is None:
            self.get_logger().error("请设置环境变量 OPENAI_API_KEY！")
        else:
            self.client = OpenAI(api_key=self.api_key)
            self.get_logger().info("LLM 节点已启动，等待 Vosk 文本输入...")

    def listener_callback(self, msg: String):
        input_text = msg.data.strip()
        if not input_text:
            return

        self.get_logger().info("收到文本: " + input_text)

        # 实时调用 ChatGPT API 处理文本（内部已经在流式发布）
        self.call_chatgpt_realtime(input_text)

    def call_chatgpt_realtime(self, prompt: str) -> str:
        try:
            messages = [
                {"role": "system", "content": "you are a voice assistant on UWA shuttle bus for supporting information. your name is Captian."},
                {"role": "user", "content": prompt}
            ]
            response = self.client.chat.completions.create(
                model="ft:gpt-4.1-mini-2025-04-14:personal:new-rev-assistant:BxqKCAoR",
                messages=messages,
                temperature=0.7,
                max_tokens=1024,
                stream=True  # **启用流式输出**
            )

            self.get_logger().info("开始生成完整回复...")
            final_reply = ""

            for chunk in response:
                # **改为对象方式访问**
                if chunk.choices:
                    delta = chunk.choices[0].delta
                    content = delta.content if delta.content else ""

                    if content:
                        final_reply += content  # 累积完整回复

                        # **立即发布流式内容**
                        msg = String()
                        msg.data = content
                        self.get_logger().debug("发布片段: {}".format(content))
                        self.publisher_.publish(msg)

                        # 记录日志
                        self.get_logger().info("流式发布: " + content)

            self.get_logger().info("完整回复生成完毕，长度: {}".format(len(final_reply.strip())))
            return final_reply.strip()

        except Exception as e:
            self.get_logger().error("调用 ChatGPT 实时 API 出错: " + str(e))
            return ""
def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()