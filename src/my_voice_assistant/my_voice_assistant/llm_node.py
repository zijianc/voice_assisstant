#!/usr/bin/env python3
import os
import re
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

    def filter_content(self, text: str) -> str:
        """过滤LLM回复中的不需要内容"""
        if not text:
            return text
            
        # 过滤【】括号及其内容（包括数字、符号等）
        # 匹配【任何内容】的模式
        filtered_text = re.sub(r'【[^】]*】', '', text)
        
        # 过滤其他可能的干扰内容
        # 过滤连续的数字和特殊符号组合
        filtered_text = re.sub(r'\b\d{3,}\b', '', filtered_text)  # 移除3位以上连续数字
        filtered_text = re.sub(r'[†‑]+', '', filtered_text)  # 移除特殊符号
        filtered_text = re.sub(r'L\d+-L\d+', '', filtered_text)  # 移除L数字-L数字模式
        
        # 清理多余的空格和标点
        filtered_text = re.sub(r'\s+', ' ', filtered_text)  # 合并多个空格
        filtered_text = re.sub(r'\s*\.\s*\.+', '.', filtered_text)  # 清理多个句号
        filtered_text = filtered_text.strip()
        
        return filtered_text

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
            content_buffer = ""  # 用于累积内容进行过滤

            for chunk in response:
                # **改为对象方式访问**
                if chunk.choices:
                    delta = chunk.choices[0].delta
                    content = delta.content if delta.content else ""

                    if content:
                        final_reply += content  # 累积完整回复
                        content_buffer += content  # 累积到缓冲区
                        
                        # 检查是否包含完整的【】括号内容
                        if '【' in content_buffer and '】' in content_buffer:
                            # 过滤掉【】括号内容后发布
                            filtered_content = self.filter_content(content_buffer)
                            if filtered_content.strip():  # 只有在过滤后还有内容时才发布
                                msg = String()
                                msg.data = filtered_content
                                self.get_logger().debug("发布过滤后片段: {}".format(filtered_content))
                                self.publisher_.publish(msg)
                                self.get_logger().info("流式发布 (已过滤): " + filtered_content)
                            else:
                                self.get_logger().debug("片段被过滤掉: {}".format(content_buffer))
                            content_buffer = ""  # 清空缓冲区
                        else:
                            # 如果不包含【】，检查是否是普通内容可以直接发布
                            if '【' not in content_buffer:
                                # 过滤普通内容
                                filtered_content = self.filter_content(content_buffer)
                                if filtered_content.strip():
                                    msg = String()
                                    msg.data = filtered_content
                                    self.get_logger().debug("发布片段: {}".format(filtered_content))
                                    self.publisher_.publish(msg)
                                    self.get_logger().info("流式发布: " + filtered_content)
                                content_buffer = ""  # 清空缓冲区
            
            # 处理剩余的缓冲区内容
            if content_buffer.strip():
                filtered_content = self.filter_content(content_buffer)
                if filtered_content.strip():
                    msg = String()
                    msg.data = filtered_content
                    self.publisher_.publish(msg)
                    self.get_logger().info("流式发布 (最终): " + filtered_content)

            
            # 最终过滤整个回复并记录
            final_filtered_reply = self.filter_content(final_reply)
            self.get_logger().info("完整回复生成完毕，原始长度: {}, 过滤后长度: {}".format(
                len(final_reply.strip()), len(final_filtered_reply.strip())))
            self.get_logger().info("最终过滤后回复: {}".format(final_filtered_reply))
            return final_filtered_reply.strip()

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