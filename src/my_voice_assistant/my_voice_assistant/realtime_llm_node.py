#!/usr/bin/env python3
import os
import json
import threading
import websocket  # pip install websocket-client
import queue
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from dotenv import load_dotenv
import base64
import pyaudio


load_dotenv()


class RealtimeLLMNode(Node):
    def __init__(self):
        super().__init__('realtime_llm_node')

        # ROS2订阅和发布
        self.subscription = self.create_subscription(
            String,
            "speech_text",
            self.speech_callback,
            10
        )
        self.publisher_ = self.create_publisher(String, "llm_response", 10)

        # API密钥和WebSocket URL
        self.api_key = os.environ.get("OPENAI_API_KEY")
        if not self.api_key:
            self.get_logger().error("请设置环境变量 OPENAI_API_KEY")
            raise ValueError("未设置 OPENAI_API_KEY")

        self.ws_url = os.environ.get(
            "REALTIME_WS_URL",
            "wss://api.openai.com/v1/realtime?model=gpt-4o-realtime-preview-2024-12-17"
        )
        self.ws = None
        self.event_queue = queue.Queue()
        self.connect_ws()
        # 初始化pyaudio，用于播放模型返回的音频
        self.audio_instance = pyaudio.PyAudio()
        self.audio_stream = self.audio_instance.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=20000,
            output=True
        )

    def connect_ws(self):
        headers = [
            f"Authorization: Bearer {self.api_key}",
            "OpenAI-Beta: realtime=v1"
        ]

        def on_open(ws):
            self.get_logger().info("WebSocket连接已建立")
            threading.Thread(target=self.event_sender, args=(ws,), daemon=True).start()

        def on_message(ws, message):
           # self.get_logger().info(f"收到原始WebSocket消息: {message}")
            try:
                data = json.loads(message)
                self.handle_ws_event(data)
            except Exception as e:
                self.get_logger().error(f"解析消息失败: {str(e)}")

        def on_error(ws, error):
            self.get_logger().error("WebSocket错误: " + str(error))

        def on_close(ws, close_status_code, close_msg):
            self.get_logger().info(f"WebSocket连接关闭: code={close_status_code}, msg={close_msg}")
            # 简单的重连逻辑
            time.sleep(5)
            self.get_logger().info("尝试重新连接WebSocket...")
            self.connect_ws()

        self.ws = websocket.WebSocketApp(
            self.ws_url,
            header=headers,
            on_open=on_open,
            on_message=on_message,
            on_error=on_error,
            on_close=on_close
        )

        self.ws_thread = threading.Thread(target=self.ws.run_forever, daemon=True)
        self.ws_thread.start()

    def event_sender(self, ws):
        while True:
            try:
                event = self.event_queue.get()
                event_str = json.dumps(event, ensure_ascii=False)
                #self.get_logger().info(f"[DEBUG] 正在从队列发送事件: {event_str}")
                ws.send(event_str)
            except Exception as e:
                self.get_logger().error(f"发送事件出错: {str(e)}")

    def handle_ws_event(self, event):
        event_type = event.get("type")

        # if event_type == "response.text.delta":
        #     delta = event.get("delta", "")
        #     if delta:
        #         self.get_logger().info(f"流式更新: {delta}")

        # elif event_type == "response.text.done":
        #     final_text = event.get("text", "")
        #     if final_text:
        #         self.get_logger().info(f"最终回复: {final_text}")
        #         msg = String()
        #         msg.data = final_text
        #         self.publisher_.publish(msg)
        if event_type == "response.text.delta":
            delta = event.get("delta", "")
            if delta:
                self.get_logger().info(f"文本流式更新: {delta}")

        elif event_type == "response.text.done":
            final_text = event.get("text", "")
            if final_text:
                self.get_logger().info(f"最终文本回复: {final_text}")
                msg = String()
                msg.data = final_text
                self.publisher_.publish(msg)

        elif event_type == "response.audio.delta":
            audio_delta = event.get("delta", "")
            if audio_delta:
                self.get_logger().info(f"音频流式更新: {audio_delta}")
                # 可在此处解码并实时播放音频片段
                self.play_audio_chunk(audio_delta)

        elif event_type == "response.audio.done":
            full_audio = event.get("audio", "")
            if full_audio:
                self.get_logger().info("收到完整音频回复")
                # 可在此处解码 full_audio 并调用音频播放接口
                self.play_full_audio(full_audio)


        elif event_type == "conversation.item.created":
            role = event["item"].get("role", "")
            content = event["item"].get("content", [])
            if content:
                text = content[0].get("text", "")
            else:
                text = "<空内容>"
           # self.get_logger().info(f"{role.capitalize()}消息: {text}")

        elif event_type == "session.created":
            self.get_logger().info("会话已创建成功。")

        else:
            self.get_logger().info("收到其他事件: " + json.dumps(event, ensure_ascii=False))

    def play_audio_chunk(self, audio_chunk_b64):
        try:
            audio_data = base64.b64decode(audio_chunk_b64)
            self.audio_stream.write(audio_data)
        except Exception as e:
            self.get_logger().error("播放音频片段时出错: " + str(e))

    def play_full_audio(self, full_audio_b64):
        try:
            audio_data = base64.b64decode(full_audio_b64)
            self.audio_stream.write(audio_data)
        except Exception as e:
            self.get_logger().error("播放完整音频时出错: " + str(e))        

    def speech_callback(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f"[DEBUG] 收到ROS2消息长度: {len(text)}, 内容: '{text}'")

        # conv_event = {
        #     "type": "response.create",
        #     "response": {
        #         "modalities": ["text"],
        #         "instructions": text
        #     }
        # }
        conv_event = {
            "type": "response.create",
            "response": {
                "modalities": ["audio", "text"],
                "instructions": text
            }
        }
        event_json = json.dumps(conv_event, ensure_ascii=False)
        #self.get_logger().info(f"[DEBUG] 发送的JSON长度: {len(event_json)}, JSON内容: {event_json}")

        self.send_ws_event(conv_event)

    def send_ws_event(self, event):
        try:
            self.event_queue.put(event)
            #self.get_logger().info(f"[DEBUG] 事件已放入队列: {event}")
        except Exception as e:
            self.get_logger().error(f"放入事件队列出错: {str(e)}")

    def destroy_node(self):
        self.get_logger().info("关闭节点，断开WebSocket连接...")
        if self.ws:
            self.ws.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealtimeLLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被用户中断。")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()