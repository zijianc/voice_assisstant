#!/usr/bin/env python3
"""
测试脚本：验证Realtime API的准确性
测试时间、天气、工具调用等功能
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from datetime import datetime

class RealtimeAccuracyTest(Node):
    def __init__(self):
        super().__init__('realtime_accuracy_test')
        self.publisher = self.create_publisher(String, 'speech_text', 10)
        self.response_count = 0
        self.test_results = {}
        
        # Subscribe to responses
        self.response_subscription = self.create_subscription(
            String,
            'realtime_response',
            self.response_callback,
            10
        )
        
    def response_callback(self, msg):
        """收集AI的回复用于分析"""
        response = msg.data
        if response.strip():
            print(f"📨 AI Response: {response}")
            
    def send_test_question(self, question, test_name):
        """发送测试问题"""
        print(f"\n🧪 测试 {test_name}: {question}")
        msg = String()
        msg.data = question
        self.publisher.publish(msg)
        
    def run_accuracy_tests(self):
        """运行准确性测试"""
        
        # 获取当前实际信息用于对比
        now = datetime.now()
        current_time = now.strftime("%I:%M %p")
        current_day = now.strftime("%A")
        current_date = now.strftime("%B %d, %Y")
        
        print("="*60)
        print("🎯 Realtime API 准确性测试")
        print("="*60)
        print(f"实际时间: {current_time}")
        print(f"实际日期: {current_day}, {current_date}")
        print("开始测试...")
        
        # 测试问题列表 - 需要验证准确性
        test_questions = [
            ("nUWAy, what time is it now?", "时间查询"),
            ("nUWAy, what day is today?", "日期查询"),
            ("nUWAy, what's the current weather in Perth?", "天气查询"),
            ("nUWAy, what's the temperature right now?", "温度查询"),
            ("nUWAy, where is Reid Library?", "位置查询"),
            ("nUWAy, what time does the library close today?", "开放时间"),
            ("nUWAy, find vegetarian food on campus", "餐饮搜索"),
            ("nUWAy, where can I park at UWA?", "停车信息"),
            ("nUWAy, find ATM near Student Central", "服务查找")
        ]
        
        for question, test_name in test_questions:
            self.send_test_question(question, test_name)
            time.sleep(20)  # 等待AI处理和回复
            
        print("\n✅ 所有测试问题已发送！")
        print("\n📋 验证清单:")
        print("1. ⏰ 时间是否准确匹配当前时间?")
        print("2. 📅 日期是否正确?") 
        print("3. 🌡️ 天气信息是否为Perth的实时数据?")
        print("4. 🔧 是否调用了相应的工具函数?")
        print("5. 📍 位置信息是否详细准确?")
        print("6. 🍽️ 餐饮信息是否包含实时选项?")
        print("7. 🅿️ 停车信息是否为最新状态?")
        print("\n💡 观察日志中的函数调用和搜索结果")

def main():
    rclpy.init()
    
    test_node = RealtimeAccuracyTest()
    
    print("🚀 启动准确性测试...")
    print("确保 Realtime API 节点正在运行!")
    time.sleep(3)
    
    test_node.run_accuracy_tests()
    
    # 保持节点运行以接收回复
    print("\n⏳ 保持运行30秒以收集回复...")
    try:
        rclpy.spin_once(test_node, timeout_sec=30)
    except KeyboardInterrupt:
        pass
    
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()