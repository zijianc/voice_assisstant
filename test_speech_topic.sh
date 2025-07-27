#!/bin/bash

echo "=== OpenAI STT 节点测试 ==="

# 确保环境设置正确
source venv/bin/activate
source install/setup.bash
export PYTHONPATH=/workspaces/ros2_ws/venv/lib/python3.10/site-packages:$PYTHONPATH

# 测试发布消息到 speech_text topic
echo "测试发布语音识别消息..."

# 模拟唤醒词测试
echo "🔍 测试唤醒词: 'Hi Captain'"
ros2 topic pub --once /speech_text std_msgs/msg/String "data: 'Hi Captain, how are you today?'"

sleep 2

echo "🔍 测试唤醒词: 'Hello Captain'"  
ros2 topic pub --once /speech_text std_msgs/msg/String "data: 'Hello Captain, what is the weather like?'"

sleep 2

echo "🔍 测试非唤醒词: 'Hello World'"
ros2 topic pub --once /speech_text std_msgs/msg/String "data: 'Hello World, this should not trigger'"

echo "✅ 测试完成"
echo "检查节点日志以查看唤醒词检测结果"
