#!/bin/bash

echo "=== 简化测试脚本 ==="

# 确保环境设置正确
source venv/bin/activate
source install/setup.bash
export PYTHONPATH=/workspaces/ros2_ws/venv/lib/python3.10/site-packages:$PYTHONPATH

echo "🔍 检查 ROS2 环境..."
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"

echo ""
echo "🔍 检查 ROS2 守护进程..."
ros2 daemon status

echo ""
echo "🔍 尝试列出 topics..."
timeout 5 ros2 topic list || echo "❌ 无法列出 topics"

echo ""
echo "🔍 尝试发布简单消息..."
timeout 5 ros2 topic pub --once /test_topic std_msgs/msg/String "data: 'test message'" || echo "❌ 无法发布消息"

echo ""
echo "✅ 测试完成"
