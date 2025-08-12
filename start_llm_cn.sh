#!/bin/bash

# 启动中文苏州话版本的LLM节点
# Start Chinese Suzhou dialect version of LLM node

echo "🚀 启动中文苏州话LLM节点..."
echo "🚀 Starting Chinese Suzhou dialect LLM node..."

# 加载环境变量
if [ -f "tts_config.env" ]; then
    echo "📋 加载TTS配置..."
    source tts_config.env
fi

# 设置ROS2环境
echo "🔧 设置ROS2环境..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动中文LLM节点
echo "🎯 启动llm_node_cn..."
ros2 run my_voice_assistant llm_node_cn
