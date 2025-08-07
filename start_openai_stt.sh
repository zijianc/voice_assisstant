#!/bin/bash

# 启动 openai_stt_node 的脚本
# 设置必要的环境变量

echo "=== 启动 OpenAI STT 节点 ==="

# 激活虚拟环境
source venv/bin/activate

# 加载 ROS2 环境
source install/setup.bash

# 设置 PYTHONPATH 包含虚拟环境的包
export PYTHONPATH=/workspaces/ros2_ws/venv/lib/python3.10/site-packages:$PYTHONPATH

# 加载 .env 文件中的环境变量
if [ -f .env ]; then
    # 过滤掉注释行和空行
    export $(grep -v '^#' .env | grep -v '^$' | xargs)
    echo "✅ 已加载 .env 文件"
else
    echo "⚠️  .env 文件未找到"
fi

# 检查 API key
if [ -z "$OPENAI_API_KEY" ]; then
    echo "❌ OPENAI_API_KEY 未设置"
    exit 1
else
    echo "✅ OPENAI_API_KEY 已设置"
fi

# 启动节点
echo "🚀 启动 openai_stt_node..."
ros2 run my_voice_assistant openai_stt_node
