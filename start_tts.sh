#!/bin/bash

# 启动 tts_node 的脚本
# 设置必要的环境变量

echo "=== 启动 TTS 节点 ==="

# 激活虚拟环境
source venv/bin/activate

# 加载 ROS2 环境
source install/setup.bash

# 设置 PYTHONPATH 包含虚拟环境的包
export PYTHONPATH=/workspaces/ros2_ws/venv/lib/python3.10/site-packages:$PYTHONPATH

# 检查必要的包
echo "检查必要的 Python 包..."
python3 -c "import gtts; print('✅ gtts 可用')" 2>/dev/null || echo "❌ gtts 不可用"

# 启动节点
echo "🚀 启动 tts_node..."
ros2 run my_voice_assistant tts_node
