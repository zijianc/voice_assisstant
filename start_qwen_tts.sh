#!/bin/bash

# Qwen TTS 中文方言语音合成启动脚本
# 使用阿里云 DashScope API 和 Jada 音色（吴语-女）

# 加载环境变量
if [ -f "/workspaces/ros2_ws/.env" ]; then
    echo "📄 加载环境变量文件..."
    set -a  # 自动导出变量
    source /workspaces/ros2_ws/.env
    set +a
fi

# 检查必要的环境变量
if [ -z "$DASHSCOPE_API_KEY" ]; then
    echo "❌ 错误: 请设置环境变量 DASHSCOPE_API_KEY"
    echo "💡 在 .env 文件中添加: DASHSCOPE_API_KEY=你的API密钥"
    exit 1
fi

echo "🚀 启动 Qwen TTS 中文方言语音合成节点..."
echo "🎭 使用音色: Jada (吴语-女)"
echo "🔧 API Key: ${DASHSCOPE_API_KEY:0:10}..."

# 设置 Qwen TTS 配置
export QWEN_TTS_MODEL=${QWEN_TTS_MODEL:-"qwen-tts-latest"}
export QWEN_TTS_VOICE=${QWEN_TTS_VOICE:-"Dylan"}
export TTS_SAVE_MODE=${TTS_SAVE_MODE:-"true"}
export QWEN_TTS_STREAMING=${QWEN_TTS_STREAMING:-"true"}
export QWEN_TTS_REALTIME=${QWEN_TTS_REALTIME:-"false"}

echo "📝 配置信息:"
echo "   - 模型: $QWEN_TTS_MODEL"
echo "   - 音色: $QWEN_TTS_VOICE"
echo "   - 保存模式: $TTS_SAVE_MODE"
echo "   - 流式输出: $QWEN_TTS_STREAMING"
echo "   - 实时播放: $QWEN_TTS_REALTIME"

# 进入ROS2工作区
cd /workspaces/ros2_ws

# 启动前安装依赖
echo "🔄 检查依赖包..."
if ! python3 -c "import dashscope" 2>/dev/null; then
    echo "📦 安装 dashscope 到系统Python..."
    python3 -m pip install dashscope
fi

if ! python3 -c "import pygame" 2>/dev/null; then
    echo "📦 安装 pygame 到系统Python..."
    python3 -m pip install pygame
fi

# 构建项目（如果需要）
if [ ! -f "install/my_voice_assistant/lib/my_voice_assistant/qwen_tts_node" ]; then
    echo "🔨 构建项目..."
    source /opt/ros/humble/setup.bash
    colcon build --packages-select my_voice_assistant
fi

# 加载ROS2环境
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "🎵 启动 Qwen TTS 节点 (监听 llm_response 话题)..."
echo "💬 提示: 确保 LLM 节点正在运行并发布 llm_response 消息"
echo "🎧 音频文件将保存到 /workspaces/ros2_ws/audio_output/ 目录"
echo ""

# 启动节点
ros2 run my_voice_assistant qwen_tts_node

echo "👋 Qwen TTS 节点已关闭"
