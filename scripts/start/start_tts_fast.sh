#!/bin/bash

echo "🚀 启动优化的 OpenAI TTS 节点"
echo "========================================="

# 基本环境设置
source venv/bin/activate 2>/dev/null
source install/setup.bash 2>/dev/null
export PYTHONPATH="/workspaces/ros2_ws/venv/lib/python3.10/site-packages:$PYTHONPATH"

# 加载API密钥
if [ -f ".env" ]; then
    set -a
    source .env
    set +a
fi

# 设置高性能TTS配置
export TTS_MODEL=gpt-4o-mini-tts
export TTS_VOICE=coral
export TTS_FORMAT=wav
export TTS_SPEED=1.1

echo "🔧 TTS 配置:"
echo "   模型: $TTS_MODEL (最新最快)"
echo "   语音: $TTS_VOICE (推荐语音)"
echo "   格式: $TTS_FORMAT (最低延迟)"
echo "   速度: ${TTS_SPEED}x (优化响应)"

# 检查API密钥
if [ -z "$OPENAI_API_KEY" ]; then
    echo "❌ 请设置 OPENAI_API_KEY"
    exit 1
fi

# 停止现有进程
pkill -f "openai_tts_node" 2>/dev/null
sleep 1

echo "✅ 启动高性能 TTS 节点..."
exec ros2 run my_voice_assistant openai_tts_node
