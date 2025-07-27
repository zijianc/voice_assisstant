
#!/bin/bash
#set -euo pipefail

# 启动 openai_tts_node 的脚本
# 设置必要的环境变量

echo "=== 启动 OpenAI TTS 节点 ==="

# 激活虚拟环境
source venv/bin/activate

# 加载 ROS2 环境
source install/setup.bash

# 设置 PYTHONPATH 包含虚拟环境的包
export PYTHONPATH=/workspaces/ros2_ws/venv/lib/python3.10/site-packages:$PYTHONPATH

# 加载 .env 文件中的环境变量
if [ -f .env ]; then
    set -a
    source .env
    set +a
    echo "✅ 已加载 .env 文件"
else
    echo "⚠️  .env 文件未找到"
fi

# 加载TTS性能配置
if [ -f tts_config.env ]; then
    set -a
    source tts_config.env
    set +a
    echo "🚀 已加载 TTS 性能配置"
    echo "   模型: ${TTS_MODEL:-gpt-4o-mini-tts}"
    echo "   语音: ${TTS_VOICE:-coral}"
    echo "   格式: ${TTS_FORMAT:-wav}"
    echo "   速度: ${TTS_SPEED:-1.1}"
else
    echo "💡 未找到 TTS 配置文件，使用默认优化设置"
    export TTS_MODEL=gpt-4o-mini-tts
    export TTS_VOICE=coral
    export TTS_FORMAT=wav
    export TTS_SPEED=1.1
fi

# 检查 API key
if [ -z "$OPENAI_API_KEY" ]; then
    echo "❌ OPENAI_API_KEY 未设置"
    exit 1
else
    echo "✅ OPENAI_API_KEY 已设置"
fi

# 检查必要的包
echo "检查必要的 Python 包..."
python3 -c "import openai; print('✅ openai 可用')" 2>/dev/null || echo "❌ openai 不可用"
python3 -c "import pygame; print('✅ pygame 可用')" 2>/dev/null || echo "❌ pygame 不可用"

# 启动节点
echo "🚀 启动 openai_tts_node..."

# 检查是否启用文件保存模式
if [ "$1" = "--save-files" ]; then
    export TTS_SAVE_MODE=true
    echo "💾 启用文件保存模式 - 音频文件将保存到 /workspaces/ros2_ws/audio_output/"
    echo "💡 您可以在宿主机上播放这些文件来听取音频"
fi

# 防止重复启动：如果已有 openai_tts_node 在跑，则直接退出
if pgrep -f 'lib/my_voice_assistant/openai_tts_node' >/dev/null 2>&1; then
    echo "⚠️  已检测到 openai_tts_node 进程在运行，避免重复启动，脚本退出。"
    exit 0
fi

exec ros2 run my_voice_assistant openai_tts_node
