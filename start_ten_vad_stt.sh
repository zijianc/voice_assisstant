#!/bin/bash
# 启动TEN VAD STT节点 - 高性能语音活动检测
# Start TEN VAD STT Node with High-Performance Voice Activity Detection

# 设置环境变量路径
ENV_FILE="$(dirname "$0")/.env"

# 检查配置文件是否存在
if [ ! -f "$ENV_FILE" ]; then
    echo "❌ 配置文件不存在: $ENV_FILE"
    echo "请确保 tts_config.env 文件存在并包含必要的配置。"
    exit 1
fi

# 加载环境变量
echo "📝 加载配置文件: $ENV_FILE"
set -o allexport
source "$ENV_FILE"
set +o allexport

# 验证必要的环境变量
if [ -z "$OPENAI_API_KEY" ]; then
    echo "❌ 错误: OPENAI_API_KEY 未设置"
    echo "请在 tts_config.env 中设置 OPENAI_API_KEY"
    exit 1
fi

# 检查并安装TEN VAD
echo "🔍 检查TEN VAD安装状态..."
python3 -c "from ten_vad import TenVad" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "📦 TEN VAD 未安装，正在安装..."
    pip install -U --force-reinstall -v git+https://github.com/TEN-framework/ten-vad.git
    if [ $? -ne 0 ]; then
        echo "❌ TEN VAD 安装失败"
        echo "请手动运行: pip install git+https://github.com/TEN-framework/ten-vad.git"
        exit 1
    fi
    echo "✅ TEN VAD 安装成功"
else
    echo "✅ TEN VAD 已安装"
fi

# 检查系统依赖 (Linux)
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "🔧 检查系统依赖..."
    dpkg -l | grep -q libc++1
    if [ $? -ne 0 ]; then
        echo "📦 安装 libc++1..."
        sudo apt update && sudo apt install -y libc++1
    fi
fi

# 设置 ROS2 环境
echo "🔧 设置 ROS2 环境..."
source install/setup.bash

# 显示配置信息
echo ""
echo "🎤 启动TEN VAD STT节点"
echo "================================"
echo "VAD类型: TEN VAD (深度学习)"
echo "  - API Key: ${OPENAI_API_KEY:0:10}... (隐藏)"
echo "  - 采样率: 16000 Hz (TEN VAD固定)"
echo "  - 帧大小: 256 samples (16ms)"
echo "  - VAD阈值: ${TEN_VAD_THRESHOLD:-0.5}"
echo "  - 最小语音帧: ${TEN_MIN_VOICE_FRAMES:-5}"
echo "  - 最大静音帧: ${TEN_MAX_SILENCE_FRAMES:-50}"

# 音频设备配置
if [ -n "$STT_INPUT_DEVICE_NAME" ]; then
    echo "  - 输入设备名称: $STT_INPUT_DEVICE_NAME"
fi
if [ -n "$STT_INPUT_DEVICE_INDEX" ]; then
    echo "  - 输入设备索引: $STT_INPUT_DEVICE_INDEX"
fi

# TTS门控配置
echo "  - 允许打断: ${STT_ALLOW_BARGE_IN:-false}"
echo "  - 恢复延迟: ${STT_RESUME_HANGOVER_SEC:-0.8}s"

echo ""
echo "🚀 启动节点..."
echo "💡 提示: 使用 Ctrl+C 停止节点"
echo "📋 TEN VAD特性:"
echo "   ✓ 比WebRTC VAD和Silero VAD更精确"
echo "   ✓ 专为对话AI设计，低延迟"
echo "   ✓ 快速检测语音到非语音转换"
echo "   ✓ 识别短暂静音，减少误触发"
echo ""

# 启动TEN VAD STT节点
ros2 run my_voice_assistant ten_vad_stt_node

echo ""
echo "✅ TEN VAD STT节点已停止"
