#!/bin/bash

echo "🎤 启动实时 STT 节点 (带VAD)"
echo "=============================================="

# 检查 .env 文件
if [ -f ".env" ]; then
    echo "📄 加载环境变量..."
    set -a  # 自动导出所有变量
    source .env
    set +a  # 关闭自动导出
    echo "✅ 环境变量加载完成"
else
    echo "⚠️  未找到 .env 文件，请确保已设置 OPENAI_API_KEY"
fi

# 检查 OpenAI API 密钥
if [ -z "${OPENAI_API_KEY:-}" ]; then
    echo "❌ 错误: 未设置 OPENAI_API_KEY 环境变量！"
    echo "请在 .env 文件中添加: OPENAI_API_KEY=your_api_key_here"
    exit 1
else
    echo "✅ OPENAI_API_KEY 已设置"
fi

# 激活 Python 虚拟环境（如果存在）
if [ -d "venv" ]; then
    echo "🐍 激活 Python 虚拟环境..."
    source venv/bin/activate
    echo "✅ 虚拟环境已激活"
fi

# 设置 ROS2 环境
echo "🤖 设置 ROS2 环境..."
source /opt/ros/humble/setup.bash 2>/dev/null || echo "⚠️  ROS2 基础环境可能未找到"
source install/setup.bash 2>/dev/null || echo "⚠️  本地构建环境可能未找到"

# 设置 PYTHONPATH 包含虚拟环境
export PYTHONPATH="/workspaces/ros2_ws/venv/lib/python3.10/site-packages:$PYTHONPATH"

# 检查包是否已构建
if [ ! -d "install/my_voice_assistant" ]; then
    echo "❌ 错误: my_voice_assistant 包未找到，请先运行构建命令："
    echo "   colcon build --packages-select my_voice_assistant"
    exit 1
fi

# 检查必要的 Python 包
echo "🔍 检查必要的 Python 包..."
python3 -c "import openai; print('✅ openai 可用')" 2>/dev/null || {
    echo "❌ openai 包不可用，请安装：pip install openai"
    exit 1
}
python3 -c "import pyaudio; print('✅ pyaudio 可用')" 2>/dev/null || {
    echo "❌ pyaudio 包不可用，请安装：pip install pyaudio"
    exit 1
}
python3 -c "import numpy; print('✅ numpy 可用')" 2>/dev/null || {
    echo "❌ numpy 包不可用，请安装：pip install numpy"
    exit 1
}

# 防止重复启动
node_pattern="openai_stt_node_with_vad"
if pgrep -f "$node_pattern" >/dev/null 2>&1; then
    echo "⚠️  检测到 realtime STT 节点已在运行"
    echo "当前进程:"
    ps aux | grep "$node_pattern" | grep -v grep
    echo ""
    read -p "是否要停止现有进程并重启? (y/N): " response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        echo "🛑 停止现有进程..."
        pkill -f "$node_pattern"
        sleep 2
    else
        echo "❌ 取消启动"
        exit 0
    fi
fi

# 显示节点信息
echo ""
echo "🎙️  实时 STT 节点功能:"
echo "   • 实时语音活动检测 (VAD)"
echo "   • 自动语音分段和转录"
echo "   • OpenAI Whisper API 支持"
echo "   • 唤醒词检测: 'hi captain', 'hey captain', 'hello captain'"
echo "   • 发布话题: /speech_text"
echo "   • 订阅话题: /tts_status"
echo ""
echo "🔧 VAD 配置 (可通过参数调整):"
echo "   • 阈值: 0.015 (较敏感)"
echo "   • 静音检测: 2.0秒"
echo "   • 最小语音: 0.2秒"
echo "   • 前缓冲区: 1.5秒"
echo ""
echo "🎚️  参数调整示例:"
echo "   ./start_realtime_stt.sh --vad-threshold 0.012 --buffer-history 2.0"
echo ""
echo "💡 提示:"
echo "   • 在安静环境中说话效果最佳"
echo "   • 唤醒词需要清晰发音"
echo "   • 按 Ctrl+C 停止节点"
echo ""

# 处理命令行参数
VAD_ARGS=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --vad-threshold)
            VAD_ARGS="$VAD_ARGS -p vad_threshold:=$2"
            shift 2
            ;;
        --silence-duration)
            VAD_ARGS="$VAD_ARGS -p silence_duration:=$2"
            shift 2
            ;;
        --min-speech-duration)
            VAD_ARGS="$VAD_ARGS -p min_speech_duration:=$2"
            shift 2
            ;;
        --buffer-history)
            VAD_ARGS="$VAD_ARGS -p buffer_history:=$2"
            shift 2
            ;;
        --debug)
            VAD_ARGS="$VAD_ARGS --log-level debug"
            shift
            ;;
        *)
            echo "未知参数: $1"
            echo "支持的参数:"
            echo "  --vad-threshold <值>      VAD阈值 (默认: 0.015)"
            echo "  --silence-duration <值>   静音检测时间 (默认: 2.0)"
            echo "  --min-speech-duration <值> 最小语音时长 (默认: 0.2)"
            echo "  --buffer-history <值>     前缓冲区时间 (默认: 1.5)"
            echo "  --debug                   启用调试模式"
            exit 1
            ;;
    esac
done

if [ -n "$VAD_ARGS" ]; then
    echo "🎚️  使用自定义参数启动..."
    echo "参数: $VAD_ARGS"
    echo "=============================================="
    exec ros2 run my_voice_assistant openai_stt_node_with_vad --ros-args $VAD_ARGS
else
    echo "🚀 使用默认参数启动 realtime STT 节点..."
    echo "=============================================="
    exec ros2 run my_voice_assistant openai_stt_node_with_vad
fi