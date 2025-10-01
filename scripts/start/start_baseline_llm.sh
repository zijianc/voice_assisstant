#!/bin/bash

# 启动纯净Baseline LLM节点脚本（禁用Function Calling）
# 此脚本用于启动不带功能调用的纯净LLM节点，用于baseline测试

echo "=== 启动纯净Baseline LLM节点 ==="
echo "⚠️  功能调用已禁用 (ENABLE_WEB_SEARCH=0)"
echo "🎯 仅用于STT→LLM→TTS基线测试"

# 进入ROS2工作空间
cd /workspaces/ros2_ws

# 加载环境变量（如果存在.env文件）
if [ -f ".env" ]; then
    echo "✅ 加载环境变量..."
    set -a  # 自动导出所有变量
    source .env
    set +a  # 关闭自动导出
    echo "✅ 环境变量加载完成"
else
    echo "❌ 警告: 未找到.env文件，请确保已设置OPENAI_API_KEY环境变量"
fi

# 强制禁用Web搜索和功能调用
export ENABLE_WEB_SEARCH=0
echo "🚫 已禁用Web搜索和功能调用"

# 检查OpenAI API密钥是否设置
if [ -z "$OPENAI_API_KEY" ]; then
    echo "❌ 错误: 未设置OPENAI_API_KEY环境变量！"
    echo "请在.env文件中添加: OPENAI_API_KEY=your_api_key_here"
    exit 1
fi

# 激活Python虚拟环境（如果存在）
if [ -d "venv" ]; then
    echo "🐍 激活Python虚拟环境..."
    source venv/bin/activate
fi

# 设置ROS2环境
echo "🤖 设置ROS2环境..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# 检查包是否已构建
if [ ! -d "install/my_voice_assistant" ]; then
    echo "❌ 错误: my_voice_assistant包未找到，请先运行构建命令："
    echo "colcon build --packages-select my_voice_assistant"
    exit 1
fi

echo "🚀 启动纯净Baseline LLM节点..."
echo "📊 节点配置："
echo "   - 订阅话题: /speech_text (来自STT节点的文本)"
echo "   - 发布话题: /llm_response (流式LLM响应文本)"
echo "   - 使用模型: gpt-4o-mini"
echo "   - 功能调用: 禁用"
echo "   - Web搜索: 禁用"
echo ""
echo "💡 此配置专用于baseline性能测试"
echo "按Ctrl+C停止节点"
echo "=========================================="

# 启动LLM节点
ros2 run my_voice_assistant llm_node