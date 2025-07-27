#!/bin/bash

# 测试脚本，验证 openai_stt_node 是否正确安装和配置

echo "=== OpenAI STT Node 测试脚本 ==="

# 检查环境变量
if [ -z "$OPENAI_API_KEY" ]; then
    echo "⚠️  警告: OPENAI_API_KEY 环境变量未设置"
    echo "请运行: export OPENAI_API_KEY='your-api-key'"
else
    echo "✅ OPENAI_API_KEY 已设置"
fi

# 检查节点是否已安装
echo ""
echo "=== 检查已安装的节点 ==="
source install/setup.bash
ros2 pkg executables my_voice_assistant | grep openai_stt_node

if [ $? -eq 0 ]; then
    echo "✅ openai_stt_node 已正确安装"
else
    echo "❌ openai_stt_node 未找到"
    exit 1
fi

# 检查依赖包
echo ""
echo "=== 检查 Python 依赖 ==="
python3 -c "import openai; print('✅ openai 包已安装')" 2>/dev/null || echo "❌ openai 包未安装"
python3 -c "import tenacity; print('✅ tenacity 包已安装')" 2>/dev/null || echo "❌ tenacity 包未安装"
python3 -c "import pyaudio; print('✅ pyaudio 包已安装')" 2>/dev/null || echo "❌ pyaudio 包未安装"

echo ""
echo "=== 测试完成 ==="
echo "如需运行节点，请确保设置 OPENAI_API_KEY 后执行："
echo "ros2 run my_voice_assistant openai_stt_node"
