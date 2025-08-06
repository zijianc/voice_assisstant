#!/bin/bash

echo "🚀 TTS 性能测试脚本"
echo "======================================"

# 测试不同配置的性能
echo "准备测试不同的 TTS 配置..."

# 基本环境设置
source venv/bin/activate 2>/dev/null
source install/setup.bash 2>/dev/null
export PYTHONPATH="/workspaces/ros2_ws/venv/lib/python3.10/site-packages:$PYTHONPATH"

# 更好的环境变量加载
if [ -f .env ]; then
    echo "📄 加载 .env 文件..."
    set -a  # 自动导出所有变量
    source .env
    set +a  # 关闭自动导出
    echo "✅ 已加载环境变量"
else
    echo "⚠️  未找到 .env 文件"
fi

# 检查 API 密钥
if [ -z "$OPENAI_API_KEY" ]; then
    echo "❌ 错误: OPENAI_API_KEY 未设置"
    echo "请在 .env 文件中添加: OPENAI_API_KEY=your_api_key_here"
    exit 1
else
    echo "✅ OPENAI_API_KEY 已设置"
fi

# 创建测试文本
TEST_TEXT="Hello Captain, this is a performance test for the TTS system."

echo ""
echo "测试文本: $TEST_TEXT"
echo ""

# 测试函数
test_tts_config() {
    local model=$1
    local voice=$2
    local format=$3
    local speed=$4
    local description=$5
    
    echo "🧪 测试配置: $description"
    echo "   模型: $model, 语音: $voice, 格式: $format, 速度: $speed"
    
    # 设置环境变量
    export TTS_MODEL=$model
    export TTS_VOICE=$voice
    export TTS_FORMAT=$format
    export TTS_SPEED=$speed
    export TTS_SAVE_MODE=true
    
    # 创建临时测试脚本
    cat > temp_tts_test.py << EOF
#!/usr/bin/env python3
import time
import os
import sys
from openai import OpenAI

# 确保API密钥存在
api_key = os.environ.get("OPENAI_API_KEY")
if not api_key:
    print("❌ OPENAI_API_KEY 环境变量未设置")
    sys.exit(1)

client = OpenAI(api_key=api_key)

# 从环境变量获取配置
model = "$model"
voice = "$voice"
input_format = "$format"
speed = float("$speed")
test_text = "$TEST_TEXT"

start_time = time.time()
print(f"开始生成语音...")

try:
    response = client.audio.speech.create(
        model=model,
        voice=voice,
        input=test_text,
        response_format=input_format,
        speed=speed
    )
    
    generation_time = time.time() - start_time
    
    # 保存文件
    filename = f"test_{model.replace('-', '_')}_{voice}_{input_format}_{speed}.{input_format if input_format != 'pcm' else 'pcm'}"
    with open(filename, "wb") as f:
        f.write(response.content)
    
    file_size = os.path.getsize(filename)
    
    print(f"✅ 生成完成!")
    print(f"   生成时间: {generation_time:.2f} 秒")
    print(f"   文件大小: {file_size} 字节")
    print(f"   文件名: {filename}")
    
    # 清理文件
    os.remove(filename)
    
except Exception as e:
    print(f"❌ 生成失败: {e}")

EOF

    # 运行测试，确保环境变量传递
    OPENAI_API_KEY="$OPENAI_API_KEY" python3 temp_tts_test.py
    rm -f temp_tts_test.py
    echo ""
}

# 运行各种配置测试
echo "开始性能测试..."
echo ""

# 测试1: 最新模型 + 最快格式
test_tts_config "gpt-4o-mini-tts" "coral" "wav" "1.1" "最新模型+最快格式 (推荐)"

# 测试2: 传统模型 + MP3
test_tts_config "tts-1" "nova" "mp3" "1.0" "传统模型+MP3格式"

# 测试3: 高质量模型
test_tts_config "tts-1-hd" "nova" "mp3" "1.0" "高质量模型"

# 测试4: PCM格式 (最低延迟)
test_tts_config "gpt-4o-mini-tts" "coral" "pcm" "1.2" "PCM格式+快速语音"

# 测试5: Opus格式 (流媒体优化)
test_tts_config "gpt-4o-mini-tts" "coral" "opus" "1.1" "Opus格式 (流媒体)"

echo "🏁 性能测试完成!"
echo ""
echo "📊 推荐配置总结:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🚀 最佳性能: gpt-4o-mini-tts + coral + wav + 1.1x速度"
echo "⚡ 极速响应: gpt-4o-mini-tts + coral + pcm + 1.2x速度"
echo "📱 流媒体: gpt-4o-mini-tts + coral + opus + 1.1x速度"
echo "🎵 高质量: tts-1-hd + nova + mp3 + 1.0x速度"
echo ""
echo "💡 使用建议:"
echo "• 实时对话: 使用 wav 或 pcm 格式"
echo "• 文件存储: 使用 mp3 格式"
echo "• 网络传输: 使用 opus 格式"
echo "• 语音速度: 1.1-1.2x 可以改善响应感"
