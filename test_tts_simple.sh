#!/bin/bash

echo "🧪 简单 TTS 测试"
echo "===================="

# 环境设置
source venv/bin/activate 2>/dev/null
export PYTHONPATH="/workspaces/ros2_ws/venv/lib/python3.10/site-packages:$PYTHONPATH"

# 加载环境变量
if [ -f .env ]; then
    set -a
    source .env
    set +a
    echo "✅ 已加载 .env 文件"
fi

# 检查API密钥
if [ -z "$OPENAI_API_KEY" ]; then
    echo "❌ 请先设置 OPENAI_API_KEY 环境变量"
    echo "在 .env 文件中添加: OPENAI_API_KEY=your_api_key"
    exit 1
fi

echo "🎯 测试文本: 'Hello Captain, testing TTS performance!'"
echo ""

# 创建简单测试
python3 << 'EOF'
import time
import os
from openai import OpenAI

try:
    client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))
    
    print("🚀 开始生成语音 (使用优化配置)...")
    start_time = time.time()
    
    response = client.audio.speech.create(
        model="gpt-4o-mini-tts",
        voice="coral",
        input="Hello Captain, testing TTS performance!",
        response_format="wav",
        speed=1.1
    )
    
    generation_time = time.time() - start_time
    
    # 保存测试文件
    test_file = "tts_test_output.wav"
    with open(test_file, "wb") as f:
        f.write(response.content)
    
    file_size = os.path.getsize(test_file)
    
    print(f"✅ 测试成功!")
    print(f"   生成时间: {generation_time:.2f} 秒")
    print(f"   文件大小: {file_size:,} 字节")
    print(f"   输出文件: {test_file}")
    
    # 尝试播放
    try:
        import pygame
        pygame.mixer.init()
        pygame.mixer.music.load(test_file)
        pygame.mixer.music.play()
        print("🔊 开始播放音频...")
        while pygame.mixer.music.get_busy():
            pygame.time.wait(100)
        pygame.mixer.quit()
        print("✅ 播放完成")
    except ImportError:
        print("💡 pygame 不可用，无法播放音频")
    except Exception as e:
        print(f"⚠️  播放失败: {e}")
    
    # 清理文件
    if os.path.exists(test_file):
        os.remove(test_file)
        print("🧹 清理临时文件")
    
except Exception as e:
    print(f"❌ 测试失败: {e}")
    if "gpt-4o-mini-tts" in str(e):
        print("💡 尝试使用传统模型...")
        # 回退到传统模型
        try:
            response = client.audio.speech.create(
                model="tts-1",
                voice="nova",
                input="Hello Captain, testing with fallback model!",
                response_format="mp3",
                speed=1.0
            )
            print("✅ 传统模型测试成功")
        except Exception as e2:
            print(f"❌ 传统模型也失败: {e2}")

EOF

echo ""
echo "🏁 测试完成!"
