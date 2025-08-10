#!/bin/bash
# 安装和配置TEN VAD
# Install and configure TEN VAD for voice activity detection

echo "🎤 TEN VAD 安装和配置脚本"
echo "==============================="

# 检查Python环境
echo "🔍 检查Python环境..."
python3 --version
pip3 --version

# 检查系统依赖 (Linux)
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "🔧 检查Linux系统依赖..."
    
    # 检查 libc++1
    dpkg -l | grep -q libc++1
    if [ $? -ne 0 ]; then
        echo "📦 安装 libc++1..."
        sudo apt update
        sudo apt install -y libc++1
        if [ $? -eq 0 ]; then
            echo "✅ libc++1 安装成功"
        else
            echo "❌ libc++1 安装失败"
            exit 1
        fi
    else
        echo "✅ libc++1 已安装"
    fi
    
    # 检查其他必要依赖
    echo "📦 安装其他依赖..."
    sudo apt install -y build-essential cmake
fi

# 安装Python依赖
echo "📦 安装Python依赖..."
pip3 install numpy scipy

# 安装TEN VAD
echo "🎯 安装TEN VAD..."
pip3 install -U --force-reinstall -v git+https://github.com/TEN-framework/ten-vad.git

if [ $? -eq 0 ]; then
    echo "✅ TEN VAD 安装成功"
else
    echo "❌ TEN VAD 安装失败"
    echo "可能的解决方案:"
    echo "1. 检查网络连接"
    echo "2. 确保有足够的磁盘空间"
    echo "3. 手动下载并安装:"
    echo "   git clone https://github.com/TEN-framework/ten-vad.git"
    echo "   cd ten-vad"
    echo "   pip install ."
    exit 1
fi

# 测试安装
echo "🧪 测试TEN VAD安装..."
python3 -c "
try:
    from ten_vad import TenVad
    import numpy as np
    
    # 创建TEN VAD实例
    vad = TenVad(hop_size=256, threshold=0.5)
    print('✅ TEN VAD 导入成功')
    
    # 测试基本功能
    test_audio = np.zeros(256, dtype=np.int16)
    prob, flag = vad.process(test_audio)
    print(f'✅ TEN VAD 功能测试成功: prob={prob:.3f}, flag={flag}')
    
    del vad
    print('✅ TEN VAD 清理成功')
    
except ImportError as e:
    print(f'❌ TEN VAD 导入失败: {e}')
    exit(1)
except Exception as e:
    print(f'❌ TEN VAD 测试失败: {e}')
    exit(1)
"

if [ $? -eq 0 ]; then
    echo ""
    echo "🎉 TEN VAD 安装和配置完成！"
    echo ""
    echo "下一步:"
    echo "1. 编译ROS2包: colcon build --packages-select my_voice_assistant"
    echo "2. 启动TEN VAD STT节点: ./start_ten_vad_stt.sh"
    echo ""
    echo "TEN VAD特性:"
    echo "  ✓ 比传统RMS VAD更精确"
    echo "  ✓ 深度学习驱动的语音活动检测"
    echo "  ✓ 专为对话AI优化，低延迟"
    echo "  ✓ 轻量级，高性能"
else
    echo "❌ TEN VAD 测试失败"
    exit 1
fi
