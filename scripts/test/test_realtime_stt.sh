#!/bin/bash

echo "🧪 测试实时 STT 节点"
echo "=================================="

# 基本环境设置
source venv/bin/activate 2>/dev/null
source install/setup.bash 2>/dev/null
export PYTHONPATH="/workspaces/ros2_ws/venv/lib/python3.10/site-packages:$PYTHONPATH"

echo "📋 当前运行的节点:"
ros2 node list 2>/dev/null | grep -E "(stt|speech)" || echo "未找到相关STT节点"

echo ""
echo "📡 检查相关话题:"
ros2 topic list 2>/dev/null | grep -E "(speech_text|tts_status)" || echo "未找到相关话题"

echo ""
echo "🎯 测试不同配置的 STT 节点:"
echo ""

# 测试选项
PS3="请选择测试配置: "
options=(
    "默认配置 (阈值: 0.015)"
    "高敏感度 (阈值: 0.010)"
    "低敏感度 (阈值: 0.025)"
    "快速响应 (静音: 1.0秒)"
    "大缓冲区 (前缓冲: 2.5秒)"
    "调试模式 (详细日志)"
    "监听转录结果"
    "退出"
)

select opt in "${options[@]}"; do
    case $opt in
        "默认配置 (阈值: 0.015)")
            echo "🚀 启动默认配置..."
            ./start_realtime_stt.sh &
            STT_PID=$!
            break
            ;;
        "高敏感度 (阈值: 0.010)")
            echo "🚀 启动高敏感度配置..."
            ./start_realtime_stt.sh --vad-threshold 0.010 &
            STT_PID=$!
            break
            ;;
        "低敏感度 (阈值: 0.025)")
            echo "🚀 启动低敏感度配置..."
            ./start_realtime_stt.sh --vad-threshold 0.025 &
            STT_PID=$!
            break
            ;;
        "快速响应 (静音: 1.0秒)")
            echo "🚀 启动快速响应配置..."
            ./start_realtime_stt.sh --silence-duration 1.0 &
            STT_PID=$!
            break
            ;;
        "大缓冲区 (前缓冲: 2.5秒)")
            echo "🚀 启动大缓冲区配置..."
            ./start_realtime_stt.sh --buffer-history 2.5 &
            STT_PID=$!
            break
            ;;
        "调试模式 (详细日志)")
            echo "🚀 启动调试模式..."
            ./start_realtime_stt.sh --debug &
            STT_PID=$!
            break
            ;;
        "监听转录结果")
            echo "👂 监听转录结果 (按 Ctrl+C 停止)..."
            echo "请说话测试，特别是以唤醒词开头的句子："
            echo "- Hi Captain, what's the weather?"
            echo "- Hello Captain, can you help me?"
            echo "- Hey Captain"
            ros2 topic echo /speech_text
            exit 0
            ;;
        "退出")
            echo "👋 退出测试"
            exit 0
            ;;
        *)
            echo "无效选择，请重试"
            ;;
    esac
done

# 等待节点启动
echo "⏳ 等待节点启动..."
sleep 3

# 检查节点是否成功启动
if ps -p $STT_PID > /dev/null 2>&1; then
    echo "✅ STT 节点已启动 (PID: $STT_PID)"
    echo ""
    echo "🎤 测试说明:"
    echo "1. 清晰地说出唤醒词 + 问题"
    echo "2. 示例: 'Hi Captain, what time is it?'"
    echo "3. 观察日志中的 VAD 检测和转录结果"
    echo "4. 按 Ctrl+C 停止测试"
    echo ""
    
    # 在另一个终端中监听结果
    echo "📡 监听转录结果:"
    timeout 30s ros2 topic echo /speech_text &
    ECHO_PID=$!
    
    # 等待用户测试
    echo "⏰ 测试时长: 30秒"
    wait $ECHO_PID
    
    echo ""
    echo "🛑 停止 STT 节点..."
    kill $STT_PID 2>/dev/null
    
    echo "✅ 测试完成"
else
    echo "❌ STT 节点启动失败"
    exit 1
fi
