#!/bin/bash

# Baseline Performance Test Script
# 用于测试传统STT → LLM → TTS管道的性能

echo "🎯 Baseline Performance Test for Conventional Pipeline"
echo "======================================================"

# 环境设置
cd /workspaces/ros2_ws

# 激活虚拟环境
if [ -d "venv" ]; then
    echo "📦 Activating virtual environment..."
    source venv/bin/activate
fi

# 加载ROS2环境
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "🤖 Loading ROS2 environment..."
    source /opt/ros/humble/setup.bash
fi

if [ -f "install/setup.bash" ]; then
    echo "🔧 Loading workspace..."
    source install/setup.bash
fi

# 加载环境变量
if [ -f ".env" ]; then
    echo "📄 Loading environment variables..."
    set -a
    source .env
    set +a
    echo "✅ Environment variables loaded"
else
    echo "⚠️  .env file not found"
fi

# 检查API密钥
if [ -z "$OPENAI_API_KEY" ]; then
    echo "❌ Error: OPENAI_API_KEY not set"
    echo "Please add OPENAI_API_KEY=your_key_here to .env file"
    exit 1
fi

# 设置Python路径
export PYTHONPATH="/workspaces/ros2_ws/venv/lib/python3.10/site-packages:$PYTHONPATH"

# 解析命令行参数
SAMPLES=30
OUTPUT_FILE="baseline_results_$(date +%Y%m%d_%H%M%S).json"
START_NODES=true

while [[ $# -gt 0 ]]; do
    case $1 in
        --samples)
            SAMPLES="$2"
            shift 2
            ;;
        --output)
            OUTPUT_FILE="$2"
            shift 2
            ;;
        --no-nodes)
            START_NODES=false
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  --samples N     Number of test samples (default: 30)"
            echo "  --output FILE   Output file name (default: baseline_results_TIMESTAMP.json)"
            echo "  --no-nodes     Skip starting nodes (assume already running)"
            echo "  --help         Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo "🔧 Test Configuration:"
echo "   Samples: $SAMPLES"
echo "   Output: $OUTPUT_FILE"
echo "   Start nodes: $START_NODES"
echo ""

# 检查工作空间是否已构建
if [ ! -f "install/setup.bash" ]; then
    echo "❌ Workspace not built. Please run:"
    echo "   colcon build --packages-select my_voice_assistant"
    exit 1
fi

# 函数：启动节点
start_baseline_nodes() {
    echo "🚀 Starting baseline pipeline nodes..."
    
    # 启动STT节点
    echo "Starting STT node..."
    ros2 run my_voice_assistant openai_stt_node &
    STT_PID=$!
    sleep 3
    
    # 启动LLM节点
    echo "Starting LLM node..."
    ros2 run my_voice_assistant llm_node &
    LLM_PID=$!
    sleep 3
    
    # 启动TTS节点
    echo "Starting TTS node..."
    ros2 run my_voice_assistant openai_tts_node &
    TTS_PID=$!
    sleep 3
    
    echo "✅ All baseline nodes started"
    echo "   STT PID: $STT_PID"
    echo "   LLM PID: $LLM_PID" 
    echo "   TTS PID: $TTS_PID"
}

# 函数：停止节点
stop_baseline_nodes() {
    echo "🛑 Stopping baseline nodes..."
    if [ ! -z "$STT_PID" ]; then
        kill $STT_PID 2>/dev/null
    fi
    if [ ! -z "$LLM_PID" ]; then
        kill $LLM_PID 2>/dev/null
    fi
    if [ ! -z "$TTS_PID" ]; then
        kill $TTS_PID 2>/dev/null
    fi
    echo "✅ Nodes stopped"
}

# 信号处理
trap 'echo ""; echo "🛑 Interrupted by user"; stop_baseline_nodes; exit 1' INT TERM

# 启动节点（如果需要）
if [ "$START_NODES" = true ]; then
    start_baseline_nodes
    
    # 等待节点完全启动
    echo "⏳ Waiting for nodes to fully initialize..."
    sleep 5
    
    # 检查节点是否运行
    if ! ros2 node list | grep -q "openai_stt_node\|llm_node\|openai_tts_node"; then
        echo "⚠️  Warning: Some nodes may not be running properly"
        echo "   Please check that all nodes started successfully"
    fi
fi

# 运行测试
echo "🎯 Starting baseline performance test..."
echo "Press Ctrl+C to stop early"
echo ""

if [ "$START_NODES" = true ]; then
    python3 ../../tests/performance/baseline_performance_test.py --samples $SAMPLES --output "$OUTPUT_FILE"
else
    python3 ../../tests/performance/baseline_performance_test.py --samples $SAMPLES --output "$OUTPUT_FILE" --no-nodes
fi

TEST_EXIT_CODE=$?

# 停止节点（如果我们启动了它们）
if [ "$START_NODES" = true ]; then
    stop_baseline_nodes
fi

# 检查测试结果
if [ $TEST_EXIT_CODE -eq 0 ]; then
    echo ""
    echo "✅ Baseline test completed successfully!"
    echo "📊 Results saved to: $OUTPUT_FILE"
    
    # 显示简要结果
    if [ -f "$OUTPUT_FILE" ]; then
        echo ""
        echo "📋 Quick Summary:"
        python3 -c "
import json
try:
    with open('$OUTPUT_FILE') as f:
        data = json.load(f)
        stats = data.get('statistics', {})
        print(f\"  Total samples: {stats.get('total_samples', 0)}\")
        print(f\"  Valid samples: {stats.get('valid_samples', 0)}\")
        
        if 'speech_to_first_response_ms' in stats:
            median = stats['speech_to_first_response_ms'].get('median', 0)
            print(f\"  Speech → First Response: {median:.1f}ms (median)\")
        
        if 'total_response_time_ms' in stats:
            median = stats['total_response_time_ms'].get('median', 0)
            print(f\"  Total Response Time: {median:.1f}ms (median)\")
except Exception as e:
    print(f\"  Could not parse results: {e}\")
"
    fi
    
    echo ""
    echo "📄 Files generated:"
    echo "   JSON: $OUTPUT_FILE"
    echo "   CSV:  ${OUTPUT_FILE%.json}.csv"
    echo "   Report: ${OUTPUT_FILE%.json}_report.txt"
    
else
    echo ""
    echo "❌ Baseline test failed with exit code: $TEST_EXIT_CODE"
fi

echo ""
echo "🏁 Baseline performance test finished"