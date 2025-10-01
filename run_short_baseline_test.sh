#!/bin/bash

# Short Baseline Performance Test Script
# Runs a smaller sample size to avoid ROS2 context issues

echo "🧪 Short Baseline Performance Test (5 Samples)"
echo "=============================================="
echo "This script runs a short baseline test with:"
echo "1. Function calling DISABLED (ENABLE_WEB_SEARCH=0)"
echo "2. Only 5 samples to avoid context issues"
echo "3. Pure STT→LLM→TTS pipeline measurements"
echo ""

# Check if nodes are running
echo "🔍 Checking if baseline nodes are running..."

# Check for required processes
STT_RUNNING=$(pgrep -f "openai_stt_node" | wc -l)
LLM_RUNNING=$(pgrep -f "llm_node" | wc -l)  
TTS_RUNNING=$(pgrep -f "openai_tts_node" | wc -l)

echo "   STT nodes: $STT_RUNNING"
echo "   LLM nodes: $LLM_RUNNING"  
echo "   TTS nodes: $TTS_RUNNING"

if [ $STT_RUNNING -eq 0 ] || [ $LLM_RUNNING -eq 0 ] || [ $TTS_RUNNING -eq 0 ]; then
    echo "❌ Not all required nodes are running!"
    echo "   Please start the baseline nodes first:"
    echo "   1. ./start_openai_stt.sh"
    echo "   2. ./start_baseline_llm.sh"
    echo "   3. ./start_openai_tts.sh"
    exit 1
fi

echo "✅ All baseline nodes are running"
echo ""

# Set environment to disable function calling
export ENABLE_WEB_SEARCH=0

# Load environment variables from .env file (handles multi-line values)
if [ -f .env ]; then
    export $(grep -v '^#' .env | xargs -d '\n')
fi

# Activate virtual environment
source venv/bin/activate

# Run the baseline test with fewer samples
echo "🚀 Starting short baseline performance test..."
echo "   Samples: 5 (reduced to avoid context issues)"
echo "   Function calling: DISABLED"
echo "   Output: short_baseline_results_$(date +%Y%m%d_%H%M%S).json"
echo ""

python3 baseline_performance_test.py \
    --samples 5 \
    --output short_baseline_results_$(date +%Y%m%d_%H%M%S).json \
    --no-nodes

echo ""
echo "📊 Short baseline test completed!"
echo "   Check the results file for baseline performance data"
echo "   This provides sufficient data for academic comparison"