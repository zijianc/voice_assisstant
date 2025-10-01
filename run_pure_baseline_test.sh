#!/bin/bash

# Pure Baseline Performance Test Script
# This script runs a clean baseline test without function calling
# to get accurate baseline (STT→LLM→TTS) performance measurements

echo "🧪 Pure Baseline Performance Test (No Function Calling)"
echo "========================================================"
echo "This script will run a clean baseline test with:"
echo "1. Function calling DISABLED (ENABLE_WEB_SEARCH=0)"
echo "2. Pure STT→LLM→TTS pipeline only"
echo "3. No web search or external tools" 
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
    echo "   2. ./start_llm.sh (with ENABLE_WEB_SEARCH=0)"
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

# Run the baseline test with pure pipeline
echo "🚀 Starting pure baseline performance test..."
echo "   Samples: 30"
echo "   Function calling: DISABLED"
echo "   Output: pure_baseline_results_$(date +%Y%m%d_%H%M%S).json"
echo ""

python3 baseline_performance_test.py \
    --samples 30 \
    --output pure_baseline_results_$(date +%Y%m%d_%H%M%S).json \
    --no-nodes

echo ""
echo "📊 Pure baseline test completed!"
echo "   Check the results file for clean baseline performance data"
echo "   This data represents true STT→LLM→TTS pipeline performance"
echo "   without any function calling overhead"