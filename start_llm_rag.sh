#!/bin/bash

# Start RAG-Enhanced LLM Node
# Enhanced version with knowledge base retrieval for UWA information

echo "🚀 Starting RAG-Enhanced LLM Node for UWA Voice Assistant"
echo "=========================================================="

# Check if we're in the right directory
if [ ! -f "src/my_voice_assistant/setup.py" ]; then
    echo "❌ Error: Please run this script from the ROS2 workspace root directory"
    echo "   Expected to find src/my_voice_assistant/setup.py"
    exit 1
fi

# Check environment variables
if [ -z "$OPENAI_API_KEY" ]; then
    echo "❌ Error: OPENAI_API_KEY environment variable is not set"
    echo "   Please set your OpenAI API key:"
    echo "   export OPENAI_API_KEY='your-api-key-here'"
    exit 1
fi

echo "✅ Environment check passed"

# Source ROS2 environment
echo "🔧 Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash

# Source workspace
if [ -f "install/setup.bash" ]; then
    echo "🔧 Sourcing workspace..."
    source install/setup.bash
else
    echo "⚠️  Workspace not built yet, building now..."
    colcon build --packages-select my_voice_assistant
    source install/setup.bash
fi

# Install RAG dependencies if needed
echo "📦 Checking RAG dependencies..."
python3 -c "import chromadb, sentence_transformers" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "📦 Installing RAG dependencies..."
    pip install chromadb==0.4.24 sentence-transformers==2.7.0 faiss-cpu==1.8.0
fi

# Test knowledge base
echo "🧪 Testing knowledge base initialization..."
cd src/my_voice_assistant/my_voice_assistant/test
python3 -c "
import sys
import os
sys.path.append('..')
from uwa_knowledge_base import UWAKnowledgeBase
try:
    kb = UWAKnowledgeBase()
    stats = kb.get_stats()
    print(f'✅ Knowledge Base ready: {stats[\"total_documents\"]} documents')
except Exception as e:
    print(f'❌ Knowledge Base error: {e}')
    sys.exit(1)
"

if [ $? -ne 0 ]; then
    echo "❌ Knowledge base test failed"
    exit 1
fi

cd - > /dev/null

# Show configuration
echo ""
echo "🔧 Configuration:"
echo "   Model: ft:gpt-4.1-mini-2025-04-14:personal:my-voice-assistant:BxxCKJUa"
echo "   Features: RAG Knowledge Base + Content Filtering + Streaming"
echo "   Knowledge: UWA Campus Information Database"
echo ""

# Start the node
echo "🚀 Starting RAG-Enhanced LLM Node..."
echo "   📡 Subscribing to: /speech_text"
echo "   📤 Publishing to: /llm_response"
echo "   🔍 RAG Database: UWA Campus Knowledge"
echo ""
echo "💡 Try asking questions like:"
echo "   - 'What time does the library open?'"
echo "   - 'How do I get to UWA by bus?'"
echo "   - 'Where can I eat on campus?'"
echo "   - 'Where is Student Central?'"
echo ""
echo "Press Ctrl+C to stop..."
echo ""

# Run the node
ros2 run my_voice_assistant llm_node
