#!/bin/bash

# Quick RAG Test Script
# Tests the knowledge base functionality before full system integration

echo "🧪 Quick RAG System Test"
echo "========================"

# Check if we're in the right directory
if [ ! -f "src/my_voice_assistant/setup.py" ]; then
    echo "❌ Error: Please run this script from the ROS2 workspace root directory"
    exit 1
fi

echo "📦 Installing dependencies..."
pip install -q chromadb==0.4.24 sentence-transformers==2.7.0 numpy==1.24.3

echo "🔧 Setting up test environment..."
cd src/my_voice_assistant/my_voice_assistant/test

echo "🚀 Running RAG tests..."
python3 test_rag_system.py

echo ""
echo "✅ RAG test completed!"
echo ""
echo "💡 Next steps:"
echo "1. Run: ./start_llm_rag.sh    # Start RAG-enhanced LLM node"
echo "2. Test with voice input or manual ROS topics"
echo "3. Ask UWA-specific questions to see RAG in action"
