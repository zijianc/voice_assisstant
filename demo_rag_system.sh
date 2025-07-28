#!/bin/bash

# UWA RAG语音助手完整演示脚本
# 展示RAG增强的语音助手功能

echo "🎓 UWA RAG 智能语音助手演示"
echo "=============================================="

# 检查环境
echo "🔧 检查系统环境..."

if [ -z "$OPENAI_API_KEY" ]; then
    echo "❌ 未设置 OPENAI_API_KEY"
    echo "请先设置API密钥: export OPENAI_API_KEY='your-key'"
    exit 1
fi

# 检查依赖
echo "📦 检查RAG依赖..."
python3 -c "import chromadb, sentence_transformers; print('✅ RAG依赖正常')" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "⚠️  正在安装RAG依赖..."
    pip install -q chromadb sentence-transformers
fi

# 进入工作目录
cd /workspaces/ros2_ws/src/my_voice_assistant/my_voice_assistant

echo ""
echo "🧪 RAG系统演示"
echo "=============================================="

# 1. 测试知识库
echo "1️⃣ 测试UWA知识库..."
python3 -c "
from uwa_knowledge_base import UWAKnowledgeBase
kb = UWAKnowledgeBase()
stats = kb.get_stats()
print(f'✅ 知识库就绪: {stats[\"total_documents\"]}条信息')
print(f'📂 覆盖类别: {len(stats[\"categories\"])}个')
" 2>/dev/null

# 2. 演示知识检索
echo ""
echo "2️⃣ 演示智能知识检索..."
python3 -c "
from uwa_knowledge_base import UWAKnowledgeBase

kb = UWAKnowledgeBase()
queries = [
    ('图书馆开放时间', '🏢'),
    ('公交路线', '🚌'), 
    ('校园餐厅', '🍽️'),
    ('学生服务', '🎓')
]

for query, emoji in queries:
    print(f'{emoji} 查询: \"{query}\"')
    results = kb.search(query, n_results=1)
    if results:
        content = results[0]['content'][:60] + '...'
        category = results[0]['metadata']['category']
        print(f'   💡 [{category}] {content}')
    print()
" 2>/dev/null

# 3. 展示RAG上下文增强
echo "3️⃣ 展示RAG上下文增强..."
python3 -c "
from uwa_knowledge_base import UWAKnowledgeBase

def format_rag_context(search_results):
    if not search_results:
        return ''
    
    context_parts = ['📚 RELEVANT UWA INFORMATION:']
    for i, result in enumerate(search_results, 1):
        building = result['metadata'].get('building', 'Unknown')
        category = result['metadata'].get('category', 'general')
        content = result['content']
        
        context_parts.append(f'{i}. [{category.upper()}] {content}')
        if building not in ['Campus General', 'unknown']:
            context_parts[-1] += f' (Located: {building})'
    
    context_parts.append('\\nPlease use this information to provide accurate responses.')
    return '\\n'.join(context_parts)

kb = UWAKnowledgeBase()
query = '医疗服务在哪里'
results = kb.search(query, n_results=2)

print(f'🔍 用户查询: \"{query}\"')
print('\\n📝 RAG增强的上下文:')
print('-' * 50)
context = format_rag_context(results)
print(context[:200] + '...')
print()
print('✅ 此上下文将传递给LLM，确保回答准确性')
" 2>/dev/null

# 4. 系统架构说明
echo ""
echo "4️⃣ 系统架构总览"
echo "=============================================="
echo "📱 语音输入 → 🎯 STT节点 → 🔍 RAG知识检索 → 🧠 LLM增强 → 🔊 TTS输出"
echo ""
echo "🎯 核心优势:"
echo "  • 基于44+条UWA官方信息"
echo "  • 准确的时间、地点、联系方式"
echo "  • 避免AI幻觉，提供可靠答案"
echo "  • 支持中英文混合查询"
echo "  • 可持续扩展知识库"

# 5. 使用方法
echo ""
echo "5️⃣ 启动说明"
echo "=============================================="
echo "🚀 启动RAG增强的语音助手:"
echo "   ./start_llm_rag.sh"
echo ""
echo "🎤 支持的查询示例:"
echo "   • '图书馆几点开门？'"
echo "   • '怎么坐公交车到UWA？'"
echo "   • '校园内哪里可以吃饭？'"
echo "   • '学生服务中心在哪里？'"
echo "   • '医疗中心开放时间？'"
echo ""
echo "📚 查看完整指南:"
echo "   cat UWA_RAG_USAGE_GUIDE.md"

# 6. 性能测试选项
echo ""
echo "6️⃣ 可选测试命令"
echo "=============================================="
echo "🧪 完整RAG测试:"
echo "   python3 test_rag_integration.py"
echo ""
echo "📊 知识库扩展:"
echo "   python3 expand_knowledge_base.py"
echo ""
echo "🔧 系统诊断:"
echo "   python3 simple_rag_test.py"

echo ""
echo "🎉 RAG演示完成！您的UWA智能语音助手已准备就绪。"
echo "=============================================="
