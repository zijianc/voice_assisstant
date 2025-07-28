#!/bin/bash
# UWA Knowledge Base Update Guide
# 知识库更新指南

echo "🎓 UWA 知识库更新方法"
echo "=" * 50

echo "方法1: 使用管理脚本 (推荐)"
echo "python3 manage_knowledge_base.py stats      # 查看现状"
echo "python3 manage_knowledge_base.py expand     # 扩展内容"
echo "python3 manage_knowledge_base.py test       # 测试功能"
echo ""

echo "方法2: 重置并重新构建"
echo "python3 manage_knowledge_base.py reset      # 完全重置"
echo "python3 expand_knowledge_base.py            # 重新扩展"
echo ""

echo "方法3: 添加单条信息"
echo "python3 -c \""
echo "from uwa_knowledge_base import UWAKnowledgeBase"
echo "kb = UWAKnowledgeBase()"
echo "kb.add_custom_info("
echo "    content='新的校园信息',"
echo "    category='自定义分类',"
echo "    building='具体建筑'"
echo ")\""
echo ""

echo "方法4: 批量添加 (创建新的数据文件)"
echo "# 1. 创建 new_data.py 文件包含新数据"
echo "# 2. 导入并添加到知识库"
echo ""

echo "📊 建议的更新流程:"
echo "1. python3 manage_knowledge_base.py backup  # 先备份"
echo "2. python3 manage_knowledge_base.py stats   # 查看现状"  
echo "3. 选择更新方法 (expand/reset/custom)"
echo "4. python3 manage_knowledge_base.py test    # 验证结果"
