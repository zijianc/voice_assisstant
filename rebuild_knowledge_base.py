#!/usr/bin/env python3
"""
重建知识库脚本
解决 ChromaDB schema 问题
"""

import os
import shutil
import sys
sys.path.append('./src/my_voice_assistant/my_voice_assistant')

def rebuild_knowledge_base():
    """重建知识库"""
    print("� 重建UWA知识库...")
    
    # 删除现有数据库
    db_path = "./uwa_knowledge_db"
    if os.path.exists(db_path):
        print(f"🗑️ 删除现有数据库: {db_path}")
        shutil.rmtree(db_path)
    
    try:
        # 重新创建知识库
        from uwa_knowledge_base import UWAKnowledgeBase
        
        print("� 初始化新的知识库...")
        kb = UWAKnowledgeBase()
        
        # 获取统计信息
        stats = kb.get_stats()
        print(f"✅ 知识库重建成功!")
        print(f"📊 文档总数: {stats['total_documents']}")
        print(f"📂 分类: {list(stats['categories'].keys())}")
        
        # 测试搜索功能
        print("\n🔍 测试搜索功能:")
        test_query = "图书馆开放时间"
        results = kb.search(test_query, n_results=2)
        for i, result in enumerate(results, 1):
            category = result['metadata']['category']
            content = result['content'][:50] + "..."
            print(f"  {i}. [{category}] {content}")
        
        print("\n✅ 知识库重建完成!")
        
    except Exception as e:
        print(f"❌ 重建失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    rebuild_knowledge_base()
