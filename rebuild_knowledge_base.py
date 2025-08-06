#!/usr/bin/env python3
"""
UWA Knowledge Base Rebuild Script
完全重构知识库的安全脚本
"""

import os
import sys
import shutil
from pathlib import Path

# 添加路径以导入模块
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(os.path.dirname(__file__), 'src', 'my_voice_assistant', 'my_voice_assistant'))

def rebuild_knowledge_base():
    """安全地重构UWA知识库"""
    print("🔄 开始重构UWA知识库")
    print("=" * 50)
    
    try:
        # 方法1: 删除数据库文件夹 (最彻底)
        db_path = "./uwa_knowledge_db"
        if os.path.exists(db_path):
            print(f"🗑️  删除现有数据库文件夹: {db_path}")
            shutil.rmtree(db_path)
            print("✅ 旧数据库已删除")
        
        # 方法2: 或者使用API删除集合
        # from uwa_knowledge_base import UWAKnowledgeBase
        # kb = UWAKnowledgeBase()
        # kb.delete_collection()
        
        print("🔧 重新初始化知识库...")
        
        # 重新导入并初始化
        from uwa_knowledge_base import UWAKnowledgeBase
        kb = UWAKnowledgeBase()
        
        print("📊 新知识库统计:")
        stats = kb.get_stats()
        print(f"  总文档数: {stats['total_documents']}")
        print(f"  分类数: {len(stats['categories'])}")
        print(f"  建筑数: {len(stats['buildings'])}")
        
        # 可选: 运行扩展脚本添加更多内容
        print("\n🚀 是否运行扩展脚本添加更多内容？ (y/n): ", end="")
        choice = input().strip().lower()
        
        if choice == 'y':
            from expand_knowledge_base import expand_knowledge_base
            expand_knowledge_base()
        
        print("\n✅ 知识库重构完成！")
        
        # 测试重构后的知识库
        print("\n🧪 测试重构后的知识库:")
        test_queries = ["图书馆开放时间", "怎么坐车到UWA", "哪里可以吃饭"]
        
        for query in test_queries:
            print(f"\n🔍 测试查询: '{query}'")
            results = kb.search(query, n_results=1)
            if results:
                result = results[0]
                content = result['content'][:60] + "..."
                print(f"  ✅ 找到相关信息: {content}")
            else:
                print("  ❌ 未找到相关信息")
        
    except Exception as e:
        print(f"❌ 重构失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    rebuild_knowledge_base()
