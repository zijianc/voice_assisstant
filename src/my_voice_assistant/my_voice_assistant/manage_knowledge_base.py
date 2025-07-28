#!/usr/bin/env python3
"""
UWA Knowledge Base Management Script
Provides easy commands for managing the RAG knowledge database
"""

import sys
import os
from uwa_knowledge_base import UWAKnowledgeBase

def show_help():
    """显示帮助信息"""
    print("🎓 UWA 知识库管理工具")
    print("=" * 50)
    print("使用方法: python3 manage_knowledge_base.py [命令]")
    print()
    print("可用命令:")
    print("  stats     - 显示知识库统计信息")
    print("  reset     - 重置知识库(删除并重新初始化)")
    print("  expand    - 扩展知识库(添加新内容)")
    print("  test      - 测试知识库搜索功能")
    print("  backup    - 备份当前知识库")
    print("  help      - 显示此帮助信息")
    print()
    print("示例:")
    print("  python3 manage_knowledge_base.py stats")
    print("  python3 manage_knowledge_base.py reset")

def show_stats():
    """显示知识库统计"""
    try:
        kb = UWAKnowledgeBase()
        stats = kb.get_stats()
        
        print("📊 知识库统计信息")
        print("=" * 50)
        print(f"📁 总文档数: {stats['total_documents']}")
        print(f"📂 分类数量: {len(stats['categories'])}")
        print(f"🏢 建筑数量: {len(stats['buildings'])}")
        print()
        
        print("📋 分类详情:")
        for category, count in stats['categories'].items():
            print(f"  • {category}: {count} 条")
        print()
        
        print("🏢 建筑分布:")
        for building, count in stats['buildings'].items():
            if building != 'unknown' and building != 'Campus General':
                print(f"  • {building}: {count} 条")
                
    except Exception as e:
        print(f"❌ 获取统计信息失败: {e}")

def reset_knowledge_base():
    """重置知识库"""
    print("⚠️  警告: 此操作将删除所有知识库数据!")
    confirm = input("确认重置? (输入 'yes' 确认): ")
    
    if confirm.lower() == 'yes':
        try:
            # 删除现有知识库
            kb = UWAKnowledgeBase()
            kb.delete_collection()
            print("🗑️  已删除现有知识库")
            
            # 重新初始化
            new_kb = UWAKnowledgeBase()
            stats = new_kb.get_stats()
            
            print("✅ 知识库重置完成!")
            print(f"📊 重新初始化了 {stats['total_documents']} 条基础信息")
            
        except Exception as e:
            print(f"❌ 重置失败: {e}")
    else:
        print("🚫 操作已取消")

def expand_knowledge_base():
    """扩展知识库"""
    print("📚 开始扩展知识库...")
    try:
        # 导入扩展脚本的功能
        from expand_knowledge_base import expand_knowledge_base as expand_func
        expand_func()
        
    except Exception as e:
        print(f"❌ 扩展失败: {e}")

def test_knowledge_base():
    """测试知识库搜索"""
    try:
        kb = UWAKnowledgeBase()
        
        test_queries = [
            "图书馆开放时间",
            "怎么到UWA",
            "校园餐厅",
            "医疗服务",
            "学生服务"
        ]
        
        print("🧪 测试知识库搜索功能")
        print("=" * 50)
        
        for query in test_queries:
            print(f"\n🔍 查询: '{query}'")
            results = kb.search(query, n_results=2)
            
            if results:
                for i, result in enumerate(results, 1):
                    category = result['metadata']['category']
                    building = result['metadata']['building']
                    content = result['content'][:60] + "..."
                    relevance = (1 - result['distance']) * 100
                    
                    print(f"  {i}. [{category}] {content}")
                    print(f"     📍 {building} | 📊 相关度: {relevance:.1f}%")
            else:
                print("  ❌ 未找到相关信息")
        
        print("\n✅ 测试完成!")
        
    except Exception as e:
        print(f"❌ 测试失败: {e}")

def backup_knowledge_base():
    """备份知识库"""
    try:
        import shutil
        import datetime
        
        kb = UWAKnowledgeBase()
        source_path = kb.db_path
        
        # 创建备份文件名
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_path = f"./uwa_knowledge_db_backup_{timestamp}"
        
        # 复制数据库目录
        shutil.copytree(source_path, backup_path)
        
        stats = kb.get_stats()
        print(f"💾 知识库已备份到: {backup_path}")
        print(f"📊 备份包含 {stats['total_documents']} 条文档")
        
    except Exception as e:
        print(f"❌ 备份失败: {e}")

def main():
    """主函数"""
    if len(sys.argv) < 2:
        show_help()
        return
    
    command = sys.argv[1].lower()
    
    if command == 'help':
        show_help()
    elif command == 'stats':
        show_stats()
    elif command == 'reset':
        reset_knowledge_base()
    elif command == 'expand':
        expand_knowledge_base()
    elif command == 'test':
        test_knowledge_base()
    elif command == 'backup':
        backup_knowledge_base()
    else:
        print(f"❌ 未知命令: {command}")
        print("使用 'python3 manage_knowledge_base.py help' 查看帮助")

if __name__ == "__main__":
    main()
