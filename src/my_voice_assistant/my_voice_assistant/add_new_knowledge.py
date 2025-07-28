#!/usr/bin/env python3
"""
UWA Knowledge Base - 新数据添加模板
用于添加新的UWA校园信息到知识库
"""

from uwa_knowledge_base import UWAKnowledgeBase

def add_new_knowledge():
    """添加新的知识库内容"""
    
    # 新的知识条目 - 按需修改以下内容
    new_knowledge = [
        {
            "content": "这里填写新的校园信息内容",
            "category": "academic",  # 可选: academic, transport, dining, health, etc.
            "building": "具体建筑名称"
        },
        # 可以添加更多条目...
        # {
        #     "content": "另一条信息",
        #     "category": "transport",
        #     "building": "Campus General"
        # },
    ]
    
    try:
        # 连接知识库
        kb = UWAKnowledgeBase()
        print(f"✅ 已连接知识库")
        
        # 显示添加前的统计
        before_stats = kb.get_stats()
        print(f"📊 添加前: {before_stats['total_documents']} 条文档")
        
        # 添加新知识
        kb.add_documents(new_knowledge)
        print(f"📝 已添加 {len(new_knowledge)} 条新信息")
        
        # 显示添加后的统计
        after_stats = kb.get_stats()
        print(f"📊 添加后: {after_stats['total_documents']} 条文档")
        
        # 测试新添加的内容
        print(f"\n🧪 测试新添加的内容:")
        for item in new_knowledge:
            # 提取关键词进行测试
            test_query = item['content'][:20]  # 使用前20个字符作为测试查询
            results = kb.search(test_query, n_results=1)
            
            if results:
                print(f"✅ 测试查询 '{test_query}...' - 找到相关结果")
            else:
                print(f"⚠️  测试查询 '{test_query}...' - 未找到结果")
        
        print(f"\n✅ 知识库更新完成!")
        
    except Exception as e:
        print(f"❌ 添加失败: {e}")
        import traceback
        traceback.print_exc()

# 预定义的常用分类和建筑
CATEGORIES = [
    "academic",       # 学术相关
    "transport",      # 交通出行
    "dining",         # 餐饮服务
    "health",         # 健康医疗
    "student_services", # 学生服务
    "library",        # 图书馆
    "recreation",     # 娱乐休闲
    "accommodation",  # 住宿信息
    "technology",     # 技术支持
    "safety",         # 安全相关
    "events",         # 活动事件
    "shopping",       # 购物服务
    "student_life",   # 学生生活
    "landmarks"       # 地标建筑
]

BUILDINGS = [
    "Reid Library",
    "Building 20",
    "Building 21", 
    "Building 30",
    "Sports Centre",
    "Guild Village",
    "Winthrop Hall",
    "Campus General",
    "Various Locations"
]

def show_template_guide():
    """显示模板使用指南"""
    print("📋 新数据添加模板使用指南")
    print("=" * 50)
    print("1. 编辑 new_knowledge 列表，添加你的新信息")
    print("2. 选择合适的 category 和 building")
    print("3. 运行脚本添加到知识库")
    print()
    print("📂 可用分类:")
    for cat in CATEGORIES:
        print(f"  • {cat}")
    print()
    print("🏢 常用建筑:")
    for building in BUILDINGS:
        print(f"  • {building}")
    print()
    print("💡 使用示例:")
    print('  python3 add_new_knowledge.py')

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == 'guide':
        show_template_guide()
    else:
        add_new_knowledge()
