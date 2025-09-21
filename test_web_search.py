#!/usr/bin/env python3
"""
测试网络搜索功能
"""
import sys
import os
sys.path.append('/workspaces/ros2_ws/src/my_voice_assistant')

from my_voice_assistant.web_search_tools import WebSearchTools

def test_weather_search():
    """测试天气搜索"""
    print("🌤️ 测试天气搜索功能...")
    
    try:
        search_tools = WebSearchTools()
        result = search_tools.search_current_weather("Perth UWA")
        
        print("\n📊 搜索结果:")
        print("="*60)
        print(f"成功: {result.get('success', False)}")
        print(f"查询: {result.get('query', 'N/A')}")
        print(f"来源: {result.get('source', 'N/A')}")
        print(f"时间: {result.get('timestamp', 'N/A')}")
        
        if result.get('error'):
            print(f"❌ 错误: {result['error']}")
        
        if result.get('note'):
            print(f"ℹ️  备注: {result['note']}")
        
        results = result.get('results', [])
        print(f"\n📋 结果数量: {len(results)}")
        
        for i, res in enumerate(results, 1):
            print(f"\n🔍 结果 {i}:")
            print(f"   标题: {res.get('title', 'N/A')}")
            print(f"   类型: {res.get('type', 'N/A')}")
            print(f"   内容: {res.get('content', 'N/A')[:200]}...")
            print(f"   链接: {res.get('url', 'N/A')}")
        
        print("="*60)
        
        # 检查是否有有效内容
        has_content = any(res.get('content') and len(res.get('content', '').strip()) > 10 for res in results)
        
        if has_content:
            print("✅ 网络搜索功能正常工作！")
            return True
        else:
            print("⚠️  网络搜索返回空结果")
            return False
            
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        return False

def test_general_search():
    """测试通用搜索"""
    print("\n🔍 测试通用搜索功能...")
    
    try:
        search_tools = WebSearchTools()
        result = search_tools.search_web("current time Perth Australia", max_results=2)
        
        print("\n📊 通用搜索结果:")
        print("="*60)
        print(f"成功: {result.get('success', False)}")
        print(f"查询: {result.get('query', 'N/A')}")
        
        results = result.get('results', [])
        print(f"结果数量: {len(results)}")
        
        for i, res in enumerate(results, 1):
            print(f"\n结果 {i}: {res.get('title', 'N/A')}")
            print(f"内容: {res.get('content', 'N/A')[:150]}...")
        
        return len(results) > 0
        
    except Exception as e:
        print(f"❌ 通用搜索测试失败: {e}")
        return False

if __name__ == "__main__":
    print("🧪 开始网络搜索功能测试\n")
    
    # 测试天气搜索
    weather_ok = test_weather_search()
    
    # 测试通用搜索
    general_ok = test_general_search()
    
    print("\n" + "="*60)
    if weather_ok and general_ok:
        print("🎉 所有网络搜索功能测试通过！")
    elif weather_ok or general_ok:
        print("⚠️  部分网络搜索功能正常")
    else:
        print("❌ 网络搜索功能需要修复")
    print("="*60)