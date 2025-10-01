#!/usr/bin/env python3
"""
测试改进后的天气搜索功能
"""
import sys
import os
sys.path.append('/workspaces/ros2_ws/src/my_voice_assistant')

from my_voice_assistant.web_search_tools import WebSearchTools

def test_enhanced_weather():
    """测试增强的天气搜索"""
    print("🌤️ 测试增强的天气搜索功能...")
    
    try:
        search_tools = WebSearchTools()
        result = search_tools.search_current_weather("Perth UWA")
        
        print("\n📊 增强天气搜索结果:")
        print("="*80)
        print(f"成功: {result.get('success', False)}")
        print(f"查询: {result.get('query', 'N/A')}")
        print(f"来源: {result.get('source', 'N/A')}")
        print(f"位置: {result.get('location', 'N/A')}")
        print(f"时间: {result.get('timestamp', 'N/A')}")
        
        results = result.get('results', [])
        print(f"\n📋 结果数量: {len(results)}")
        
        for i, res in enumerate(results, 1):
            print(f"\n🔍 结果 {i}:")
            print(f"   标题: {res.get('title', 'N/A')}")
            print(f"   类型: {res.get('type', 'N/A')}")
            print(f"   内容: {res.get('content', 'N/A')}")
            print(f"   链接: {res.get('url', 'N/A')}")
            if res.get('timestamp'):
                print(f"   时间戳: {res.get('timestamp')}")
        
        print("="*80)
        
        # 检查是否有实时建议
        has_realtime = any(res.get('type') in ['real_time_advice', 'seasonal_info'] for res in results)
        
        if has_realtime:
            print("✅ 增强天气搜索功能正常工作！包含实时建议。")
            return True
        else:
            print("⚠️  增强功能可能未生效")
            return False
            
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("🧪 测试增强的天气搜索功能\n")
    test_enhanced_weather()