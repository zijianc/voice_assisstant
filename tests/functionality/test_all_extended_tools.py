#!/usr/bin/env python3
"""
测试完整的扩展UWA工具功能
"""
import sys
import os

# 添加正确的路径
sys.path.append('/workspaces/ros2_ws/src/my_voice_assistant/my_voice_assistant')

# 设置环境变量来处理相对导入
os.environ['PYTHONPATH'] = '/workspaces/ros2_ws/src/my_voice_assistant/my_voice_assistant'

from extended_uwa_tools import ExtendedUWATools

def test_all_extended_functions():
    """测试所有扩展功能"""
    print("🧪 完整的扩展UWA工具测试")
    print("="*80)
    
    tools = ExtendedUWATools()
    
    # 获取所有可用函数
    functions = tools.get_available_functions()
    print(f"\n📋 可用函数总数: {len(functions)}")
    for func in functions:
        print(f"   - {func['name']}: {func['description']}")
    
    print("\n" + "="*80)
    print("🔍 详细功能测试:")
    
    # 1. 测试位置搜索
    print("\n1️⃣ 校园位置搜索:")
    result = tools.search_uwa_locations("Reid Library")
    print(f"   查询: Reid Library")
    print(f"   结果数: {len(result.get('results', []))}")
    if result.get('navigation_tips'):
        print(f"   导航提示: {len(result['navigation_tips'])} 条")
    
    # 2. 测试设施时间
    print("\n2️⃣ 设施开放时间:")
    result = tools.get_uwa_hours("library")
    hours_info = result.get('facility_hours', {})
    if hours_info:
        print(f"   设施: {hours_info.get('facility_name', 'N/A')}")
        print(f"   今天时间: {hours_info.get('today_hours', 'N/A')}")
        print(f"   当前状态: {'开放' if hours_info.get('is_currently_open') else '关闭'}")
    
    # 3. 测试餐饮搜索
    print("\n3️⃣ 校园餐饮:")
    result = tools.search_campus_dining("coffee")
    dining_locations = result.get('all_dining_locations', {})
    print(f"   餐饮地点: {len(dining_locations)} 个")
    if result.get('current_tip'):
        print(f"   当前建议: {result['current_tip']}")
    
    # 4. 测试停车信息
    print("\n4️⃣ 停车信息:")
    result = tools.check_parking_availability()
    parking_areas = result.get('parking_areas', {})
    print(f"   停车区域: {len(parking_areas)} 个")
    if result.get('current_advice'):
        print(f"   停车建议: {result['current_advice']}")
    
    # 5. 测试服务查找
    print("\n5️⃣ 校园服务:")
    result = tools.find_nearby_services("Reid Library", "ATM")
    services = result.get('relevant_services', {})
    print(f"   服务类型: {list(services.keys())}")
    if result.get('location_specific'):
        print(f"   位置特定服务: {len(result['location_specific']['nearby_services'])} 项")
    
    # 6. 测试天气搜索（继承功能）
    print("\n6️⃣ 天气搜索:")
    result = tools.search_current_weather("Perth UWA")
    print(f"   天气查询成功: {result.get('success', False)}")
    print(f"   结果数: {len(result.get('results', []))}")
    
    # 7. 测试UWA交通（继承功能）
    print("\n7️⃣ UWA交通:")
    result = tools.search_uwa_transport()
    print(f"   交通查询成功: {result.get('success', False)}")
    print(f"   结果数: {len(result.get('results', []))}")
    
    print("\n" + "="*80)
    print("✅ 所有扩展功能测试完成！")
    print(f"🎯 nUWAy 4 助手现在拥有 {len(functions)} 个强大的功能工具")

if __name__ == "__main__":
    test_all_extended_functions()