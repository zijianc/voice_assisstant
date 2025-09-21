#!/usr/bin/env python3
"""
Extended UWA Tools for Campus Services
Pr            # 只有在Google搜索完全失败时才使用最小备用数据
            if not all_results:
                       # 只有在网络搜索失            # 只有在网络搜索            # 只有在网络搜索            # 只有在网络搜索失败时才使用备用信息
            if not all_results:
                self.get_logger().warning("网络搜索无结果，使用备用位置数据")
                minimal_fallback = self._get_minimal_location_fallback(query)
                final_result['results'] = minimal_fallback
                final_result['source'] = 'Fallback_Data'
                final_result['note'] = "网络搜索暂时不可用，显示基础信息"
            
            # 添加Google搜索验证方法
            final_result['google_verification_methods'] = [
                f"🔍 Google搜索: 'UWA {query} location Perth address'",
                f"🗺️ Google Maps: 搜索 'UWA {query}' 查看准确位置和路线",
                "📱 Google评论: 查看其他访客的最新经验和提示",
                "📞 Google商家信息: 查看联系方式和营业时间"
            ]
            
            # 添加实时导航建议
            final_result['real_time_tips'] = [
                f"🌐 建议访问 uwa.edu.au 获取最新的 {query} 信息",
                "📱 使用UWA官方应用查看实时设施状态",
                "🗺️ Google Maps 搜索 'UWA' + 您的目标设施",
                "📞 如需确认，请致电UWA总机 (08) 6488 6000"
            ]
            
            final_result['search_type'] = "UWA_Location_Google_Search"
            
            return final_result    if not all_results:
                self.get_logger().warning("网络搜索无结果，使用备用位置数据")
                minimal_fallback = self._get_minimal_location_fallback(query)
                final_result['results'] = minimal_fallback
                final_result['source'] = 'Fallback_Data'
                final_result['note'] = "网络搜索暂时不可用，显示基础信息"
            
            # 添加Google搜索验证方法
            final_result['google_verification_methods'] = [
                f"🔍 Google搜索: 'UWA {query} location Perth address'",
                f"🗺️ Google Maps: 搜索 'UWA {query}' 查看准确位置和路线",
                "📱 Google评论: 查看其他访客的最新经验和提示",
                "📞 Google商家信息: 查看联系方式和营业时间"
            ]
            
            # 添加实时导航建议
            final_result['real_time_tips'] = [
                f"🌐 建议访问 uwa.edu.au 获取最新的 {query} 信息",
                "📱 使用UWA官方应用查看实时设施状态",
                "🗺️ Google Maps 搜索 'UWA' + 您的目标设施",
                "📞 如需确认，请致电UWA总机 (08) 6488 6000"
            ]        if not all_results:
                self.get_logger().warning("网络搜索无结果，使用备用位置数据")
                minimal_fallback = self._get_minimal_location_fallback(query)
                final_result['results'] = minimal_fallback
                final_result['source'] = 'Fallback_Data'
                final_result['note'] = "网络搜索暂时不可用，显示基础信息"
            
            # 添加Google搜索验证方法
            final_result['google_verification_methods'] = [
                f"🔍 Google搜索: 'UWA {query} location Perth address'",
                f"🗺️ Google Maps: 搜索 'UWA {query}' 查看准确位置和路线",
                "📱 Google评论: 查看其他访客的最新经验和提示",
                "📞 Google商家信息: 查看联系方式和营业时间"
            ]
            
            # 添加实时导航建议
            final_result['real_time_tips'] = [
                f"🌐 建议访问 uwa.edu.au 获取最新的 {query} 信息",
                "📱 使用UWA官方应用查看实时设施状态",
                "🗺️ Google Maps 搜索 'UWA' + 您的目标设施",
                "📞 如需确认，请致电UWA总机 (08) 6488 6000"
            ]       if not all_results:
                self.get_logger().warning("网络搜索无结果，使用备用位置数据")
                minimal_fallback = self._get_minimal_location_fallback(query)
                final_result['results'] = minimal_fallback
                final_result['source'] = 'Fallback_Data'
                final_result['note'] = "网络搜索暂时不可用，显示基础信息"
            
            # 添加Google搜索验证方法
            final_result['google_verification_methods'] = [
                f"🔍 Google搜索: 'UWA {query} location Perth address'",
                f"🗺️ Google Maps: 搜索 'UWA {query}' 查看准确位置和路线",
                "📱 Google评论: 查看其他访客的最新经验和提示",
                "📞 Google商家信息: 查看联系方式和营业时间"
            ]
            
            # 添加实时导航建议
            final_result['real_time_tips'] = [
                f"🌐 建议访问 uwa.edu.au 获取最新的 {query} 信息",
                "📱 使用UWA官方应用查看实时设施状态",
                "🗺️ Google Maps 搜索 'UWA' + 您的目标设施",
                "📞 如需确认，请致电UWA总机 (08) 6488 6000"
            ]llback = self._get_minimal_location_fallback(query)
                       # 添加Google搜索验证方            # 添加Google搜            final_result['google_verification_methods'] = [
                f"🔍 Google搜索: 'UWA campus food {dining_type} Perth hours'",
                f"🗺️ Google Maps: 搜索 'UWA food court' 查看评分和营业时间",
                "📱 Google评论: 查看最新学生食堂评价",
                "🍽️ UberEats/Deliveroo: 查看校园周边外卖选择"
            ]
            
            # 只有在无法获取信息时才提供估计
            if not all_results:
                final_result['estimated_options'] = "⚠️ 无法获取实时餐饮信息，请使用上述方法确认"
                final_result['source'] = 'No_Real_Time_Data'
            
            final_result['search_type'] = "Campus_Dining_Search_Enhanced"
            return final_result
            
        except Exception as e:
            return {
                'success': False,
                'error': f"餐饮搜索失败: {str(e)}",
                'search_type': "Campus_Dining_Search_Error"
            }

    def check_parking_availability(self, area: str = "UWA campus") -> Dict:            final_result['google_verification_methods'] = [
                f"🔍 Google搜索: 'UWA campus food {dining_type} Perth hours'",
                f"🗺️ Google Maps: 搜索 'UWA food court' 查看评分和营业时间",
                "📱 Google评论: 查看最新学生食堂评价",
                "🍽️ UberEats/Deliveroo: 查看校园周边外卖选择"
            ]
            
            # 只有在无法获取信息时才提供估计
            if not all_results:
                final_result['estimated_options'] = "⚠️ 无法获取实时餐饮信息，请使用上述方法确认"
                final_result['source'] = 'No_Real_Time_Data'
            
            final_result['search_type'] = "Campus_Dining_Search_Enhanced"
            return final_result
            
        except Exception as e:
            return {
                'success': False,
                'error': f"餐饮搜索失败: {str(e)}",
                'search_type': "Campus_Dining_Search_Error"
            }

    def check_parking_availability(self, area: str = "UWA campus") -> Dict:       final_result['google_verification_methods'] = [
                f"🔍 Google搜索: 'UWA campus food {dining_type} Perth hours'",
                f"🗺️ Google Maps: 搜索 'UWA food court' 查看评分和营业时间",
                "📱 Google评论: 查看最新学生食堂评价",
                "🍽️ UberEats/Deliveroo: 查看校园周边外卖选择"
            ]
            
            ]
            
            # 只有在无法获取信息时才提供估计
            if not all_results:
                final_result['estimated_options'] = "⚠️ 无法获取实时餐饮信息，请使用上述方法确认"
                final_result['source'] = 'No_Real_Time_Data'
            
            final_result['search_type'] = "Campus_Dining_Search_Enhanced"
            return final_result
            
        except Exception as e:
            return {
                'success': False,
                'error': f"餐饮搜索失败: {str(e)}",
                'search_type': "Campus_Dining_Search_Error"
            }

    def check_parking_availability(self, area: str = "UWA campus") -> Dict:

    def check_parking_availability(self, area: str = "UWA campus") -> Dict:nal_result['results'] = minimal_fallback
                final_result['source'] = 'Minimal_Fallback'
                final_result['note'] = "⚠️ 网络搜索暂时不可用，显示基础信息"
            
            # 添加Google搜索建议
            final_result['google_search_tips'] = [
                f"🔍 Google搜索: 'UWA {query} Perth' 获取最新信息",
                "🗺️ Google Maps: 搜索 'UWA' 查看校园地图和实时信息",
                "📍 Google Reviews: 查看其他访客的最新评价和建议",
                "📱 Google助手: 询问 'UWA附近的{query}'"
            ]nsive campus information and services
"""

import os
import requests
import json
from datetime import datetime, timedelta
from typing import Dict, List, Optional
from dotenv import load_dotenv

# Handle relative imports
import sys
import os
if __name__ == '__main__':
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from .web_search_tools import WebSearchTools
except ImportError:
    from web_search_tools import WebSearchTools

load_dotenv()


class ExtendedUWATools(WebSearchTools):
    """Extended UWA campus services and information tools"""
    
    def __init__(self):
        super().__init__()
        self.uwa_base_url = "https://www.uwa.edu.au"
        
    def search_uwa_locations(self, query: str = "building locations") -> Dict:
        """
        搜索UWA校园建筑和设施位置 - 优先使用实时网络搜索
        
        Args:
            query: 位置查询（建筑名、设施类型等）
            
        Returns:
            Dict with location information and directions
        """
        try:
            # 使用Google搜索获取最新最全面的信息
            search_queries = [
                f"UWA {query} location address Perth 2025",
                f"University Western Australia {query} contact phone hours",
                f"UWA campus {query} directions map Google Maps",
                f"UWA {query} opening hours current status"
            ]
            
            all_results = []
            for search_query in search_queries:
                result = self.search_web(search_query, max_results=3)
                if result.get('success') and result.get('results'):
                    all_results.extend(result['results'])
            
            # 如果Google搜索有结果，优先使用
            final_result = {
                'success': True,
                'query': query,
                'source': 'Google_Search_Real_Time',
                'results': all_results,
                'timestamp': datetime.now().isoformat()
            }
            
            # 只有在网络搜索失败时才使用备用信息
            if not all_results:
                self.get_logger().warning("网络搜索无结果，使用备用位置数据")
                minimal_fallback = self._get_minimal_location_fallback(query)
                final_result['results'] = minimal_fallback
                final_result['source'] = 'Fallback_Data'
                final_result['note'] = "网络搜索暂时不可用，显示基础信息"
            
            # 添加Google搜索验证方法
            final_result['google_verification_methods'] = [
                f"🔍 Google搜索: 'UWA {query} location Perth address'",
                f"🗺️ Google Maps: 搜索 'UWA {query}' 查看准确位置和路线",
                "📱 Google评论: 查看其他访客的最新经验和提示",
                "📞 Google商家信息: 查看联系方式和营业时间"
            ]
            
            # 添加实时导航建议
            final_result['real_time_tips'] = [
                f"� 建议访问 uwa.edu.au 获取最新的 {query} 信息",
                "� 使用UWA官方应用查看实时设施状态",
                "🗺️ Google Maps 搜索 'UWA' + 您的目标设施",
                "� 如需确认，请致电UWA总机 (08) 6488 6000"
            ]
            
            final_result['search_type'] = "UWA_Location_Google_Search"
            
            return final_result
            
        except Exception as e:
            return {
                'success': False,
                'error': f"位置搜索失败: {str(e)}",
                'suggestion': f"请尝试Google搜索 'UWA {query} Perth' 获取最新信息",
                'timestamp': datetime.now().isoformat()
            }
    
    def get_uwa_hours(self, facility: str) -> Dict:
        """
        获取UWA设施开放时间 - 优先实时搜索
        
        Args:
            facility: 设施名称（library, dining, sports centre等）
            
        Returns:
            Dict with opening hours and current status
        """
        try:
            now = datetime.now()
            current_time = now.strftime("%I:%M %p")
            current_day = now.strftime("%A")
            current_date = now.strftime("%B %d, %Y")
            
            # 使用Google搜索获取实时开放时间信息
            search_queries = [
                f"UWA {facility} opening hours today {current_day} September 2025",
                f"University Western Australia {facility} hours contact phone",
                f"UWA {facility} current status open closed today Perth",
                f"UWA campus {facility} hours weekend weekday"
            ]
            
            all_results = []
            for search_query in search_queries:
                result = self.search_web(search_query, max_results=2)
                if result.get('success') and result.get('results'):
                    all_results.extend(result['results'])
            
            final_result = {
                'success': True,
                'facility_query': facility,
                'current_time': current_time,
                'current_day': current_day,
                'current_date': current_date,
                'source': 'Google_Search_Real_Time',
                'results': all_results,
                'timestamp': datetime.now().isoformat()
            }
            
            # 添加Google搜索验证建议
            final_result['google_verification_methods'] = [
                f"🔍 Google搜索: 'UWA {facility} opening hours Perth'",
                f"�️ Google Maps: 查看 UWA {facility} 的营业时间",
                "� Google商家信息: 查看电话号码和最新营业状态",
                "� Google助手: 询问 'UWA图书馆现在开放吗?'"
            ]
            
            # 只有在完全无法获取网络信息时才提供估计时间
            if not all_results:
                final_result['estimated_note'] = f"⚠️ 无法获取 {facility} 的实时开放时间，请使用上述方法确认"
                final_result['source'] = 'No_Real_Time_Data'
            
            # 添加季节性提醒
            month = now.month
            if month in [12, 1, 2]:
                final_result['seasonal_reminder'] = "🏖️ 夏季假期期间，开放时间可能有变化"
            elif month in [6, 7]:
                final_result['seasonal_reminder'] = "❄️ 冬季假期期间，请确认开放时间"
            else:
                final_result['seasonal_reminder'] = "📚 正常学期，考试期间可能延长开放时间"
            
            final_result['search_type'] = "UWA_Hours_Search_Enhanced"
            
            return final_result
            
        except Exception as e:
            return {
                'success': False,
                'error': f"开放时间搜索失败: {str(e)}",
                'current_time': current_time,
                'current_day': current_day,
                'suggestion': f"请直接联系UWA或访问官网查询 {facility} 开放时间",
                'timestamp': datetime.now().isoformat()
            }
    
    def search_campus_dining(self, dining_type: str = "all", preferences: str = "") -> Dict:
        """
        搜索校园餐饮选择 - 使用Google搜索获取实时信息
        
        Args:
            dining_type: 餐饮类型（cafe, restaurant, food court等）
            preferences: 饮食偏好（vegetarian, halal, asian等）
            
        Returns:
            Dict with dining options and current availability
        """
        try:
            now = datetime.now()
            current_time = now.strftime("%I:%M %p")
            current_day = now.strftime("%A")
            
            # 构建Google搜索查询而不是UWA站点限制
            search_queries = [
                f"UWA campus dining food options {dining_type} Perth 2025",
                f"University Western Australia cafe restaurant food court",
                f"UWA campus food {preferences} options open today"
            ]
            
            if preferences:
                search_queries.append(f"UWA {preferences} food dining options Perth")
            
            all_results = []
            for search_query in search_queries:
                result = self.search_web(search_query, max_results=3)
                if result.get('success') and result.get('results'):
                    all_results.extend(result['results'])
            
            final_result = {
                'success': True,
                'dining_type': dining_type,
                'preferences': preferences,
                'current_time': current_time,
                'current_day': current_day,
                'source': 'Google_Search_Real_Time',
                'results': all_results,
                'timestamp': datetime.now().isoformat()
            }
            
            # 添加Google搜索验证方法
            final_result['google_verification_methods'] = [
                f"🔍 Google搜索: 'UWA campus food {dining_type} Perth hours'",
                f"�️ Google Maps: 搜索 'UWA food court' 查看评分和营业时间",
                "📱 Google评论: 查看最新学生食堂评价",
                "🍽️ UberEats/Deliveroo: 查看校园周边外卖选择"
            ]
            
            # 只有在无法获取信息时才提供估计
            if not all_results:
                final_result['estimated_options'] = "⚠️ 无法获取实时餐饮信息，请使用上述方法确认"
                final_result['source'] = 'No_Real_Time_Data'
            
            final_result['search_type'] = "Campus_Dining_Search_Enhanced"
            return final_result
            
        except Exception as e:
            return {
                'success': False,
                'error': f"餐饮搜索失败: {str(e)}",
                'search_type': "Campus_Dining_Search_Error"
            }

    def check_parking_availability(self, area: str = "UWA campus") -> Dict:
        """
        检查UWA停车位实时可用性和信息
        
        Args:
            area: 停车区域查询
            
        Returns:
            Dict with current parking information and rates
        """
        try:
            current_time = datetime.now()
            
            # 使用Google搜索获取实时停车信息
            search_queries = [
                f"UWA parking {area} availability rates Perth 2025",
                f"University Western Australia parking permits visitor costs current",
                f"UWA campus parking real time availability today {current_time.strftime('%A')}"
            ]
            
            all_results = []
            for search_query in search_queries:
                result = self.search_web(search_query, max_results=2)
                if result.get('success') and result.get('results'):
                    all_results.extend(result['results'])
            
            final_result = {
                'success': True,
                'area_query': area,
                'source': 'Google_Search_Real_Time',
                'results': all_results,
                'search_time': current_time.isoformat()
            }
            
            # 添加Google搜索验证方法
            final_result['google_verification_methods'] = [
                f"🔍 Google搜索: 'UWA parking {area} rates Perth real time'",
                f"🗺️ Google Maps: 搜索 'UWA parking' 查看实时停车状态",
                "📱 SpotHero/ParkWhiz: 查看预订停车位选项",
                "🚗 Wilson Parking: 查看UWA附近Wilson停车场"
            ]
            
            # 实时停车建议
            current_hour = current_time.hour
            is_weekend = current_time.weekday() >= 5
            
            if is_weekend:
                final_result['current_advice'] = "🅿️ 周末停车限制较少，更多区域可用"
            elif 8 <= current_hour <= 10:
                final_result['current_advice'] = "⚠️ 上班/上课高峰期，建议提早到达或考虑公共交通"
            elif 11 <= current_hour <= 14:
                final_result['current_advice'] = "🚗 中午时段相对繁忙，预留额外寻找停车位的时间"
            elif current_hour >= 17:
                final_result['current_advice'] = "✅ 下班时间，停车位逐渐增多"
            else:
                final_result['current_advice'] = "🌙 非高峰时段，停车相对容易"
            
            # 实时验证方法
            final_result['real_time_options'] = [
                "🌐 访问 uwa.edu.au/campus/parking 查看最新停车信息",
                "� 下载ParkingEye或相关停车应用查看实时可用性",
                "📞 致电UWA安保 (08) 6488 2222 询问停车状态",
                "�️ 使用Google Maps查看UWA附近停车场实时情况"
            ]
            
            # 实用的当前建议
            final_result['immediate_suggestions'] = [
                "🎯 Sports Centre停车场通常容量较大",
                "💳 准备信用卡或手机支付",
                "⏱️ 注意停车时间限制和标志",
                "� 考虑公共交通作为替代方案"
            ]
            
            # 如果网络搜索无结果，提供基本指导
            if not all_results:
                final_result['basic_guidance'] = [
                    "🏢 主要访客停车区域：Sports Centre, Student Central",
                    "💰 预计费用：$4-20/天（具体以现场为准）",
                    "⚠️ 实时费率和可用性请现场查看或致电确认"
                ]
                final_result['source'] = 'Basic_Guidance_Only'
            
            final_result['search_type'] = "UWA_Parking_Search_Enhanced"
            
            return final_result
            
        except Exception as e:
            return {
                'success': False,
                'error': f"停车信息搜索失败: {str(e)}",
                'suggestion': "请访问 uwa.edu.au 或致电 (08) 6488 2222 查询最新停车信息",
                'timestamp': datetime.now().isoformat()
            }
    
    def find_nearby_services(self, location: str, service_type: str = "general") -> Dict:
        """
        查找UWA校园附近的服务设施 - 使用Google搜索获取实时信息
        
        Args:
            location: 校园位置（Student Central, library等）
            service_type: 服务类型（bank, medical, pharmacy, food等）
            
        Returns:
            Dict with nearby services and current information
        """
        try:
            current_time = datetime.now()
            
            # 构建Google搜索查询获取实时服务信息
            search_queries = [
                f"near UWA {location} {service_type} services Perth open now",
                f"University Western Australia campus {service_type} nearby locations hours",
                f"UWA campus {location} {service_type} walking distance 2025"
            ]
            
            if service_type.lower() == 'medical':
                search_queries.extend([
                    "UWA campus medical center GP clinic Perth",
                    "near University Western Australia pharmacy health services"
                ])
            elif service_type.lower() == 'bank':
                search_queries.extend([
                    "UWA campus ATM bank Westpac Commonwealth ANZ Perth",
                    "near University Western Australia banking services"
                ])
            elif service_type.lower() == 'food':
                search_queries.extend([
                    "UWA campus restaurants cafes nearby Crawley Perth",
                    "near University Western Australia food delivery options"
                ])
            
            all_results = []
            for search_query in search_queries:
                result = self.search_web(search_query, max_results=3)
                if result.get('success') and result.get('results'):
                    all_results.extend(result['results'])
            
            final_result = {
                'success': True,
                'location_query': location,
                'service_type': service_type,
                'search_time': current_time.strftime("%I:%M %p %A"),
                'source': 'Google_Search_Real_Time',
                'results': all_results,
                'timestamp': datetime.now().isoformat()
            }
            
            # 添加Google搜索验证方法
            final_result['google_verification_methods'] = [
                f"🔍 Google搜索: 'near UWA {service_type} open now Perth'",
                f"🗺️ Google Maps: 搜索 'UWA {location}' 然后查看附近的 {service_type}",
                "📱 Google业务时间: 查看实时营业状态和评分",
                "🚶‍♂️ Google步行导航: 获取从UWA到服务地点的路线"
            ]
            
            # 基于服务类型提供有用提示
            if service_type.lower() == 'medical':
                final_result['service_tips'] = [
                    "🏥 UWA校园内有医疗中心",
                    "🚑 紧急情况请拨打000",
                    "💊 最近的药店可能在Stirling Highway"
                ]
            elif service_type.lower() == 'bank':
                final_result['service_tips'] = [
                    "💳 校园内多个ATM位置",
                    "🏦 附近银行分支在Claremont或Nedlands",
                    "📱 大多数银行支持手机银行服务"
                ]
            elif service_type.lower() == 'food':
                final_result['service_tips'] = [
                    "🍔 校园内有多个餐饮选择",
                    "🚚 外卖服务覆盖UWA区域",
                    "🛒 Coles和Woolworths在附近购物中心"
                ]
            
            # 如果没有获取到实时信息，提供基本指导
            if not all_results:
                final_result['basic_guidance'] = f"⚠️ 无法获取 {service_type} 的实时信息，请使用上述Google搜索方法确认"
                final_result['source'] = 'No_Real_Time_Data'
            
            final_result['search_type'] = "Nearby_Services_Search_Enhanced"
            return final_result
            
        except Exception as e:
            return {
                'success': False,
                'error': f"附近服务搜索失败: {str(e)}",
                'search_type': "Nearby_Services_Search_Error"
            }
            
            # Comprehensive campus services map
            campus_services = {
                "ATM": [
                    {"location": "Student Central", "bank": "Various banks", "fees": "Check with your bank"},
                    {"location": "Sports Centre", "bank": "Commonwealth", "fees": "Standard fees apply"},
                    {"location": "Reid Library entrance", "bank": "Westpac", "fees": "24/7 access"}
                ],
                "Coffee": [
                    {"name": "Reid Cafe", "location": "Reid Library ground floor", "specialty": "Study-friendly"},
                    {"name": "Hackett Cafe", "location": "Central campus", "specialty": "Gourmet coffee"},
                    {"name": "Guild Coffee", "location": "Guild Village", "specialty": "Quick service"},
                    {"name": "Vending machines", "location": "Most buildings", "specialty": "24/7 availability"}
                ],
                "Printing": [
                    {"location": "Reid Library", "services": ["B&W", "Color", "Binding"], "hours": "Library hours"},
                    {"location": "Student Central", "services": ["Document services", "ID printing"], "hours": "9am-5pm"},
                    {"location": "Computer labs", "services": ["Student printing"], "hours": "Varies by lab"},
                    {"location": "Guild services", "services": ["Thesis printing", "Large format"], "hours": "Business hours"}
                ],
                "Study Spaces": [
                    {"name": "Reid Library", "type": "Library study", "features": ["Quiet zones", "Group rooms", "24/7 during exams"]},
                    {"name": "Winthrop Hall foyer", "type": "Informal study", "features": ["Open space", "Natural light"]},
                    {"name": "Faculty buildings", "type": "Department study", "features": ["Subject-specific", "Varies by building"]},
                    {"name": "Outdoor areas", "type": "Casual study", "features": ["Fresh air", "Relaxed atmosphere"]}
                ],
                "Health Services": [
                    {"name": "UWA Medical Centre", "services": ["GP", "Specialists", "Mental health"], "booking": "Appointment required"},
                    {"name": "Pharmacy", "location": "Medical Centre", "services": ["Prescriptions", "Health products"]},
                    {"name": "Counselling", "location": "Student services", "services": ["Free counselling", "Academic support"]}
                ],
                "Food Services": [
                    {"name": "Convenience store", "location": "Campus shop", "items": ["Snacks", "Drinks", "Essentials"]},
                    {"name": "Vending machines", "location": "Throughout campus", "items": ["24/7 snacks and drinks"]},
                    {"name": "Catering", "services": ["Event catering", "Meeting refreshments"], "booking": "Advance booking"}
                ]
            }
            
            # Filter services by type if specified
            service_lower = service_type.lower()
            relevant_services = {}
            
            for service_category, services in campus_services.items():
                if service_lower in service_category.lower() or service_lower == "general":
                    relevant_services[service_category] = services
            
            if not relevant_services:
                # Try partial matching
                for service_category, services in campus_services.items():
                    if any(word in service_category.lower() for word in service_lower.split()):
                        relevant_services[service_category] = services
            
            result['relevant_services'] = relevant_services if relevant_services else campus_services
            
            # Add location-specific recommendations
            location_lower = location.lower()
            if "library" in location_lower or "reid" in location_lower:
                result['location_specific'] = {
                    "nearby_services": ["ATM (Westpac)", "Reid Cafe", "Printing services", "Study rooms"],
                    "tip": "Reid Library is the campus hub - most services within walking distance"
                }
            elif "sports" in location_lower:
                result['location_specific'] = {
                    "nearby_services": ["ATM (Commonwealth)", "Parking", "Cafe", "Fitness facilities"],
                    "tip": "Sports Centre area good for parking and quick access to central campus"
                }
            elif "guild" in location_lower:
                result['location_specific'] = {
                    "nearby_services": ["Multiple food vendors", "Student services", "Printing", "Social spaces"],
                    "tip": "Guild Village is the social and service hub of campus"
                }
            
            # Add operating hours context
            current_hour = datetime.now().hour
            if current_hour < 8 or current_hour > 18:
                result['after_hours_note'] = "⏰ Many services closed after hours. 24/7 options: Vending machines, some ATMs, emergency services"
            
            result['search_type'] = "UWA_Services_Search"
            result['timestamp'] = datetime.now().isoformat()
            
            return result
            
        except Exception as e:
            return {
                'success': False,
                'error': f"Services search failed: {str(e)}",
                'basic_services': "Main services at Reid Library, Student Central, and Guild Village",
                'timestamp': datetime.now().isoformat()
            }
    
    def _is_currently_open(self, hours_string: str, current_time: datetime) -> bool:
        """Check if facility is currently open based on hours string"""
        if hours_string == "Closed" or "closed" in hours_string.lower():
            return False
        
        try:
            # Parse hours like "9:00 AM - 5:00 PM"
            if " - " in hours_string:
                start_str, end_str = hours_string.split(" - ")
                start_time = datetime.strptime(start_str.strip(), "%I:%M %p").time()
                end_time = datetime.strptime(end_str.strip(), "%I:%M %p").time()
                current_time_only = current_time.time()
                
                # Handle overnight hours (like 8 AM - 12 AM)
                if end_time < start_time:
                    return current_time_only >= start_time or current_time_only <= end_time
                else:
                    return start_time <= current_time_only <= end_time
        except:
            pass
        
        return None  # Unable to determine
    
    def _get_seasonal_hours_info(self, current_date: datetime) -> Dict:
        """Get seasonal adjustments for campus hours"""
        month = current_date.month
        
        if month in [12, 1, 2]:  # Summer break
            return {
                "season": "Summer Break",
                "note": "⚠️ 夏季假期期间开放时间可能变化，请确认最新信息",
                "reminder": "建议查看 uwa.edu.au 获取假期开放时间"
            }
        elif month in [6, 7]:  # Winter break
            return {
                "season": "Winter Break", 
                "note": "⚠️ 冬季假期期间开放时间可能调整",
                "reminder": "建议提前确认设施开放状态"
            }
        else:  # Regular semester
            return {
                "season": "Regular Semester",
                "note": "正常学期期间",
                "reminder": "考试期间部分设施可能延长开放时间"
            }
    
    def _get_minimal_location_fallback(self, query: str) -> List[Dict]:
        """提供最基本的位置信息作为备用"""
        basic_locations = {
            "reid library": "校园中心主图书馆，35 Stirling Hwy",
            "student central": "学生服务中心，Guild Courtyard", 
            "sports centre": "体育中心，35 Stirling Hwy",
            "hackett cafe": "主要餐饮区域，Hackett Dr"
        }
        
        query_lower = query.lower()
        matched = []
        
        for key, info in basic_locations.items():
            if any(word in key for word in query_lower.split()):
                matched.append({
                    "name": key.title(),
                    "basic_info": info,
                    "type": "minimal_fallback",
                    "note": "基础信息，请访问 uwa.edu.au 获取详细信息"
                })
        
        return matched if matched else [{
            "name": "UWA Campus General",
            "basic_info": "35 Stirling Highway, Crawley WA 6009",
            "type": "general_fallback", 
            "note": "请访问 uwa.edu.au 或致电 (08) 6488 6000 获取具体位置信息"
        }]
    
    def get_available_functions(self) -> List[Dict]:
        """Return available function definitions for OpenAI function calling"""
        base_functions = super().get_available_functions()
        
        extended_functions = [
            {
                "name": "search_uwa_locations",
                "description": "搜索UWA校园建筑和设施位置，包括地址和导航信息",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "位置查询（建筑名、设施类型等）",
                            "default": "building locations"
                        }
                    }
                }
            },
            {
                "name": "get_uwa_hours", 
                "description": "获取UWA设施开放时间和当前状态",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "facility": {
                            "type": "string",
                            "description": "设施名称（library, dining, sports centre, student services等）",
                            "default": "library"
                        }
                    },
                    "required": ["facility"]
                }
            },
            {
                "name": "search_campus_dining",
                "description": "搜索校园餐饮选项、菜单和价格信息",
                "parameters": {
                    "type": "object", 
                    "properties": {
                        "preferences": {
                            "type": "string",
                            "description": "饮食偏好（vegetarian, halal, coffee, Asian, budget等）",
                            "default": ""
                        }
                    }
                }
            },
            {
                "name": "check_parking_availability",
                "description": "检查UWA停车位可用性、费率和停车建议",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "area": {
                            "type": "string", 
                            "description": "停车区域查询",
                            "default": "UWA campus"
                        }
                    }
                }
            },
            {
                "name": "find_nearby_services",
                "description": "查找UWA校园附近的服务设施（ATM、打印、咖啡等）",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "location": {
                            "type": "string",
                            "description": "校园位置参考点",
                            "default": "central campus"
                        },
                        "service_type": {
                            "type": "string",
                            "description": "服务类型（ATM, coffee, study, printing, health等）",
                            "default": "general"
                        }
                    },
                    "required": ["location", "service_type"]
                }
            }
        ]
        
        return base_functions + extended_functions


def test_extended_tools():
    """Test the extended UWA tools functionality"""
    print("🏫 Testing Extended UWA Tools")
    print("=" * 60)
    
    tools = ExtendedUWATools()
    
    # Test location search
    print("\n1. Testing UWA location search...")
    result = tools.search_uwa_locations("Reid Library")
    if result.get('success', True):
        print(f"✅ Found {len(result.get('results', []))} location results")
        if result.get('navigation_tips'):
            print(f"📍 Navigation tips available")
    else:
        print(f"❌ Location search failed: {result.get('error')}")
    
    # Test facility hours
    print("\n2. Testing facility hours...")
    result = tools.get_uwa_hours("library")
    if result.get('success', True):
        hours_info = result.get('facility_hours', {})
        if hours_info:
            print(f"⏰ {hours_info.get('facility_name', 'Facility')}: {hours_info.get('today_hours', 'N/A')}")
            print(f"🔓 Currently open: {hours_info.get('is_currently_open', 'Unknown')}")
    else:
        print(f"❌ Hours search failed: {result.get('error')}")
    
    # Test dining search
    print("\n3. Testing campus dining...")
    result = tools.search_campus_dining("coffee")
    if result.get('success', True):
        dining_count = len(result.get('all_dining_locations', {}))
        print(f"🍽️ Found {dining_count} dining locations")
        if result.get('current_tip'):
            print(f"💡 Current tip: {result.get('current_tip')}")
    else:
        print(f"❌ Dining search failed: {result.get('error')}")
    
    # Test parking info
    print("\n4. Testing parking information...")
    result = tools.check_parking_availability()
    if result.get('success', True):
        parking_count = len(result.get('parking_areas', {}))
        print(f"🅿️ Found {parking_count} parking areas")
        if result.get('current_advice'):
            print(f"🚗 Current advice: {result.get('current_advice')}")
    else:
        print(f"❌ Parking search failed: {result.get('error')}")
    
    # Test services search
    print("\n5. Testing campus services...")
    result = tools.find_nearby_services("Reid Library", "ATM")
    if result.get('success', True):
        services_count = len(result.get('relevant_services', {}))
        print(f"🏪 Found {services_count} service categories")
    else:
        print(f"❌ Services search failed: {result.get('error')}")
    
    print("\n✅ Extended UWA tools test completed!")


if __name__ == "__main__":
    test_extended_tools()