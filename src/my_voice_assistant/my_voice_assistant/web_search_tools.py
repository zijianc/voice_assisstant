#!/usr/bin/env python3
"""
Web Search Tools for LLM Function Calling
Provides real-time information search capabilities
"""

import requests
import json
import os
from datetime import datetime
from typing import Dict, List, Optional
from dotenv import load_dotenv

load_dotenv()


class WebSearchTools:
    def __init__(self):
        """Initialize web search tools"""
        self.session = requests.Session()
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36'
        })
    
    def search_web(self, query: str, max_results: int = 3) -> Dict:
        """
        Search the web for current information
        
        Args:
            query: Search query string
            max_results: Maximum number of results to return
            
        Returns:
            Dict with search results and metadata
        """
        try:
            # Method 1: Try DuckDuckGo Instant Answer API
            search_url = "https://api.duckduckgo.com/"
            params = {
                'q': query,
                'format': 'json',
                'no_html': '1',
                'skip_disambig': '1'
            }
            
            response = self.session.get(search_url, params=params, timeout=10)
            data = response.json()
            
            results = []
            
            # Extract instant answer
            if data.get('AbstractText'):
                results.append({
                    'title': data.get('AbstractSource', 'DuckDuckGo'),
                    'content': data['AbstractText'][:500],
                    'url': data.get('AbstractURL', ''),
                    'type': 'instant_answer'
                })
            
            # Extract related topics
            for topic in data.get('RelatedTopics', [])[:max_results-len(results)]:
                if isinstance(topic, dict) and topic.get('Text'):
                    results.append({
                        'title': topic.get('FirstURL', '').split('/')[-1].replace('_', ' '),
                        'content': topic['Text'][:300],
                        'url': topic.get('FirstURL', ''),
                        'type': 'related_topic'
                    })
            
            # If no results from DuckDuckGo API, try fallback method
            if not results:
                results = self._fallback_search(query, max_results)
            
            return {
                'success': True,
                'query': query,
                'results': results[:max_results],
                'timestamp': datetime.now().isoformat(),
                'source': 'DuckDuckGo' if results else 'Fallback'
            }
            
        except Exception as e:
            # Try fallback search on error
            try:
                results = self._fallback_search(query, max_results)
                return {
                    'success': True,
                    'query': query,
                    'results': results,
                    'timestamp': datetime.now().isoformat(),
                    'source': 'Fallback',
                    'note': f'Primary search failed: {str(e)}'
                }
            except:
                return {
                    'success': False,
                    'error': str(e),
                    'query': query,
                    'timestamp': datetime.now().isoformat()
                }
    
    def _fallback_search(self, query: str, max_results: int = 3) -> List[Dict]:
        """
        Fallback search method using predefined knowledge patterns
        """
        results = []
        query_lower = query.lower()
        
        # UWA-specific information patterns
        if any(word in query_lower for word in ['uwa', 'university of western australia']):
            if any(word in query_lower for word in ['bus', 'transport', 'shuttle']):
                results.append({
                    'title': 'UWA Transport Information',
                    'content': 'UWA is serviced by several bus routes including 380, 950, and 107. The main bus stops are at UWA Hackett Drive, Broadway UWA, and Stirling Hwy UWA. Check Transperth website for current schedules.',
                    'url': 'https://www.transperth.wa.gov.au/',
                    'type': 'cached_info'
                })
            
            if any(word in query_lower for word in ['library', 'opening', 'hours']):
                results.append({
                    'title': 'UWA Library Hours',
                    'content': 'UWA Library typically opens 7:00 AM - 11:00 PM Monday-Friday, 9:00 AM - 6:00 PM weekends. Hours may vary during semester breaks. Check UWA library website for current hours.',
                    'url': 'https://www.library.uwa.edu.au/',
                    'type': 'cached_info'
                })
            
            if any(word in query_lower for word in ['events', 'activities']):
                results.append({
                    'title': 'UWA Events',
                    'content': 'UWA regularly hosts academic conferences, student activities, cultural events, and public lectures. Check the UWA events calendar for current listings.',
                    'url': 'https://www.uwa.edu.au/events',
                    'type': 'cached_info'
                })
        
        # Weather patterns
        if any(word in query_lower for word in ['weather', 'temperature', 'rain']):
            from datetime import datetime
            current_time = datetime.now()
            time_str = current_time.strftime("%I:%M %p on %B %d")
            
            results.append({
                'title': 'Perth Weather - Current Information',
                'content': f'For up-to-the-minute weather at {time_str}, check Bureau of Meteorology (bom.gov.au/wa/forecasts/perth.shtml) or weather apps. Perth has Mediterranean climate - current season conditions vary. For immediate conditions, check outside your window or local weather apps.',
                'url': 'http://www.bom.gov.au/wa/forecasts/perth.shtml',
                'type': 'real_time_info'
            })
        
        # Transport patterns
        if any(word in query_lower for word in ['perth', 'transport', 'train', 'bus']):
            results.append({
                'title': 'Perth Public Transport',
                'content': 'Perth public transport includes buses, trains, and ferries operated by Transperth. Use the Transperth app or website for journey planning and real-time information.',
                'url': 'https://www.transperth.wa.gov.au/',
                'type': 'cached_info'
            })
        
        return results[:max_results]
    
    def search_uwa_transport(self, query: str = "UWA bus transport perth") -> Dict:
        """
        Search for UWA transport information specifically
        
        Args:
            query: Transport-related query
            
        Returns:
            Dict with transport search results
        """
        transport_query = f"{query} site:transperth.wa.gov.au OR site:uwa.edu.au transport"
        return self.search_web(transport_query, max_results=2)
    
    def search_current_weather(self, location: str = "Perth UWA") -> Dict:
        """
        Search for current weather information with enhanced real-time suggestions
        
        Args:
            location: Location for weather query
            
        Returns:
            Dict with weather information and actionable advice
        """
        # Try general web search first
        weather_query = f"current weather {location} today temperature"
        base_result = self.search_web(weather_query, max_results=2)
        
        # Enhance with real-time weather guidance
        enhanced_results = []
        
        # Add current time context
        from datetime import datetime
        current_time = datetime.now()
        time_str = current_time.strftime("%I:%M %p")
        date_str = current_time.strftime("%B %d, %Y")
        
        # Add contextual weather advice
        enhanced_results.append({
            'title': f'Current Weather Check for {location}',
            'content': f'As of {time_str} on {date_str}, for the most accurate current weather in Perth area, I recommend checking: 1) Bureau of Meteorology (bom.gov.au) for official conditions, 2) Weather apps like ABC Weather or WeatherZone, 3) Looking outside for immediate conditions. Perth typically has Mediterranean climate - warm summers, mild winters.',
            'url': 'http://www.bom.gov.au/wa/forecasts/perth.shtml',
            'type': 'real_time_advice',
            'timestamp': current_time.isoformat()
        })
        
        # Add seasonal context
        month = current_time.month
        if month in [12, 1, 2]:  # Summer
            season_advice = "It's summer in Perth - expect warm to hot temperatures (20-35°C), minimal rainfall, and strong UV. Consider sun protection."
        elif month in [3, 4, 5]:  # Autumn
            season_advice = "It's autumn in Perth - temperatures are cooling (15-25°C), occasional rain possible. Pleasant weather for outdoor activities."
        elif month in [6, 7, 8]:  # Winter
            season_advice = "It's winter in Perth - mild temperatures (8-18°C), higher chance of rain. You might need a light jacket."
        else:  # Spring
            season_advice = "It's spring in Perth - warming up (12-23°C), variable weather with some rain. Great time to be outdoors."
        
        enhanced_results.append({
            'title': 'Seasonal Context',
            'content': season_advice,
            'url': '',
            'type': 'seasonal_info'
        })
        
        # Combine base results with enhanced info
        all_results = enhanced_results + base_result.get('results', [])
        
        return {
            'success': True,
            'query': weather_query,
            'results': all_results[:3],  # Limit to 3 most relevant
            'timestamp': current_time.isoformat(),
            'source': 'Enhanced_Weather_Search',
            'location': location
        }
    
    def search_uwa_events(self, query: str = "UWA events today") -> Dict:
        """
        Search for current UWA events and activities
        
        Args:
            query: Events-related query
            
        Returns:
            Dict with events information
        """
        events_query = f"{query} site:uwa.edu.au events activities"
        return self.search_web(events_query, max_results=3)
    
    def get_available_functions(self) -> List[Dict]:
        """
        Get list of available functions for OpenAI Function Calling
        
        Returns:
            List of function definitions for OpenAI API
        """
        return [
            {
                "name": "search_web",
                "description": "Search the web for current, real-time information about any topic. USE THIS for most UWA-related questions to get the most up-to-date information.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "The search query to find current information. For UWA questions, include 'UWA' or 'University of Western Australia' in the query."
                        },
                        "max_results": {
                            "type": "integer",
                            "description": "Maximum number of search results (default: 3)",
                            "default": 3
                        }
                    },
                    "required": ["query"]
                }
            },
            {
                "name": "search_uwa_transport",
                "description": "Search for current UWA transport information, bus schedules, route updates, and delays. Use this for any transport-related UWA questions.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "Transport-related query (default: general UWA transport)",
                            "default": "UWA bus transport perth"
                        }
                    },
                    "required": []
                }
            },
            {
                "name": "search_current_weather",
                "description": "Get current weather information for UWA campus or Perth area. Use this for any weather-related questions.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "location": {
                            "type": "string",
                            "description": "Location for weather query (default: Perth UWA)",
                            "default": "Perth UWA"
                        }
                    },
                    "required": []
                }
            },
            {
                "name": "search_uwa_events",
                "description": "Search for current UWA events, activities, campus happenings, and facility information. Use this for questions about UWA activities, events, or general campus information.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "Events or campus information query (default: today's UWA events)",
                            "default": "UWA events today"
                        }
                    },
                    "required": []
                }
            }
        ]


def test_web_search():
    """Test the web search functionality"""
    print("🌐 Testing Web Search Tools")
    print("=" * 50)
    
    tools = WebSearchTools()
    
    # Test general search
    print("\n1. Testing general web search...")
    result = tools.search_web("UWA library opening hours 2025")
    if result['success']:
        print(f"✅ Found {len(result['results'])} results")
        for i, res in enumerate(result['results'], 1):
            print(f"  {i}. {res['title']}: {res['content'][:100]}...")
    else:
        print(f"❌ Search failed: {result['error']}")
    
    # Test UWA transport search
    print("\n2. Testing UWA transport search...")
    result = tools.search_uwa_transport()
    if result['success']:
        print(f"✅ Found {len(result['results'])} transport results")
        for i, res in enumerate(result['results'], 1):
            print(f"  {i}. {res['title']}: {res['content'][:100]}...")
    else:
        print(f"❌ Transport search failed: {result['error']}")
    
    # Test weather search
    print("\n3. Testing weather search...")
    result = tools.search_current_weather()
    if result['success']:
        print(f"✅ Found {len(result['results'])} weather results")
        for i, res in enumerate(result['results'], 1):
            print(f"  {i}. {res['title']}: {res['content'][:100]}...")
    else:
        print(f"❌ Weather search failed: {result['error']}")
    
    print("\n✅ Web search tools test completed!")


if __name__ == "__main__":
    test_web_search()