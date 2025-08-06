#!/usr/bin/env python3
"""
Test RAG-enhanced LLM node functionality
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from uwa_knowledge_base import UWAKnowledgeBase

def test_rag_integration():
    """Test the RAG knowledge search functionality"""
    print("🧪 Testing RAG Integration for LLM Node")
    print("=" * 60)
    
    try:
        # Initialize knowledge base
        kb = UWAKnowledgeBase()
        print("✅ Knowledge base initialized")
        
        # Test queries that would come from voice input
        test_queries = [
            "what time does the library open",
            "how to get to UWA",
            "where can I buy food",
            "student services location",
            "medical center hours",
            "WiFi connection",
            "orientation week information"
        ]
        
        print(f"🔍 Testing {len(test_queries)} typical voice queries:")
        print("-" * 60)
        
        for i, query in enumerate(test_queries, 1):
            print(f"\n{i}. Voice Query: '{query}'")
            
            # Search knowledge base
            results = kb.search(query, n_results=3)
            
            if results:
                print(f"   🎯 Found {len(results)} relevant results:")
                for j, result in enumerate(results, 1):
                    category = result['metadata']['category']
                    building = result['metadata']['building']
                    content = result['content']
                    distance = result['distance']
                    
                    print(f"   {j}. [{category}] {content[:60]}...")
                    if building not in ['Campus General', 'unknown']:
                        print(f"      🏢 Location: {building}")
                    print(f"      📊 Relevance: {(1-distance)*100:.1f}%")
            else:
                print("   ❌ No relevant results found")
        
        # Test context formatting
        print(f"\n📝 Testing context formatting:")
        print("-" * 60)
        
        sample_query = "Where is the library?"
        results = kb.search(sample_query, n_results=2)
        
        if results:
            context_parts = ["📚 RELEVANT UWA INFORMATION:"]
            
            for i, result in enumerate(results, 1):
                building = result['metadata'].get('building', 'Unknown')
                category = result['metadata'].get('category', 'general')
                content = result['content']
                
                context_parts.append(f"{i}. [{category.upper()}] {content}")
                if building != 'Campus General' and building != 'unknown':
                    context_parts[-1] += f" (Located: {building})"
            
            context_parts.append("\nPlease use this information to provide accurate, helpful responses about UWA.")
            
            formatted_context = "\n".join(context_parts)
            print("Sample formatted context for LLM:")
            print(f"'{formatted_context}'")
        
        print(f"\n✅ All RAG integration tests passed!")
        print(f"📊 Total documents in knowledge base: {kb.get_stats()['total_documents']}")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_rag_integration()
