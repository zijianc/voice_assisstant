#!/usr/bin/env python3
"""
Test script for UWA RAG Knowledge Base
Tests various queries related to UWA campus information
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from my_voice_assistant.uwa_knowledge_base import UWAKnowledgeBase


def test_rag_functionality():
    """Test the RAG system with various UWA-related queries"""
    print("🧪 Testing UWA RAG Knowledge Base")
    print("=" * 60)
    
    try:
        # Initialize knowledge base
        kb = UWAKnowledgeBase()
        print("✅ Knowledge base initialized successfully")
        
        # Show stats
        stats = kb.get_stats()
        print(f"📊 Database contains {stats['total_documents']} documents")
        print(f"📂 Categories: {list(stats['categories'].keys())}")
        print(f"🏢 Buildings: {list(stats['buildings'].keys())}")
        print()
        
        # Test queries that new students typically ask
        test_queries = [
            # Building hours and locations
            "What time does the library open?",
            "Where is Student Central located?",
            "When is the medical centre open?",
            "Where can I buy textbooks?",
            
            # Transportation
            "How do I get to UWA by public transport?",
            "What buses go to UWA?",
            "Where can I park on campus?",
            "Is there a shuttle bus?",
            
            # Food and amenities
            "Where can I eat on campus?",
            "What food options are available?",
            "Where is the cafe?",
            "Is there a food court?",
            
            # Services
            "Where do I go for enrollment?",
            "How do I get help with my studies?",
            "Where is the pharmacy?",
            "What WiFi is available?",
            
            # Recreation
            "Where is the gym?",
            "What time does the sports centre open?",
            "What recreational facilities are available?",
            
            # Events and orientation
            "When is orientation week?",
            "What happens during O-Week?",
        ]
        
        print("🔍 Testing Knowledge Retrieval:")
        print("-" * 60)
        
        for i, query in enumerate(test_queries, 1):
            print(f"\n{i:2d}. Query: '{query}'")
            
            # Search for relevant information
            results = kb.search(query, n_results=2)
            
            if results:
                for j, result in enumerate(results):
                    category = result['metadata']['category']
                    building = result['metadata']['building']
                    content = result['content']
                    distance = result['distance']
                    
                    print(f"    📋 Result {j+1}: [{category}] {content}")
                    if building not in ['Campus General', 'unknown']:
                        print(f"    🏢 Location: {building}")
                    print(f"    📊 Relevance: {(1-distance)*100:.1f}%")
            else:
                print("    ❌ No relevant results found")
        
        # Test category-specific searches
        print(f"\n\n🎯 Testing Category-Specific Searches:")
        print("-" * 60)
        
        categories = ['library', 'transport', 'dining', 'health', 'student_services']
        
        for category in categories:
            print(f"\n📂 Category: {category}")
            results = kb.search("information", n_results=3, category_filter=category)
            
            for result in results:
                print(f"   • {result['content'][:80]}...")
        
        # Test adding new information
        print(f"\n\n➕ Testing Adding Custom Information:")
        print("-" * 60)
        
        # Add some new information
        new_info = [
            {
                "content": "IT Help Desk is located in Building 22, open Monday-Friday 8am-5pm. Provides computer support and software assistance for students.",
                "category": "technology",
                "building": "Building 22"
            },
            {
                "content": "International Student Services in Building 20 helps with visa questions, cultural adjustment, and international student events.",
                "category": "student_services", 
                "building": "Building 20"
            }
        ]
        
        kb.add_documents(new_info)
        print(f"✅ Added {len(new_info)} new documents")
        
        # Test retrieval of new information
        new_query = "Where can I get computer help?"
        print(f"\n🔍 Testing new query: '{new_query}'")
        results = kb.search(new_query, n_results=2)
        
        for result in results:
            print(f"   📋 {result['content']}")
            print(f"   🏢 {result['metadata']['building']}")
        
        # Final stats
        final_stats = kb.get_stats()
        print(f"\n📊 Final Statistics:")
        print(f"   Total documents: {final_stats['total_documents']}")
        print(f"   Categories: {final_stats['categories']}")
        
        print("\n✅ All RAG tests completed successfully!")
        
    except Exception as e:
        print(f"❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    test_rag_functionality()
