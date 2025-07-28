#!/usr/bin/env python3
"""
Simple RAG test script
"""

import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from uwa_knowledge_base import UWAKnowledgeBase

def main():
    print("🧪 Testing UWA RAG Knowledge Base")
    print("=" * 50)
    
    try:
        # Initialize knowledge base
        kb = UWAKnowledgeBase()
        print("✅ Knowledge base initialized successfully")
        
        # Show stats
        stats = kb.get_stats()
        print(f"📊 Database contains {stats['total_documents']} documents")
        print(f"📂 Categories: {list(stats['categories'].keys())}")
        print()
        
        # Test queries
        queries = [
            "What time does the library open?",
            "How do I get to UWA by bus?", 
            "Where can I eat on campus?",
            "Where is Student Central?"
        ]
        
        for i, query in enumerate(queries, 1):
            print(f"{i}. Query: '{query}'")
            results = kb.search(query, n_results=2)
            
            if results:
                for j, result in enumerate(results):
                    category = result['metadata']['category']
                    content = result['content'][:80] + "..."
                    print(f"   📋 [{category}] {content}")
            else:
                print("   ❌ No results found")
            print()
        
        print("✅ RAG test completed successfully!")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
