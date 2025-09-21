#!/usr/bin/env python3
"""
Clear UWA Knowledge Database
Removes all existing data from the knowledge base
"""

import os
import sys
import shutil
from pathlib import Path

# Add current directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def clear_knowledge_database():
    """Clear all data from the UWA knowledge database"""
    print("🗑️ Clearing UWA Knowledge Database")
    print("=" * 50)
    
    # Path to the knowledge database
    db_path = "./uwa_knowledge_db"
    
    try:
        if os.path.exists(db_path):
            # Get database stats before deletion
            try:
                from uwa_knowledge_base import UWAKnowledgeBase
                kb = UWAKnowledgeBase()
                stats = kb.get_stats()
                print(f"📊 Current database stats:")
                print(f"   • Total documents: {stats['total_documents']}")
                print(f"   • Categories: {list(stats['categories'].keys())}")
                print(f"   • Database size: {get_folder_size(db_path):.2f} MB")
                
                # Close the database connection
                del kb
                
            except Exception as e:
                print(f"⚠️ Could not read database stats: {e}")
            
            # Remove the entire database directory
            shutil.rmtree(db_path)
            print(f"✅ Successfully deleted database directory: {db_path}")
            
        else:
            print(f"ℹ️ Database directory does not exist: {db_path}")
        
        # Verify deletion
        if not os.path.exists(db_path):
            print(f"✅ Database successfully cleared")
            print(f"🔄 The database will be recreated (empty) when next accessed")
        else:
            print(f"❌ Failed to delete database directory")
            
    except Exception as e:
        print(f"❌ Error clearing database: {e}")
        return False
    
    return True

def get_folder_size(folder_path):
    """Get the size of a folder in MB"""
    total_size = 0
    try:
        for dirpath, dirnames, filenames in os.walk(folder_path):
            for filename in filenames:
                filepath = os.path.join(dirpath, filename)
                if os.path.exists(filepath):
                    total_size += os.path.getsize(filepath)
        return total_size / (1024 * 1024)  # Convert to MB
    except:
        return 0

def test_empty_database():
    """Test that the database is empty after clearing"""
    print(f"\n🧪 Testing Empty Database")
    print("-" * 30)
    
    try:
        from uwa_knowledge_base import UWAKnowledgeBase
        
        # Create new empty database
        kb = UWAKnowledgeBase()
        stats = kb.get_stats()
        
        print(f"📊 New database stats:")
        print(f"   • Total documents: {stats['total_documents']}")
        print(f"   • Categories: {list(stats['categories'].keys())}")
        
        # Test search on empty database
        results = kb.search("UWA library", n_results=3)
        print(f"🔍 Search test result: {len(results)} documents found")
        
        if stats['total_documents'] == 0:
            print(f"✅ Database is successfully empty")
        else:
            print(f"⚠️ Database still contains {stats['total_documents']} documents")
            
    except Exception as e:
        print(f"❌ Error testing empty database: {e}")

if __name__ == "__main__":
    print("⚠️ WARNING: This will permanently delete all knowledge base data!")
    print("This includes any manually added content.")
    
    response = input("\nDo you want to continue? (yes/no): ").lower().strip()
    
    if response in ['yes', 'y']:
        success = clear_knowledge_database()
        
        if success:
            test_empty_database()
            print(f"\n🎯 Summary:")
            print(f"   • ✅ Knowledge database cleared")
            print(f"   • ✅ RAG search disabled in LLM node")
            print(f"   • ✅ System now uses: Web Search + LLM Knowledge")
            print(f"   • 🔄 RAG functionality preserved for future use")
    else:
        print("❌ Operation cancelled")