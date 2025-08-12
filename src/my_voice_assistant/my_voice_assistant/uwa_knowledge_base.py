#!/usr/bin/env python3
"""
UWA Knowledge Base RAG System
Manages vector database for UWA campus information retrieval
"""

import os
import json
import hashlib

# Disable ChromaDB telemetry to avoid posthog errors - must be set before import
os.environ["ANONYMIZED_TELEMETRY"] = "False"
os.environ["CHROMA_DISABLE_TELEMETRY"] = "1"
os.environ["DO_NOT_TRACK"] = "1"

import chromadb
from chromadb.config import Settings
from sentence_transformers import SentenceTransformer
from typing import List, Dict, Optional
import logging

# Additional telemetry disable after import
try:
    chromadb.telemetry.posthog.disabled = True
except:
    pass


class UWAKnowledgeBase:
    def __init__(self, db_path: str = "./uwa_knowledge_db"):
        """Initialize UWA Knowledge Base with vector database"""
        # 额外的遥测禁用设置
        os.environ["ANONYMIZED_TELEMETRY"] = "False"
        os.environ["CHROMA_DISABLE_TELEMETRY"] = "1"
        
        self.db_path = db_path
        self.collection_name = "uwa_info"
        
        # Initialize ChromaDB with telemetry disabled
        settings = Settings(
            anonymized_telemetry=False,
            allow_reset=True
        )
        self.chroma_client = chromadb.PersistentClient(path=db_path, settings=settings)
        
        # Initialize embedding model
        self.embedding_model = SentenceTransformer('all-MiniLM-L6-v2')
        
        # Get or create collection
        try:
            self.collection = self.chroma_client.get_collection(name=self.collection_name)
            logging.info(f"Loaded existing collection: {self.collection_name}")
        except:
            self.collection = self.chroma_client.create_collection(
                name=self.collection_name,
                metadata={"description": "UWA Campus Information"}
            )
            logging.info(f"Created new collection: {self.collection_name}")
        
        # New: optional assistant memory collection
        self.memory_collection_name = "assistant_memory"
        try:
            self.memory_collection = self.chroma_client.get_collection(name=self.memory_collection_name)
            logging.info(f"Loaded existing collection: {self.memory_collection_name}")
        except:
            self.memory_collection = self.chroma_client.create_collection(
                name=self.memory_collection_name,
                metadata={"description": "Assistant long-term conversational memory"}
            )
            logging.info(f"Created new collection: {self.memory_collection_name}")
            
        # Initialize with default UWA data if empty
        if self.collection.count() == 0:
            self._initialize_default_data()
    
    def _initialize_default_data(self):
        """Initialize with default UWA campus information"""
        default_knowledge = [
            {
                "content": "UWA Library (Reid Library) is open Monday-Thursday 8am-10pm, Friday 8am-6pm, Saturday 9am-5pm, Sunday 11am-10pm during semester. Located at the heart of campus.",
                "category": "library",
                "building": "Reid Library"
            },
            {
                "content": "Student Central is open Monday-Friday 8:30am-4:30pm. Located in Building 20, they handle enrollment, fees, and general student services.",
                "category": "student_services",
                "building": "Building 20"
            },
            {
                "content": "UWA Sports Centre is open Monday-Friday 5:30am-10pm, Saturday-Sunday 6:30am-8pm. Located on Fairway Road, offers gym, pools, and sports facilities.",
                "category": "recreation",
                "building": "Sports Centre"
            },
            {
                "content": "Hackett Cafe is open Monday-Friday 7am-4pm, located in Hackett Hall. Popular spot for coffee and light meals between classes.",
                "category": "dining",
                "building": "Hackett Hall"
            },
            {
                "content": "To get to UWA by public transport: Take bus routes 21, 23, 102, 103, 106, 107, or 108 to UWA bus station. Train to Crawley station then 10-minute walk.",
                "category": "transport",
                "building": "Campus General"
            },
            {
                "content": "UWA Medical Centre is open Monday-Friday 8am-5pm, located in Building 57. Provides healthcare services for students and staff.",
                "category": "health",
                "building": "Building 57"
            },
            {
                "content": "Pharmacy is located in Building 19, open Monday-Friday 8:30am-5pm. Provides prescription and over-the-counter medications.",
                "category": "health",
                "building": "Building 19"
            },
            {
                "content": "UWA Bookshop is open Monday-Friday 8:30am-4:30pm, located in Building 18. Sells textbooks, stationery, and UWA merchandise.",
                "category": "shopping",
                "building": "Building 18"
            },
            {
                "content": "Guild Village food court is open Monday-Friday 7am-7pm, Saturday 8am-3pm. Multiple food options including Asian, Italian, and healthy choices.",
                "category": "dining",
                "building": "Guild Village"
            },
            {
                "content": "Parking on campus costs $5-15 per day depending on zone. Zones 1-2 are closest to main buildings. Alternative: Park at nearby streets (check time limits).",
                "category": "transport",
                "building": "Campus General"
            },
            {
                "content": "Orientation Week (O-Week) usually occurs in late February. Includes campus tours, club fairs, welcome events, and academic preparation sessions.",
                "category": "events",
                "building": "Campus General"
            },
            {
                "content": "Science Library (in Physics Building) is open Monday-Friday 8am-6pm. Specialized collection for science and engineering students.",
                "category": "library",
                "building": "Physics Building"
            },
            {
                "content": "Campus WiFi: Connect to 'UWA' network using your student ID and password. Guest WiFi available for visitors.",
                "category": "technology",
                "building": "Campus General"
            },
            {
                "content": "Academic Support Centre in Building 30 offers tutoring, study skills workshops, and writing assistance. Open Monday-Friday 9am-5pm.",
                "category": "academic",
                "building": "Building 30"
            },
            {
                "content": "Shuttle bus operates between Crawley campus and other UWA locations. Check UWA website for current schedules and routes.",
                "category": "transport",
                "building": "Campus General"
            }
        ]
        
        self.add_documents(default_knowledge)
        logging.info(f"Initialized {len(default_knowledge)} default knowledge entries")
    
    def add_documents(self, documents: List[Dict]):
        """Add documents to the knowledge base"""
        contents = [doc["content"] for doc in documents]
        embeddings = self.embedding_model.encode(contents).tolist()
        
        # Generate IDs and metadata
        ids = []
        metadatas = []
        
        for i, doc in enumerate(documents):
            # Create unique ID based on content hash
            doc_id = hashlib.md5(doc["content"].encode()).hexdigest()
            ids.append(doc_id)
            
            # Prepare metadata
            metadata = {
                "category": doc.get("category", "general"),
                "building": doc.get("building", "unknown"),
                "source": doc.get("source", "default")
            }
            metadatas.append(metadata)
        
        # Add to collection
        self.collection.add(
            embeddings=embeddings,
            documents=contents,
            metadatas=metadatas,
            ids=ids
        )
        
        logging.info(f"Added {len(documents)} documents to knowledge base")

    # New: assistant memory helpers -------------------------------------------------
    def upsert_memory(self, memory_id: str, content: str, metadata: Optional[Dict] = None):
        """Create or update a single memory document by ID."""
        if not content:
            return
        metadata = metadata or {}
        emb = self.embedding_model.encode([content]).tolist()
        try:
            existing = self.memory_collection.get(ids=[memory_id])
            exists = bool(existing and existing.get("ids") and len(existing["ids"]) > 0)
        except Exception:
            exists = False
        if exists:
            # update
            self.memory_collection.update(
                ids=[memory_id],
                embeddings=emb,
                documents=[content],
                metadatas=[metadata]
            )
            logging.info(f"Updated assistant_memory id={memory_id}")
        else:
            # add
            self.memory_collection.add(
                ids=[memory_id],
                embeddings=emb,
                documents=[content],
                metadatas=[metadata]
            )
            logging.info(f"Added assistant_memory id={memory_id}")

    def add_memory_entries(self, entries: List[Dict]):
        """Add a batch of memory entries. Each item: {content, metadata?, id?}"""
        if not entries:
            return
        contents = [e["content"] for e in entries if e.get("content")]
        if not contents:
            return
        embeddings = self.embedding_model.encode(contents).tolist()
        ids = []
        metadatas = []
        for e in entries:
            content = e.get("content")
            if not content:
                continue
            eid = e.get("id") or hashlib.md5(content.encode()).hexdigest()
            ids.append(eid)
            metadatas.append(e.get("metadata", {}))
        self.memory_collection.add(
            ids=ids,
            embeddings=embeddings,
            documents=contents,
            metadatas=metadatas
        )
        logging.info(f"Added {len(ids)} assistant memory entries")

    def search_memory(self, query: str, n_results: int = 3) -> List[Dict]:
        """Semantic search over assistant memory."""
        if not query:
            return []
        query_embedding = self.embedding_model.encode([query]).tolist()
        results = self.memory_collection.query(
            query_embeddings=query_embedding,
            n_results=n_results,
            include=["documents", "metadatas", "distances"]
        )
        formatted = []
        if results and results.get("documents"):
            for i in range(len(results["documents"][0])):
                formatted.append({
                    "content": results["documents"][0][i],
                    "metadata": results["metadatas"][0][i],
                    "distance": results["distances"][0][i]
                })
        return formatted

    def get_memory_stats(self) -> Dict:
        """Basic stats for assistant memory collection"""
        try:
            count = self.memory_collection.count()
        except Exception:
            count = 0
        return {"total_memories": count}

    def search(self, query: str, n_results: int = 3, category_filter: Optional[str] = None) -> List[Dict]:
        """Search for relevant information"""
        # Generate query embedding
        query_embedding = self.embedding_model.encode([query]).tolist()
        
        # Prepare where clause for filtering
        where_clause = None
        if category_filter:
            where_clause = {"category": category_filter}
        
        # Search in collection
        results = self.collection.query(
            query_embeddings=query_embedding,
            n_results=n_results,
            where=where_clause,
            include=["documents", "metadatas", "distances"]
        )
        
        # Format results
        formatted_results = []
        for i in range(len(results["documents"][0])):
            formatted_results.append({
                "content": results["documents"][0][i],
                "metadata": results["metadatas"][0][i],
                "distance": results["distances"][0][i]
            })
        
        return formatted_results
    
    def get_stats(self) -> Dict:
        """Get knowledge base statistics"""
        count = self.collection.count()
        
        # Get category breakdown
        all_results = self.collection.get(include=["metadatas"])
        categories = {}
        buildings = {}
        
        for metadata in all_results["metadatas"]:
            cat = metadata.get("category", "unknown")
            building = metadata.get("building", "unknown")
            
            categories[cat] = categories.get(cat, 0) + 1
            buildings[building] = buildings.get(building, 0) + 1
        
        return {
            "total_documents": count,
            "categories": categories,
            "buildings": buildings
        }
    
    def add_custom_info(self, content: str, category: str = "custom", building: str = "unknown"):
        """Add a single piece of information"""
        doc = {
            "content": content,
            "category": category,
            "building": building,
            "source": "user_added"
        }
        self.add_documents([doc])
    
    def delete_collection(self):
        """Delete the entire collection (use with caution)"""
        self.chroma_client.delete_collection(name=self.collection_name)
        logging.info("Knowledge base collection deleted")


def main():
    """Test the knowledge base"""
    kb = UWAKnowledgeBase()
    
    # Test searches
    test_queries = [
        "What time does the library open?",
        "How do I get to UWA by bus?",
        "Where can I buy food on campus?",
        "Medical services at UWA",
        "Student services and enrollment"
    ]
    
    print("🧪 Testing UWA Knowledge Base")
    print("=" * 50)
    
    for query in test_queries:
        print(f"\n🔍 Query: {query}")
        results = kb.search(query, n_results=2)
        
        for i, result in enumerate(results, 1):
            print(f"  {i}. [{result['metadata']['category']}] {result['content'][:100]}...")
            print(f"     Building: {result['metadata']['building']}, Distance: {result['distance']:.3f}")
    
    # Show stats
    print(f"\n📊 Knowledge Base Stats:")
    stats = kb.get_stats()
    print(f"  Total documents: {stats['total_documents']}")
    print(f"  Categories: {list(stats['categories'].keys())}")
    print(f"  Buildings: {list(stats['buildings'].keys())}")


if __name__ == "__main__":
    main()
