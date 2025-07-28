#!/usr/bin/env python3
"""
UWA Knowledge Base Expansion Script
Adds comprehensive new student information to the RAG database
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from uwa_knowledge_base import UWAKnowledgeBase

def expand_knowledge_base():
    """Add comprehensive UWA new student information"""
    print("📚 Expanding UWA Knowledge Base")
    print("=" * 50)
    
    try:
        kb = UWAKnowledgeBase()
        print("✅ Connected to knowledge base")
        
        # Extended UWA new student information
        extended_knowledge = [
            # Academic Buildings & Hours
            {
                "content": "Winthrop Hall is UWA's iconic clock tower building, used for graduation ceremonies and special events. It's located at the center of campus and is a popular meeting point.",
                "category": "landmarks",
                "building": "Winthrop Hall"
            },
            {
                "content": "Engineering Building (Building 21) houses the School of Engineering and Applied Sciences. Has computer labs, lecture theaters, and study spaces. Open 24/7 for students with card access.",
                "category": "academic",
                "building": "Building 21"
            },
            {
                "content": "Business School Building (Building 39) is home to the UWA Business School. Features modern lecture theaters, seminar rooms, and collaborative learning spaces.",
                "category": "academic", 
                "building": "Building 39"
            },
            
            # Detailed Transport Information
            {
                "content": "TransPerth bus routes to UWA: Route 21 (Claremont-UWA), Route 23 (Scarborough Beach-UWA), Route 102 (Marmion-UWA), Route 103 (Whitfords-UWA). Buses run every 15-30 minutes during peak times.",
                "category": "transport",
                "building": "Campus General"
            },
            {
                "content": "UWA is accessible by train via Crawley Station on the Fremantle line. From the station, it's a 10-15 minute walk to campus or take the free CAT bus.",
                "category": "transport",
                "building": "Campus General"
            },
            {
                "content": "Car parking zones on campus: Purple zones (staff only), Blue zones ($5/day), Green zones ($10/day), Yellow zones ($15/day). Free parking available on surrounding streets with time limits.",
                "category": "transport",
                "building": "Campus General"
            },
            
            # Food & Dining Extended
            {
                "content": "Prescott Court Food Court offers diverse food options including Asian cuisine, pizza, sandwiches, and healthy options. Located between the Library and Student Services.",
                "category": "dining",
                "building": "Prescott Court"
            },
            {
                "content": "Ref Cafe in Reid Library offers coffee, light meals, and study snacks. Popular during exam periods. Open Monday-Friday 7:30am-4pm.",
                "category": "dining",
                "building": "Reid Library"
            },
            {
                "content": "Tavern on the Campus is UWA's licensed venue offering meals, drinks, and entertainment. Popular for student social events and celebrations.",
                "category": "dining",
                "building": "Tavern"
            },
            
            # Student Services Extended
            {
                "content": "International Student Services provides support for visa issues, orientation programs, cultural activities, and academic support specifically for international students.",
                "category": "student_services",
                "building": "Building 20"
            },
            {
                "content": "Counselling and Psychological Services (CAPS) offers free confidential counselling for students. Located in Building 25, appointments available Monday-Friday.",
                "category": "health",
                "building": "Building 25"
            },
            {
                "content": "Disability Services provides support for students with disabilities including accessible facilities, assistive technology, and academic accommodations.",
                "category": "student_services",
                "building": "Building 20"
            },
            
            # Accommodation Information
            {
                "content": "On-campus residential colleges: St. George's College, St. Thomas More College, Trinity Residential College, and University Hall. Applications typically open in August for following year.",
                "category": "accommodation",
                "building": "Campus General"
            },
            {
                "content": "Off-campus housing options include suburbs like Crawley, Nedlands, Dalkeith, and Subiaco. Many students share houses or apartments. Rental costs vary from $150-400 per week.",
                "category": "accommodation",
                "building": "Off Campus"
            },
            
            # Academic Support Extended
            {
                "content": "STUDYSmarter program offers free workshops on time management, note-taking, exam preparation, and research skills. Bookings available online through StudentConnect.",
                "category": "academic",
                "building": "Building 30"
            },
            {
                "content": "Peer Assisted Study Sessions (PASS) are weekly review sessions for difficult units, led by students who previously excelled in those units. Free to attend.",
                "category": "academic",
                "building": "Various Locations"
            },
            {
                "content": "Writing Centre provides one-on-one consultations to help improve academic writing skills. Particularly helpful for essay assignments and thesis writing.",
                "category": "academic",
                "building": "Building 30"
            },
            
            # Library Services Extended
            {
                "content": "Library study spaces include silent study areas, group study rooms (bookable), 24/7 study zones, and collaborative learning spaces. Group rooms require advance booking.",
                "category": "library",
                "building": "Reid Library"
            },
            {
                "content": "Research support services include librarian consultations, database training, citation style guidance, and thesis support. Available to all enrolled students.",
                "category": "library",
                "building": "Reid Library"
            },
            
            # Technology & IT Extended
            {
                "content": "Student IT support available at ITS Help Desk in Building 22. Services include computer troubleshooting, software installation, email setup, and WiFi assistance.",
                "category": "technology",
                "building": "Building 22"
            },
            {
                "content": "Computer labs available in multiple buildings with Windows and Mac computers. Some labs are 24/7 access with student ID card. Printing services available.",
                "category": "technology",
                "building": "Various Locations"
            },
            
            # Recreation & Fitness Extended
            {
                "content": "UWA Sports Centre facilities include gym, 50m swimming pool, basketball courts, squash courts, and group fitness classes. Membership fees apply for external users.",
                "category": "recreation",
                "building": "Sports Centre"
            },
            {
                "content": "Outdoor recreational facilities include tennis courts, oval for football/cricket, and riverside walking/cycling paths along the Swan River.",
                "category": "recreation",
                "building": "Campus General"
            },
            
            # Student Clubs & Activities
            {
                "content": "UWA Student Guild represents student interests and organizes events, clubs, and services. Guild membership is automatic for all enrolled students.",
                "category": "student_life",
                "building": "Guild Village"
            },
            {
                "content": "Over 140 student clubs and societies covering interests from academic disciplines to hobbies, sports, and cultural groups. Join during O-Week club fair.",
                "category": "student_life",
                "building": "Campus General"
            },
            
            # Emergency & Safety
            {
                "content": "Campus Security operates 24/7 with emergency phone points located throughout campus. Security escorts available after dark. Emergency number: 6488 2222.",
                "category": "safety",
                "building": "Campus General"
            },
            {
                "content": "First Aid rooms located in multiple buildings including Student Central, Sports Centre, and Library. Trained first aid officers available during business hours.",
                "category": "health",
                "building": "Various Locations"
            },
            
            # Important Dates & Deadlines
            {
                "content": "Academic year runs from late February to November. Semester 1: Feb-June, Semester 2: July-November. Study break and exam periods included in each semester.",
                "category": "academic",
                "building": "Campus General"
            },
            {
                "content": "Key enrollment deadlines: Census dates (can't withdraw without penalty after this), Add/drop periods, and withdrawal deadlines vary by semester and unit.",
                "category": "academic",
                "building": "Campus General"
            }
        ]
        
        print(f"📝 Adding {len(extended_knowledge)} new knowledge entries...")
        
        # Add the extended knowledge
        kb.add_documents(extended_knowledge)
        
        # Show updated stats
        final_stats = kb.get_stats()
        print(f"✅ Knowledge base expanded successfully!")
        print(f"📊 Total documents: {final_stats['total_documents']}")
        print(f"📂 Categories: {list(final_stats['categories'].keys())}")
        
        # Test some of the new information
        print(f"\n🔍 Testing new knowledge retrieval:")
        print("-" * 50)
        
        test_queries = [
            "Where can I live on campus?",
            "What sports facilities are available?", 
            "How do I get academic writing help?",
            "What clubs can I join?",
            "Emergency contact information"
        ]
        
        for query in test_queries:
            print(f"\nQuery: '{query}'")
            results = kb.search(query, n_results=2)
            for i, result in enumerate(results, 1):
                category = result['metadata']['category']
                content = result['content'][:80] + "..."
                print(f"  {i}. [{category}] {content}")
        
        print(f"\n✅ Knowledge base expansion completed!")
        
    except Exception as e:
        print(f"❌ Expansion failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    expand_knowledge_base()
