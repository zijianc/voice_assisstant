#!/usr/bin/env python3
import os
import re
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI
from dotenv import load_dotenv
from datetime import datetime, time
from typing import List, Dict

# Handle relative imports when running as main module
if __name__ == '__main__':
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from uwa_knowledge_base import UWAKnowledgeBase
else:
    from .uwa_knowledge_base import UWAKnowledgeBase

load_dotenv()


class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')
        # Subscribe to text from Vosk node, topic name is "speech_text"
        self.subscription = self.create_subscription(
            String,
            'speech_text',
            self.listener_callback,
            10
        )
        # Publish ChatGPT response results to topic "llm_response"
        self.publisher_ = self.create_publisher(String, 'llm_response', 10)
        
        # Get OpenAI API key from environment variables
        self.api_key = os.environ.get("OPENAI_API_KEY")
        if self.api_key is None:
            self.get_logger().error("Please set environment variable OPENAI_API_KEY!")
        else:
            self.client = OpenAI(api_key=self.api_key)
            
        # Initialize RAG Knowledge Base
        try:
            self.knowledge_base = UWAKnowledgeBase()
            self.get_logger().info("✅ RAG Knowledge Base initialized successfully")
            
            # Log knowledge base stats
            stats = self.knowledge_base.get_stats()
            self.get_logger().info(f"📊 Knowledge Base: {stats['total_documents']} documents, "
                                 f"Categories: {list(stats['categories'].keys())}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize Knowledge Base: {e}")
            self.knowledge_base = None
            
        self.get_logger().info("🚀 LLM node with RAG started, waiting for voice input...")

        # Store past dialogue turns (user/assistant) to maintain conversational context
        self.conversation_history: list[dict[str, str]] = []  # each item: {"role": "...", "content": "..."}
        self.max_history_messages = 20   # keep at most the last 20 message objects (10 turns)

    def listener_callback(self, msg: String):
        input_text = msg.data.strip()
        if not input_text:
            return

        # Print user input clearly
        print("\n" + "="*60)
        print("🎤 用户语音输入:")
        print("-"*60)
        print(f"'{input_text}'")
        print("="*60)
        
        self.get_logger().info(f"🎤 Received text: {input_text}")

        # Call ChatGPT API with RAG enhancement
        self.call_chatgpt_with_rag(input_text)
    
    def search_knowledge_base(self, query: str, n_results: int = 3):
        """Search UWA knowledge base for relevant information"""
        if not self.knowledge_base:
            return []
            
        try:
            results = self.knowledge_base.search(query, n_results=n_results)
            
            if results:
                self.get_logger().info(f"🔍 Found {len(results)} relevant knowledge entries")
                for i, result in enumerate(results):
                    self.get_logger().debug(f"  {i+1}. [{result['metadata']['category']}] "
                                          f"{result['content'][:50]}... (distance: {result['distance']:.3f})")
            else:
                self.get_logger().info("🔍 No relevant knowledge found in database")
                
            return results
            
        except Exception as e:
            self.get_logger().error(f"❌ Knowledge search error: {e}")
            return []
    
    def format_rag_context(self, search_results: list) -> str:
        """Format search results into context for the LLM"""
        if not search_results:
            return ""
            
        context_parts = ["📚 RELEVANT UWA INFORMATION:"]
        
        for i, result in enumerate(search_results, 1):
            building = result['metadata'].get('building', 'Unknown')
            category = result['metadata'].get('category', 'general')
            content = result['content']
            
            context_parts.append(f"{i}. [{category.upper()}] {content}")
            if building != 'Campus General' and building != 'unknown':
                context_parts[-1] += f" (Located: {building})"
        
        context_parts.append("\nPlease use this information to provide accurate, helpful responses about UWA.")
        
        return "\n".join(context_parts)
    
    def filter_content(self, text: str) -> str:
        """Enhanced content filter for LLM responses"""
        if not text:
            return text
            
        original_length = len(text)
        
        # Remove various noise patterns
        filtered_text = text
        
        # Remove bracket content with IDs/references
        filtered_text = re.sub(r'【[^】]*】', '', filtered_text)
        
        # Remove long number sequences (likely references)
        filtered_text = re.sub(r'\b\d{6,}\b', '', filtered_text)
        
        # Remove special reference symbols
        filtered_text = re.sub(r'[†‑]+', '', filtered_text)
        
        # Remove line reference patterns (L123-L456)
        filtered_text = re.sub(r'L\d+-L\d+', '', filtered_text)
        
        # Clean up formatting
        filtered_text = re.sub(r'\s+', ' ', filtered_text)  # Multiple spaces
        filtered_text = re.sub(r'\s*\.\s*\.+', '.', filtered_text)  # Multiple dots
        filtered_text = filtered_text.strip()
        
        # Log filtering if significant
        if len(filtered_text) != original_length:
            self.get_logger().info(f"🧹 Content filtered: {original_length} -> {len(filtered_text)} chars")
        
        return filtered_text

    def call_chatgpt_with_rag(self, prompt: str) -> str:
        """Enhanced ChatGPT call with RAG support"""
        try:
            # Step 1: Search knowledge base for relevant context
            search_results = self.search_knowledge_base(prompt, n_results=3)
            rag_context = self.format_rag_context(search_results)
            
            # Print RAG search results if found
            if search_results:
                print("\n" + "="*60)
                print("🔍 RAG 知识库搜索结果:")
                print("-"*60)
                for i, result in enumerate(search_results, 1):
                    category = result['metadata']['category']
                    building = result['metadata']['building']
                    content = result['content'][:80] + "..." if len(result['content']) > 80 else result['content']
                    distance = result['distance']
                    
                    print(f"{i}. [{category}] {content}")
                    if building not in ['Campus General', 'unknown']:
                        print(f"   📍 位置: {building}")
                    print(f"   📊 相关度: {(1-distance)*100:.1f}%")
                print("="*60)
            else:
                print("\n" + "="*60)
                print("🔍 RAG 搜索: 未找到相关知识库信息")
                print("="*60)
            
            # Step 2: Build enhanced system prompt
            base_system_prompt = """You are Captain, the friendly voice assistant for the UWA (University of Western Australia) shuttle bus service. Your personality is:

ROLE & CONTEXT:
- You're an onboard AI assistant helping UWA students, staff, and visitors
- You provide information about shuttle routes, campus locations, and transportation
- You're knowledgeable about UWA campus facilities and services

PERSONALITY TRAITS:
- Warm, welcoming, and professional
- Patient and understanding with all passengers
- Helpful and informative without being overwhelming
- Use a conversational, friendly tone
- Occasionally use gentle humor when appropriate

📍 KEY KNOWLEDGE AREAS:
- UWA shuttle bus routes and schedules
- Campus buildings and facilities
- Student services and amenities
- General campus navigation help
- Safety and accessibility information

COMMUNICATION STYLE:
- Keep responses concise but complete (aim for 1-3 sentences)
- Use clear, simple language suitable for all passengers
- Address passengers directly and personally
- Offer additional help when relevant
- Express genuine care for passenger experience

IMPORTANT: When you have relevant UWA information provided below, use it to give accurate, up-to-date answers. Always prioritize the provided information over general knowledge.

Remember: You're not just providing information - you're enhancing the journey experience for everyone aboard the UWA shuttle!"""

            # Add RAG context if available
            if rag_context:
                system_prompt = f"{base_system_prompt}\n\n{rag_context}"
                self.get_logger().info(f"🧠 Enhanced prompt with {len(search_results)} knowledge entries")
            else:
                system_prompt = base_system_prompt
                self.get_logger().info("🧠 Using base prompt (no relevant knowledge found)")
            
            # Step 3: Make API call with enhanced context
            # ---- Build message list with contextual history ----
            messages: list[dict[str, str]] = [{"role": "system", "content": system_prompt}]

            # Append the most recent history (keep it short to control token usage)
            if self.conversation_history:
                # keep only the last N messages
                messages.extend(self.conversation_history[-self.max_history_messages:])

            # Add the new user message
            messages.append({"role": "user", "content": prompt})
            
            response = self.client.chat.completions.create(
                model="ft:gpt-4.1-mini-2025-04-14:personal:my-voice-assistant:BxxCKJUa",
                messages=messages,
                temperature=0.7,
                max_tokens=1024,
                stream=True
            )

            self.get_logger().info("🚀 Starting RAG-enhanced response generation...")
            final_reply = ""
            filtered_segments = []

            for chunk in response:
                if chunk.choices:
                    delta = chunk.choices[0].delta
                    content = delta.content if delta.content else ""

                    if content:
                        final_reply += content
                        
                        # Apply content filtering to each segment
                        filtered_content = self.filter_content(content)
                        
                        if filtered_content.strip():  # Only publish non-empty filtered content
                            filtered_segments.append(filtered_content)
                            
                            # Publish filtered streaming content
                            msg = String()
                            msg.data = filtered_content
                            self.publisher_.publish(msg)
                            
                            self.get_logger().debug(f"📡 Published filtered: {filtered_content}")

            # Final filtering and validation
            final_filtered = self.filter_content(final_reply)
            
            # Print and log the complete LLM response
            print("\n" + "="*60)
            print("🤖 LLM 完整回答:")
            print("-"*60)
            print(final_filtered)
            print("="*60 + "\n")
            
            # Log final results
            rag_indicator = "🔍 RAG" if search_results else "💬 Direct"
            self.get_logger().info(f"✅ {rag_indicator} response complete: "
                                 f"{len(final_reply)} chars -> {len(final_filtered)} chars filtered")
            self.get_logger().info(f"📝 Complete Response: {final_filtered}")
            
            if search_results:
                self.get_logger().info(f"📚 Used knowledge from: "
                                     f"{', '.join([r['metadata']['category'] for r in search_results])}")

            # ---- Update conversation history ----
            self.conversation_history.append({"role": "user", "content": prompt})
            self.conversation_history.append({"role": "assistant", "content": final_filtered})

            # Truncate history list if it grows beyond the cap
            if len(self.conversation_history) > self.max_history_messages:
                excess = len(self.conversation_history) - self.max_history_messages
                self.conversation_history = self.conversation_history[excess:]

            return final_filtered

        except Exception as e:
            self.get_logger().error(f"❌ Enhanced ChatGPT API error: {str(e)}")
            return ""
def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()