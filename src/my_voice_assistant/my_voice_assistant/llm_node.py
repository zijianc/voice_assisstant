#!/usr/bin/env python3
import os
import re
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from openai import OpenAI
from dotenv import load_dotenv
from datetime import datetime, time
from typing import List, Dict
import json

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
        # New: publish full reply and end signal
        self.full_publisher_ = self.create_publisher(String, 'llm_response_full', 10)
        self.end_publisher_ = self.create_publisher(Bool, 'llm_response_end', 10)
        # New: subscribe to tts_status (pause streaming) and tts_interrupt (cancel streaming)
        self.tts_status_sub = self.create_subscription(Bool, 'tts_status', self.tts_status_callback, 10)
        self.tts_interrupt_sub = self.create_subscription(Bool, 'tts_interrupt', self.tts_interrupt_callback, 10)
        self._tts_playing = False
        self._interrupted = False
        self._pending_sentences: list[str] = []  # 新增：TTS期间暂存句子
        
        # Get OpenAI API key from environment variables
        self.api_key = os.environ.get("OPENAI_API_KEY")
        if self.api_key is None:
            self.get_logger().error("Please set environment variable OPENAI_API_KEY!")
        else:
            self.client = OpenAI(api_key=self.api_key)
        
        # New: env-driven LLM configuration
        self.llm_model = os.environ.get(
            "LLM_MODEL",
            "ft:gpt-4.1-mini-2025-04-14:personal:my-voice-assistant:BxxCKJUa"
        )
        try:
            self.llm_temperature = float(os.environ.get("LLM_TEMPERATURE", "0.6"))
        except Exception:
            self.llm_temperature = 0.6
        try:
            self.llm_max_tokens = int(os.environ.get("LLM_MAX_TOKENS", "300"))
        except Exception:
            self.llm_max_tokens = 300
        
        # Initialize RAG Knowledge Base
        try:
            self.knowledge_base = UWAKnowledgeBase()
            self.get_logger().info("✅ RAG Knowledge Base initialized successfully")
            
            # Log knowledge base stats
            stats = self.knowledge_base.get_stats()
            self.get_logger().info(f"📊 Knowledge Base: {stats['total_documents']} documents, "
                                 f"Categories: {list(stats['categories'].keys())}")
        except Exception as e:
            self.get_logger().error(f"❌ 知识库初始化失败: {e}")
            self.get_logger().warning("🔧 尝试重建知识库...")
            try:
                # 删除损坏的数据库并重新创建
                import shutil
                db_path = "./uwa_knowledge_db"
                if os.path.exists(db_path):
                    shutil.rmtree(db_path)
                    self.get_logger().info("🗑️ 已删除损坏的知识库")
                
                # 重新初始化
                self.knowledge_base = UWAKnowledgeBase()
                self.get_logger().info("✅ 知识库重建成功")
            except Exception as e2:
                self.get_logger().error(f"❌ 知识库重建失败: {e2}")
                self.knowledge_base = None
            
        # New: assistant rolling summary + persistence + optional Chroma memory
        self.enable_rolling_summary = os.environ.get("ROLLING_SUMMARY_ENABLED", "1") == "1"
        self.summary_every_n_turns = int(os.environ.get("ROLLING_SUMMARY_EVERY_N_TURNS", "3"))
        self.summary_max_tokens = int(os.environ.get("ROLLING_SUMMARY_MAX_TOKENS", "180"))
        self.memory_json_path = os.environ.get("ASSISTANT_MEMORY_JSON", "assistant_memory.json")
        self.enable_memory_chroma = os.environ.get("ENABLE_MEMORY_CHROMA", "1") == "1"
        self.memory_search_top_k = int(os.environ.get("MEMORY_SEARCH_TOP_K", "2"))
        self.rolling_summary: str = ""
        self._load_persistent_memory()
            
        self.get_logger().info("🚀 LLM node with RAG started, waiting for voice input...")

        # Store past dialogue turns (user/assistant) to maintain conversational context
        self.conversation_history: list[dict[str, str]] = []  # each item: {"role": "...", "content": "..."}
        self.max_history_messages = 20   # keep at most the last 20 message objects (10 turns)

    # ------------------------- Memory persistence helpers -------------------------
    def _load_persistent_memory(self):
        try:
            if os.path.exists(self.memory_json_path):
                with open(self.memory_json_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    self.rolling_summary = data.get('rolling_summary', '')
                    self.get_logger().info("🧠 Loaded rolling summary from JSON")
        except Exception as e:
            self.get_logger().warning(f"⚠️ Failed to load assistant memory JSON: {e}")

    def _save_persistent_memory(self):
        try:
            data = {
                'rolling_summary': self.rolling_summary,
                'updated_at': datetime.utcnow().isoformat() + 'Z'
            }
            with open(self.memory_json_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
        except Exception as e:
            self.get_logger().warning(f"⚠️ Failed to save assistant memory JSON: {e}")

    def _get_last_assistant_message(self) -> str:
        for item in reversed(self.conversation_history):
            if item.get('role') == 'assistant':
                return item.get('content', '')
        return ""

    def _augment_if_affirmation(self, user_text: str) -> str:
        # Follow-up handling for short affirmations like yes/yeah
        if re.match(r"^(yes|yeah|yep|yup|sure|ok|okay)\W*$", user_text, re.IGNORECASE):
            last_assistant = self._get_last_assistant_message()
            if last_assistant:
                return (
                    f"[User affirmation]\n"
                    f"The user replied 'yes' to your previous message.\n"
                    f"Previous assistant message:\n{last_assistant}\n"
                    f"Please proceed with the next helpful step or provide the requested information succinctly."
                )
        return user_text

    def _format_memory_context(self, user_query: str) -> str:
        parts: list[str] = []
        if self.enable_rolling_summary and self.rolling_summary:
            parts.append("🧠 ASSISTANT MEMORY SUMMARY (for continuity):\n" + self.rolling_summary.strip())
        # Optional: query Chroma assistant_memory for relevant items
        try:
            if self.enable_memory_chroma and self.knowledge_base:
                mem_hits = self.knowledge_base.search_memory(user_query, n_results=self.memory_search_top_k)
                if mem_hits:
                    parts.append("\n🧠 RELATED CONVERSATION MEMORY:")
                    for i, hit in enumerate(mem_hits, 1):
                        parts.append(f"{i}. {hit['content']}")
        except Exception as e:
            self.get_logger().warning(f"⚠️ Memory search error: {e}")
        return "\n\n".join(parts) if parts else ""

    def _maybe_update_rolling_summary(self):
        if not self.enable_rolling_summary:
            return
        # Trigger every N user turns
        user_turns = sum(1 for m in self.conversation_history if m.get('role') == 'user')
        if user_turns % max(1, self.summary_every_n_turns) != 0:
            return
        # Build a concise summarization prompt
        recent = self.conversation_history[-10:]  # last ~5 turns
        # Convert to a readable transcript
        transcript_lines = [f"{m['role']}: {m['content']}" for m in recent]
        transcript = "\n".join(transcript_lines)
        system_prompt = (
            "You are a memory summarizer for the Captain voice assistant. "
            "Given the previous rolling summary and the latest conversation excerpt, "
            "produce a compact summary in 2-3 sentences focusing on: stable facts about the user, "
            "preferences, constraints, names, and any unresolved tasks or commitments. Avoid ephemeral details."
        )
        user_payload = (
            f"Previous summary (may be empty):\n{self.rolling_summary}\n\n"
            f"Recent conversation excerpt:\n{transcript}\n\n"
            f"Write the updated summary only."
        )
        try:
            resp = self.client.chat.completions.create(
                model=self.llm_model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_payload},
                ],
                temperature=0.2,
                max_tokens=self.summary_max_tokens,
                stream=False,
            )
            content = resp.choices[0].message.content.strip() if resp.choices else ""
            if content:
                self.rolling_summary = content
                # Persist to JSON
                self._save_persistent_memory()
                # Also upsert into Chroma assistant_memory
                try:
                    if self.enable_memory_chroma and self.knowledge_base:
                        self.knowledge_base.upsert_memory(
                            memory_id="rolling_summary",
                            content=self.rolling_summary,
                            metadata={"type": "summary", "updated": datetime.utcnow().isoformat() + 'Z'}
                        )
                except Exception as e:
                    self.get_logger().warning(f"⚠️ Failed to upsert rolling summary into Chroma: {e}")
                self.get_logger().info("🧠 Rolling summary updated")
        except Exception as e:
            self.get_logger().warning(f"⚠️ Rolling summary update failed: {e}")

    # -----------------------------------------------------------------------------
    def listener_callback(self, msg: String):
        input_text = msg.data.strip()
        if not input_text:
            return
        # Follow-up handling for short affirmations
        input_text = self._augment_if_affirmation(input_text)

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
                for i, result in enumerate(results, 1):
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

    def tts_status_callback(self, msg: Bool):
        prev = self._tts_playing
        self._tts_playing = bool(msg.data)
        # 当TTS结束时，刷新期间积累的句子，减少分片但不丢内容
        if prev and not self._tts_playing and self._pending_sentences:
            for s in self._pending_sentences:
                cleaned = self.filter_content(s)
                if cleaned:
                    m = String(); m.data = cleaned
                    self.publisher_.publish(m)
                    self.get_logger().debug(f"📡 Flush pending sentence: {cleaned}")
            self._pending_sentences.clear()

    def tts_interrupt_callback(self, msg: Bool):
        if msg.data:
            self._interrupted = True
            self.get_logger().info("⛔ 收到 tts_interrupt，中止本次流式输出")

    def call_chatgpt_with_rag(self, prompt: str) -> str:
        """Enhanced ChatGPT call with RAG support + sentence-level streaming + end/full topics"""
        try:
            # Step 1: Search knowledge base for relevant context
            search_results = self.search_knowledge_base(prompt, n_results=3)
            rag_context = self.format_rag_context(search_results)
            
            # Optional: assistant memory context (rolling summary + relevant memory hits)
            memory_context = self._format_memory_context(prompt)
            
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
- Use clear punctuation and an upbeat, cheerful tone suitable for spoken TTS
- Use clear, simple language suitable for all passengers
- Address passengers directly and personally
- Offer additional help when relevant
- Express genuine care for passenger experience

IMPORTANT: When you have relevant UWA information provided below, use it to give accurate, up-to-date answers. Always prioritize the provided information over general knowledge.

Remember: You're not just providing information - you're enhancing the journey experience for everyone aboard the UWA shuttle!"""

            # Add RAG + Memory context if available
            context_blocks = []
            if rag_context:
                context_blocks.append(rag_context)
                self.get_logger().info(f"🧠 Enhanced prompt with {len(search_results)} knowledge entries")
            else:
                self.get_logger().info("🧠 Using base prompt (no relevant knowledge found)")
            if memory_context:
                context_blocks.append(memory_context)
                self.get_logger().info("🧠 Injected assistant memory context into prompt")
            
            if context_blocks:
                system_prompt = f"{base_system_prompt}\n\n" + "\n\n".join(context_blocks)
            else:
                system_prompt = base_system_prompt
            
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
                model=self.llm_model,
                messages=messages,
                temperature=self.llm_temperature,
                max_tokens=self.llm_max_tokens,
                stream=True
            )

            self.get_logger().info("🚀 Starting RAG-enhanced response generation...")
            final_reply = ""
            sentence_buf = ""

            def flush_sentences_if_ready(force: bool = False):
                nonlocal sentence_buf
                out = []
                pattern = re.compile(r'[^.!?]*[.!?]')
                while True:
                    m = pattern.match(sentence_buf)
                    if not m:
                        break
                    s = m.group(0)
                    if not s.strip():
                        break
                    out.append(s.strip())
                    sentence_buf = sentence_buf[len(s):]
                if not out:
                    return
                # 如果TTS正在播放，则暂存，避免过度碎片化
                if self._tts_playing and not force:
                    self._pending_sentences.extend(out)
                    return
                # 否则立刻发布
                for s in out:
                    msg = String(); msg.data = self.filter_content(s)
                    if msg.data:
                        self.publisher_.publish(msg)
                        self.get_logger().debug(f"📡 Published sentence: {msg.data}")

            for chunk in response:
                if self._interrupted:
                    self.get_logger().info("⏹️ 流式输出被中止 (interrupt)")
                    break
                if chunk.choices:
                    delta = chunk.choices[0].delta
                    content = delta.content if delta.content else ""
                    if content:
                        final_reply += content
                        # buffer and attempt flushing on sentence boundaries
                        sentence_buf += content
                        flush_sentences_if_ready(force=False)

            # 处理剩余缓冲
            if sentence_buf.strip():
                remainder = self.filter_content(sentence_buf.strip())
                if remainder:
                    if self._tts_playing:
                        self._pending_sentences.append(remainder)
                    else:
                        msg = String(); msg.data = remainder
                        self.publisher_.publish(msg)
                        self.get_logger().debug(f"📡 Published remainder: {remainder}")

            # Final filtering and publish full + end
            final_filtered = self.filter_content(final_reply)
            if final_filtered:
                full_msg = String(); full_msg.data = final_filtered
                self.full_publisher_.publish(full_msg)
            end_msg = Bool(); end_msg.data = True
            self.end_publisher_.publish(end_msg)

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
            # Note: store the raw filtered assistant reply
            self.conversation_history.append({"role": "user", "content": prompt})
            self.conversation_history.append({"role": "assistant", "content": final_filtered})
            if len(self.conversation_history) > self.max_history_messages:
                excess = len(self.conversation_history) - self.max_history_messages
                self.conversation_history = self.conversation_history[excess:]

            # After updating history, maybe refresh rolling summary and persist
            self._maybe_update_rolling_summary()

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