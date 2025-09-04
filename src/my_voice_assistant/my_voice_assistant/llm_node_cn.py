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
# RAG功能已移除以提高响应速度
# if __name__ == '__main__':
#     sys.path.append(os.path.dirname(os.path.abspath(__file__)))
#     from uwa_knowledge_base import UWAKnowledgeBase
# else:
#     from .uwa_knowledge_base import UWAKnowledgeBase

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
            #"ft:gpt-4.1-mini-2025-04-14:personal:my-voice-assistant:BxxCKJUa"
            "gpt-5-mini-2025-08-07"
        )
        try:
            self.llm_temperature = float(os.environ.get("LLM_TEMPERATURE", "1"))
        except Exception:
            self.llm_temperature = 1
        try:
            self.llm_max_tokens = int(os.environ.get("LLM_MAX_TOKENS", "2048"))
        except Exception:
            self.llm_max_tokens = 300
        
        # RAG Knowledge Base功能已移除以提高响应速度
        self.knowledge_base = None
        self.get_logger().info("🚀 RAG功能已禁用，使用基础LLM模式")
            
        # New: assistant rolling summary + persistence + optional Chroma memory
        self.enable_rolling_summary = os.environ.get("ROLLING_SUMMARY_ENABLED", "1") == "1"
        self.summary_every_n_turns = int(os.environ.get("ROLLING_SUMMARY_EVERY_N_TURNS", "3"))
        self.summary_max_tokens = int(os.environ.get("ROLLING_SUMMARY_MAX_TOKENS", "180"))
        self.memory_json_path = os.environ.get("ASSISTANT_MEMORY_JSON", "assistant_memory.json")
        self.enable_memory_chroma = os.environ.get("ENABLE_MEMORY_CHROMA", "1") == "1"
        self.memory_search_top_k = int(os.environ.get("MEMORY_SEARCH_TOP_K", "2"))
        self.rolling_summary: str = ""
        self._load_persistent_memory()
            
        self.get_logger().info("🚀 中文LLM节点已启动，等待语音输入... (RAG功能已禁用)")

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
        # 注意：RAG功能已禁用，不再进行知识库内存搜索
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
                temperature=1,
                max_completion_tokens=self.summary_max_tokens,
                stream=False,
            )
            content = resp.choices[0].message.content.strip() if resp.choices else ""
            if content:
                self.rolling_summary = content
                # Persist to JSON
                self._save_persistent_memory()
                # 注意：RAG功能已禁用，不再存储到Chroma
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
        
        self.get_logger().info(f"🎤 收到文本: {input_text}")

        # Call ChatGPT API with RAG enhancement
        self.call_chatgpt_with_rag(input_text)
    
    def search_knowledge_base(self, query: str, n_results: int = 3):
        """RAG功能已禁用，返回空结果"""
        return []
    
    def format_rag_context(self, search_results: list) -> str:
        """RAG功能已禁用，返回空字符串"""
        return ""
    
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
            self.get_logger().info(f"🧹 内容过滤: {original_length} -> {len(filtered_text)} 字符")
        
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
                    self.get_logger().debug(f"📡 刷新待发句子: {cleaned}")
            self._pending_sentences.clear()

    def tts_interrupt_callback(self, msg: Bool):
        if msg.data:
            self._interrupted = True
            self.get_logger().info("⛔ 收到 tts_interrupt，中止本次流式输出")

    def call_chatgpt_with_rag(self, prompt: str) -> str:
        """简化的ChatGPT调用，不使用RAG以提高响应速度"""
        try:
            # 不进行知识库搜索，直接使用基础LLM
            self.get_logger().info("🚀 使用基础LLM模式 (无RAG)")
            
            # Optional: assistant memory context (rolling summary only)
            memory_context = self._format_memory_context(prompt)
            
            # Step 2: Build basic system prompt
            base_system_prompt = """你是Captain，西澳大学（UWA）班车服务个友善语音助手。你个性格特点是：

角色和背景：
- 你是车上个AI助手，帮助UWA个学生、教工搭仔访客
- 你提供班车路线、校园位置搭仔交通信息
- 你对UWA校园设施搭仔服务侪晓得

性格特点：
- 热情、欢迎搭仔专业
- 对所有乘客侪有耐心、理解
- 有用搭仔信息丰富，弗会太复杂
- 用亲切个苏州话风格
- 适当时候用点温和个幽默

📍 主要知识领域：
- UWA班车路线搭仔时刻表
- 校园建筑搭仔设施
- 学生服务搭仔便民设施
- 校园导航帮助
- 安全搭仔无障碍信息

交流风格：
- 回答要简洁但完整（争取1-3句话）
- 用清爽个标点搭仔适合TTS个愉快语调
- 用苏州吴语风格，但要让人听得懂
- 直接搭仔亲切地称呼乘客
- 需要时候主动提供额外帮助
- 表达对乘客体验个真诚关心

记牢：侬弗单单是提供信息 - 侬要让UWA班车上个每个人侪有更好个出行体验！

语言要求：只用中文回答，使用苏州吴语风格，但要保证大家侪听得懂。"""

            # Add Memory context if available
            if memory_context:
                system_prompt = f"{base_system_prompt}\n\n{memory_context}"
                self.get_logger().info("🧠 注入助手记忆上下文到提示词中")
            else:
                system_prompt = base_system_prompt
            
            # Step 3: Make API call
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
                temperature=1,
                max_completion_tokens=self.llm_max_tokens,
                stream=True
            )

            self.get_logger().info("🚀 开始基础回答生成...")
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
                        self.get_logger().debug(f"📡 发布句子: {msg.data}")

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
                        self.get_logger().debug(f"📡 发布剩余部分: {remainder}")

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
            self.get_logger().info(f"✅ 💬 直接回答完成: "
                                 f"{len(final_reply)} 字符 -> {len(final_filtered)} 字符过滤后")
            self.get_logger().info(f"📝 完整回答: {final_filtered}")

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
            self.get_logger().error(f"❌ ChatGPT API错误: {str(e)}")
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