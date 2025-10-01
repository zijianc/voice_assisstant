#!/usr/bin/env python3
import os
import re
import sys
import json
import asyncio
import aiohttp
import threading
import time
import tempfile
import wave
import pyaudio
from datetime import datetime
from typing import Dict, List, Optional, Union

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from aiohttp import WSMsgType
from dotenv import load_dotenv

# Handle relative imports when running as main module
if __name__ == '__main__':
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from web_search_tools import WebSearchTools
    from extended_uwa_tools import ExtendedUWATools
else:
    from .web_search_tools import WebSearchTools
    from .extended_uwa_tools import ExtendedUWATools

load_dotenv()


class OpenAIRealtimeNode(Node):
    """
    OpenAI Realtime API node that combines LLM and TTS functionality
    using WebSocket connection for low-latency voice interactions
    """
    
    def __init__(self):
        super().__init__('openai_realtime_node')
        
        # Configuration from environment variables
        self.api_key = os.environ.get("OPENAI_API_KEY")
        if not self.api_key:
            self.get_logger().error("Please set environment variable OPENAI_API_KEY!")
            return
            
        self.realtime_model = os.environ.get("REALTIME_MODEL", "gpt-realtime-2025-08-28")
        #self.realtime_model = os.environ.get("REALTIME_MODEL", "gpt-4o-realtime-preview-2025-06-03")
        self.realtime_voice = os.environ.get("REALTIME_VOICE", "cedar")  # alloy, echo, fable, onyx, nova, shimmer
        self.enable_function_calling = os.environ.get("ENABLE_REALTIME_FUNCTION_CALLING", "1") == "1"
        
        # Audio configuration
        self.audio_sample_rate = 24000  # OpenAI Realtime API requirement
        self.audio_chunk_size = 2048
        
        # ROS2 Publishers and Subscribers
        self.setup_ros_communication()
        
                # Initialize Web Search Tools for function calling
        if self.enable_function_calling:
            try:
                self.web_search_tools = ExtendedUWATools()  # Use extended tools instead of basic
                self.get_logger().info("✅ Extended UWA Tools initialized for Realtime API")
            except Exception as e:
                self.get_logger().error(f"❌ Failed to initialize Extended UWA Tools: {e}")
                self.web_search_tools = WebSearchTools()  # Fallback to basic tools
                self.get_logger().info("📦 Fallback to basic Web Search Tools")
        else:
            self.web_search_tools = None
        
        # WebSocket and audio management
        self.websocket = None
        self.is_connected = True  # Start as True to begin connection attempts
        self.conversation_items = []
        self.response_active = False
        self.audio_playback_started = False
        self.drop_audio_frames = False
        self.current_response_id = None
        self.cancelled_responses = set()
        
        # Audio output configuration
        self.audio_output_enabled = os.environ.get("REALTIME_AUDIO_OUTPUT", "1") == "1"
        if self.audio_output_enabled:
            self.setup_audio_output()
        
        # Start background tasks
        self.event_loop = None
        self.websocket_thread = None
        self.start_realtime_connection()
        
        self.get_logger().info("🎙️ OpenAI Realtime node initialized")
        
    def setup_ros_communication(self):
        """Setup ROS2 publishers and subscribers"""
        # Subscribe to speech text from STT node
        self.speech_subscription = self.create_subscription(
            String,
            'speech_text',
            self.speech_text_callback,
            10
        )
        
        # Publish realtime responses (for compatibility with existing nodes)
        self.response_publisher = self.create_publisher(String, 'realtime_response', 10)
        self.full_response_publisher = self.create_publisher(String, 'realtime_response_full', 10)
        self.end_signal_publisher = self.create_publisher(Bool, 'realtime_response_end', 10)

        # Compatibility publishers for legacy LLM/TTS pipeline
        self.llm_response_publisher = self.create_publisher(String, 'llm_response', 10)
        self.llm_full_response_publisher = self.create_publisher(String, 'llm_response_full', 10)
        self.llm_end_signal_publisher = self.create_publisher(Bool, 'llm_response_end', 10)

        # Status publishing (realtime + TTS bridge)
        self.status_publisher = self.create_publisher(Bool, 'realtime_status', 10)
        self.tts_status_publisher = self.create_publisher(Bool, 'tts_status', 10)

        # Subscribe to interrupt signals
        self.interrupt_subscription = self.create_subscription(
            Bool,
            'realtime_interrupt',
            self.interrupt_callback,
            10
        )

        # Backwards compatibility: respond to historic tts_interrupt topic
        self.tts_interrupt_subscription = self.create_subscription(
            Bool,
            'tts_interrupt',
            self.interrupt_callback,
            10
        )
        
        self.get_logger().info("📡 ROS2 communication setup complete")

    def publish_stream_text(self, text: str):
        """Publish streaming text deltas to realtime and legacy topics"""
        if not text or self.drop_audio_frames:
            return
        msg = String()
        msg.data = text
        self.response_publisher.publish(msg)
        self.llm_response_publisher.publish(msg)

    def publish_full_text(self, text: str):
        """Publish completed text segments"""
        if not text or self.drop_audio_frames:
            return
        msg = String()
        msg.data = text
        self.full_response_publisher.publish(msg)
        self.llm_full_response_publisher.publish(msg)

    def publish_end_of_response(self):
        """Publish end-of-response signals to both pipelines"""
        end_msg = Bool()
        end_msg.data = True
        self.end_signal_publisher.publish(end_msg)
        self.llm_end_signal_publisher.publish(end_msg)

    def publish_status(self, is_active: bool):
        """Publish realtime/tts status flags"""
        status_msg = Bool()
        status_msg.data = is_active
        self.status_publisher.publish(status_msg)
        self.tts_status_publisher.publish(status_msg)

    def reset_audio_output(self):
        """Stop current audio stream to prevent residual playback"""
        if not self.audio_output_enabled:
            return
        try:
            if hasattr(self, 'output_stream') and self.output_stream:
                if self.output_stream.is_active():
                    self.output_stream.stop_stream()
                self.output_stream.close()
        except Exception as e:
            self.get_logger().warning(f'Audio stream reset warning: {e}')
        finally:
            self.output_stream = None
        try:
            if hasattr(self, 'audio_output') and self.audio_output:
                self.audio_output.terminate()
        except Exception as e:
            self.get_logger().warning(f'Audio device terminate warning: {e}')
        finally:
            self.audio_output = None
        try:
            self.setup_audio_output()
        except Exception as e:
            self.get_logger().error(f'❌ Failed to reinitialize audio output: {e}')

    def _extract_response_id(self, data: Dict) -> Optional[str]:
        response = data.get('response') if isinstance(data, dict) else None
        if isinstance(response, dict):
            rid = response.get('id') or response.get('response_id')
            if rid:
                return rid
        rid = data.get('response_id') if isinstance(data, dict) else None
        if rid:
            return rid
        item = data.get('item') if isinstance(data, dict) else None
        if isinstance(item, dict):
            rid = item.get('response_id')
            if rid:
                return rid
        return None

    def _is_response_cancelled(self, response_id: Optional[str]) -> bool:
        return bool(response_id) and response_id in self.cancelled_responses

    def signal_response_active(self):
        """Ensure downstream nodes know a response is in progress"""
        if not self.response_active:
            self.response_active = True
            self.audio_playback_started = False
            self.drop_audio_frames = False
            self.publish_status(True)

    def setup_audio_output(self):
        """Setup audio output for realtime TTS"""
        try:
            self.audio_output = pyaudio.PyAudio()
            self.output_stream = self.audio_output.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self.audio_sample_rate,
                output=True,
                frames_per_buffer=self.audio_chunk_size
            )
            self.get_logger().info("🔊 Audio output initialized")
        except Exception as e:
            self.get_logger().error(f"❌ Audio output initialization failed: {e}")
            self.audio_output_enabled = False
    
    def start_realtime_connection(self):
        """Start WebSocket connection in background thread"""
        def run_event_loop():
            try:
                self.event_loop = asyncio.new_event_loop()
                asyncio.set_event_loop(self.event_loop)
                self.event_loop.run_until_complete(self.maintain_connection())
            except Exception as e:
                self.get_logger().error(f"❌ Event loop error: {e}")
            finally:
                if self.event_loop:
                    self.event_loop.close()
        
        self.websocket_thread = threading.Thread(target=run_event_loop, daemon=True)
        self.websocket_thread.start()
    
    async def maintain_connection(self):
        """Maintain WebSocket connection with automatic reconnection"""
        while rclpy.ok() and self.is_connected:
            try:
                await self.connect_to_realtime_api()
            except Exception as e:
                self.get_logger().error(f"❌ Realtime connection failed: {e}")
                if self.is_connected:  # Only retry if still supposed to be connected
                    await asyncio.sleep(5)  # Wait before reconnecting
                else:
                    break  # Exit loop if disconnection was intentional
    
    async def connect_to_realtime_api(self):
        """Connect to OpenAI Realtime API via WebSocket"""
        url = "wss://api.openai.com/v1/realtime?model=" + self.realtime_model
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "OpenAI-Beta": "realtime=v1"
        }

        session: aiohttp.ClientSession | None = None
        try:
            session = aiohttp.ClientSession()
            async with session.ws_connect(
                url,
                headers=headers,
                protocols=("openai-realtime-v1",),
                heartbeat=20.0,
                max_msg_size=5 * 512 * 512, 
            ) as websocket:
                self.websocket = websocket
                self.get_logger().info("✅ Connected to OpenAI Realtime API")

                # Send initial session configuration
                await self.configure_session()

                # Handle incoming messages
                async for msg in websocket:
                    if msg.type == WSMsgType.TEXT:
                        await self.handle_websocket_message(msg.data)
                    elif msg.type == WSMsgType.BINARY:
                        await self.handle_binary_message(msg.data)
                    elif msg.type == WSMsgType.ERROR:
                        raise msg.data
                    elif msg.type in (WSMsgType.CLOSE, WSMsgType.CLOSING, WSMsgType.CLOSED):
                        break
        except aiohttp.WSServerHandshakeError as e:
            self.get_logger().error(f"❌ Realtime handshake failed: {e}")
        except aiohttp.ClientError as e:
            self.get_logger().error(f"❌ Realtime client error: {e}")
        except Exception as e:
            self.get_logger().error(f"❌ WebSocket error: {e}")
        finally:
            self.websocket = None
            if session is not None:
                await session.close()
    
    async def configure_session(self):
        """Configure the Realtime API session"""
        session_config = {
            "type": "session.update",
            "session": {
                "modalities": ["text", "audio"],
                "instructions": self.get_system_instructions(),
                "voice": self.realtime_voice,
                "input_audio_format": "pcm16",
                "output_audio_format": "pcm16",
                "input_audio_transcription": {
                    "model": "whisper-1"
                },
                "turn_detection": {
                    "type": "server_vad",
                    "threshold": 0.5,
                    "prefix_padding_ms": 300,
                    "silence_duration_ms": 200,
                    "interrupt_response": True,
                    "create_response": True
                },
                "tools": self.get_function_calling_tools() if self.enable_function_calling else []
            }
        }
        
        await self.send_websocket_message(session_config)
        self.get_logger().info("⚙️ Session configured")
    
    def get_system_instructions(self):
        """Get system instructions for the Realtime API"""
        # Get real-time information
        now = datetime.now()
        current_time = now.strftime("%I:%M %p")
        current_day = now.strftime("%A") 
        current_date = now.strftime("%B %d, %Y")
        current_full = now.strftime("%I:%M %p on %A, %B %d, %Y")
        
        return f"""You are "New Way 4", an intelligent and helpful campus shuttle assistant for the University of Western Australia (UWA). 

CURRENT TIME AND DATE: {current_full}
Current time: {current_time}
Current day: {current_day}
Current date: {current_date}
Location: Perth, Western Australia
Timezone: Australian Western Standard Time (AWST, UTC+8)

WAKE WORD INFORMATION:
Users activate you by saying "new way 4" (or variations like "new way four", "neway 4", "neway four", etc.). 
When responding, you can refer to yourself as "New Way 4" or just respond naturally without repeating the wake word.

Your personality:
- Friendly, knowledgeable, and energetic
- Speak like a helpful campus guide who knows UWA inside and out
- Use emojis occasionally to be engaging
- Be concise but informative
- Always prioritize user safety and campus policies

Your capabilities:
- Provide real-time information about UWA campus
- Search for current weather, transport, events, and campus services
- Help with building locations, facility hours, dining options, and parking
- Offer navigation assistance and campus tips
- Access live information through function calling

Key areas of expertise:
🏫 Campus locations and navigation
⏰ Facility hours and availability  
�️ Campus dining and food options
🅿️ Parking information and rates
🚌 Transport and shuttle information
🌤️ Current weather for campus planning
📚 Campus services (library, printing, ATMs, etc.)

Always use your function calling tools to provide current, accurate information rather than making assumptions. When users ask about campus information, actively search for the most up-to-date details.

Remember: You're here to make campus life easier and more enjoyable for everyone at UWA!"""
    
    def get_function_calling_tools(self):
        """Get available function calling tools for Realtime API"""
        if not self.web_search_tools:
            return []
        
        tools = []
        
        # General web search
        tools.append({
            "type": "function",
            "name": "search_web",
            "description": "Search the web for general information",
            "parameters": {
                "type": "object",
                "properties": {
                    "query": {
                        "type": "string",
                        "description": "Search query"
                    },
                    "max_results": {
                        "type": "integer",
                        "description": "Maximum number of results to return",
                        "default": 3
                    }
                },
                "required": ["query"]
            }
        })
        
        # UWA transport search
        tools.append({
            "type": "function",
            "name": "search_uwa_transport",
            "description": "Search for UWA transportation and bus route information",
            "parameters": {
                "type": "object",
                "properties": {
                    "query": {
                        "type": "string",
                        "description": "Transport query (default: 'UWA bus transport perth')",
                        "default": "UWA bus transport perth"
                    }
                }
            }
        })
        
        # Weather search
        tools.append({
            "type": "function",
            "name": "search_current_weather",
            "description": "Get current weather information",
            "parameters": {
                "type": "object",
                "properties": {
                    "location": {
                        "type": "string",
                        "description": "Location for weather query",
                        "default": "Perth UWA"
                    }
                }
            }
        })
        
        # UWA events search
        tools.append({
            "type": "function",
            "name": "search_uwa_events",
            "description": "Search for current UWA events and activities",
            "parameters": {
                "type": "object",
                "properties": {
                    "query": {
                        "type": "string",
                        "description": "Events query",
                        "default": "UWA events today"
                    }
                }
            }
        })
        
        # Extended UWA Tools - Campus Locations
        tools.append({
            "type": "function",
            "name": "search_uwa_locations",
            "description": "Search for UWA campus building and facility locations with directions",
            "parameters": {
                "type": "object",
                "properties": {
                    "query": {
                        "type": "string",
                        "description": "Location query (building name, facility type, etc.)",
                        "default": "building locations"
                    }
                },
                "required": ["query"]
            }
        })
        
        # Extended UWA Tools - Facility Hours
        tools.append({
            "type": "function",
            "name": "get_uwa_hours",
            "description": "Get UWA facility opening hours and current status",
            "parameters": {
                "type": "object",
                "properties": {
                    "facility": {
                        "type": "string",
                        "description": "Facility name (library, dining, sports centre, student services, etc.)"
                    }
                },
                "required": ["facility"]
            }
        })
        
        # Extended UWA Tools - Campus Dining
        tools.append({
            "type": "function",
            "name": "search_campus_dining",
            "description": "Search for campus dining options, food preferences, and meal information",
            "parameters": {
                "type": "object",
                "properties": {
                    "dining_type": {
                        "type": "string",
                        "description": "Dining type (cafe, restaurant, food court, etc.)",
                        "default": "all"
                    },
                    "preferences": {
                        "type": "string",
                        "description": "Dietary preferences (vegetarian, halal, asian, budget, etc.)",
                        "default": ""
                    }
                }
            }
        })
        
        # Extended UWA Tools - Parking Information
        tools.append({
            "type": "function",
            "name": "check_parking_availability",
            "description": "Check UWA parking availability, rates, and parking recommendations",
            "parameters": {
                "type": "object",
                "properties": {
                    "area": {
                        "type": "string",
                        "description": "Parking area query (student parking, visitor parking, etc.)",
                        "default": "UWA campus"
                    }
                }
            }
        })
        
        # Extended UWA Tools - Nearby Services
        tools.append({
            "type": "function",
            "name": "find_nearby_services",
            "description": "Find services near UWA campus (ATM, printing, medical, food, etc.)",
            "parameters": {
                "type": "object",
                "properties": {
                    "location": {
                        "type": "string",
                        "description": "Campus location reference point",
                        "default": "central campus"
                    },
                    "service_type": {
                        "type": "string",
                        "description": "Service type (bank, medical, pharmacy, food, ATM, printing, etc.)",
                        "default": "general"
                    }
                },
                "required": ["location", "service_type"]
            }
        })
        
        return tools
    
    async def handle_websocket_message(self, message):
        """Handle incoming WebSocket messages from Realtime API"""
        try:
            data = json.loads(message)
            event_type = data.get("type", "")
            
            # Log all events for debugging
            self.get_logger().info(f"📨 Received event: {event_type}")
            
            if event_type == "session.created":
                self.get_logger().info("✅ Realtime session created")
                
            elif event_type == "session.updated":
                self.get_logger().info("⚙️ Session updated")
                
            elif event_type == "conversation.item.created":
                self.handle_conversation_item_created(data)
                
            elif event_type == "response.created":
                self.handle_response_created(data)
                
            elif event_type == "response.output_item.added":
                self.handle_output_item_added(data)
                
            elif event_type == "response.content_part.added":
                self.handle_content_part_added(data)
                
            elif event_type == "response.audio.delta":
                await self.handle_audio_delta(data)
                
            elif event_type == "response.text.delta":
                self.handle_text_delta(data)
                
            elif event_type == "response.function_call_arguments.delta":
                self.handle_function_call_delta(data)
                
            elif event_type == "response.function_call_arguments.done":
                await self.handle_function_call_done(data)
                
            elif event_type == "response.output_item.done":
                await self.handle_output_item_done(data)
                
            elif event_type == "conversation.item.completed":
                await self.handle_item_completed(data)
                
            elif event_type == "response.done":
                self.handle_response_done(data)
                
            elif event_type == "error":
                self.get_logger().error(f"❌ Realtime API error: {data.get('error', {})}")
                
            else:
                self.get_logger().info(f"📨 Unhandled event: {event_type} - {data}")
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ Failed to parse WebSocket message: {e}")
        except Exception as e:
            self.get_logger().error(f"❌ Error handling WebSocket message: {e}")
    
    def handle_conversation_item_created(self, data):
        """Handle conversation item creation"""
        item = data.get("item", {})
        self.conversation_items.append(item)
        self.get_logger().info(f"📝 Item created: {item.get('type', 'unknown')} - {item.get('id', 'no-id')}")
    
    def handle_response_created(self, data):
        """Handle response creation"""
        response = data.get("response", {})
        response_id = self._extract_response_id(data)
        if response_id:
            self.current_response_id = response_id
            self.cancelled_responses.discard(response_id)
        self.drop_audio_frames = False
        self.signal_response_active()
        self.get_logger().info(f"📣 Response created: {response.get('id', 'no-id')}")
    
    def handle_output_item_added(self, data):
        """Handle output item added"""
        item = data.get("item", {})
        self.get_logger().info(f"📤 Output item added: {item.get('type', 'unknown')}")
    
    def handle_content_part_added(self, data):
        """Handle content part added"""
        response_id = self._extract_response_id(data)
        if self._is_response_cancelled(response_id):
            self.get_logger().debug('Skipping content part for cancelled response')
            return
        part = data.get("part", {})
        content_type = part.get("type", "unknown")
        self.get_logger().info(f"📝 Content part added: {content_type}")

        # If it's text content, publish it immediately
        if content_type == "text":
            text = part.get("text", "")
            if text:
                self.signal_response_active()
                self.get_logger().info(f"✅ Text content part: {text}")

    

    async def handle_binary_message(self, data: bytes):
        """Fallback handler for binary websocket frames."""
        if not data:
            return
        try:
            text_data = data.decode('utf-8')
        except Exception:
            self.get_logger().warning("⚠️ Received unsupported binary frame; dropping")
            return
        await self.handle_websocket_message(text_data)

    async def handle_audio_delta(self, data):
        """Handle incoming audio delta from Realtime API"""
        response_id = self._extract_response_id(data)
        if self._is_response_cancelled(response_id):
            return
        delta = data.get("delta", "")
        if not delta or self.drop_audio_frames:
            return

        self.signal_response_active()
        if not self.audio_playback_started:
            self.audio_playback_started = True

        if not self.audio_output_enabled:
            return

        try:
            # Decode base64 audio data
            import base64
            audio_bytes = base64.b64decode(delta)

            # Play audio directly (ensure bytes object)
            if hasattr(self, 'output_stream') and self.output_stream:
                # Convert to bytes if needed and ensure proper format
                if isinstance(audio_bytes, str):
                    audio_bytes = audio_bytes.encode('latin-1')
                self.output_stream.write(bytes(audio_bytes))

        except Exception as e:
            self.get_logger().error(f"❌ Audio playback error: {e}")
            # Continue without audio playback if error occurs
            pass
    
    def handle_text_delta(self, data):
        """Handle text delta from Realtime API"""
        response_id = self._extract_response_id(data)
        if self._is_response_cancelled(response_id):
            return
        delta = data.get("delta", "")
        if delta:
            self.signal_response_active()
            self.publish_stream_text(delta)
            self.get_logger().debug(f"📝 Text delta: {delta}")
    
    def handle_function_call_delta(self, data):
        """Handle function call arguments delta"""
        delta = data.get("delta", "")
        if delta:
            self.get_logger().debug(f"🔧 Function call delta: {delta}")
    
    async def handle_function_call_done(self, data):
        """Handle function call arguments completion"""
        function_name = data.get("name", "")
        arguments = data.get("arguments", "{}")
        call_id = data.get("call_id", "")
        
        self.get_logger().info(f"🔧 Function call completed: {function_name} with args: {arguments}")
        
        try:
            # Parse arguments
            import json
            args = json.loads(arguments)
            
            # Execute the function
            result = None
            if function_name == "search_current_weather":
                location = args.get("location", "Perth UWA")
                result = self.web_search_tools.search_current_weather(location)
            elif function_name == "search_web":
                query = args.get("query", "")
                max_results = args.get("max_results", 3)
                result = self.web_search_tools.search_web(query, max_results)
            elif function_name == "search_uwa_transport":
                query = args.get("query", "UWA bus transport perth")
                result = self.web_search_tools.search_uwa_transport(query)
            elif function_name == "search_uwa_events":
                query = args.get("query", "UWA events today")
                result = self.web_search_tools.search_uwa_events(query)
            # Extended UWA Tools
            elif function_name == "search_uwa_locations":
                query = args.get("query", "building locations")
                result = self.web_search_tools.search_uwa_locations(query)
            elif function_name == "get_uwa_hours":
                facility = args.get("facility", "library")
                result = self.web_search_tools.get_uwa_hours(facility)
            elif function_name == "search_campus_dining":
                dining_type = args.get("dining_type", "all")
                preferences = args.get("preferences", "")
                result = self.web_search_tools.search_campus_dining(dining_type, preferences)
            elif function_name == "check_parking_availability":
                area = args.get("area", "UWA campus")
                result = self.web_search_tools.check_parking_availability(area)
            elif function_name == "find_nearby_services":
                location = args.get("location", "central campus")
                service_type = args.get("service_type", "general")
                result = self.web_search_tools.find_nearby_services(location, service_type)
            
            # Send function result back to API
            if result:
                await self.send_function_result(call_id, result)
                
        except Exception as e:
            self.get_logger().error(f"❌ Function execution error: {e}")
            # Send error result
            error_result = {"error": str(e)}
            await self.send_function_result(call_id, error_result)
    
    async def handle_output_item_done(self, data):
        """Handle output item completion"""
        response_id = self._extract_response_id(data)
        if self._is_response_cancelled(response_id):
            return
        item = data.get("item", {})
        item_type = item.get("type", "")

        if item_type == "function_call":
            # Function call completed, already handled by function_call_done
            self.get_logger().info(f"✅ Function call output completed: {item.get('name', 'unknown')}")
        elif item_type == "message":
            # Message output completed
            self.handle_completed_message(item)
    
    async def handle_item_completed(self, data):
        """Handle completed conversation item"""
        response_id = self._extract_response_id(data)
        if self._is_response_cancelled(response_id):
            return
        item = data.get("item", {})
        item_type = item.get("type", "")

        if item_type == "function_call":
            await self.execute_function_call(item)
        elif item_type == "message":
            self.handle_completed_message(item)
    
    async def execute_function_call(self, item):
        """Execute function call and send result back"""
        if not self.web_search_tools:
            return
            
        try:
            function_name = item.get("name", "")
            arguments = json.loads(item.get("arguments", "{}"))
            call_id = item.get("call_id", "")
            
            self.get_logger().info(f"🔧 Executing function: {function_name}")
            
            # Execute the function
            result = None
            if function_name == "search_web":
                query = arguments.get("query", "")
                max_results = arguments.get("max_results", 3)
                result = self.web_search_tools.search_web(query, max_results)
            elif function_name == "search_uwa_transport":
                query = arguments.get("query", "UWA bus transport perth")
                result = self.web_search_tools.search_uwa_transport(query)
            elif function_name == "search_current_weather":
                location = arguments.get("location", "Perth UWA")
                result = self.web_search_tools.search_current_weather(location)
            elif function_name == "search_uwa_events":
                query = arguments.get("query", "UWA events today")
                result = self.web_search_tools.search_uwa_events(query)
            
            # Send function result back to Realtime API
            if result:
                await self.send_function_result(call_id, result)
                
        except Exception as e:
            self.get_logger().error(f"❌ Function execution error: {e}")
            # Send error result
            error_result = {"error": str(e)}
            await self.send_function_result(call_id, error_result)
    
    async def send_function_result(self, call_id: str, result: Dict):
        """Send function call result back to Realtime API"""
        function_output = {
            "type": "conversation.item.create",
            "item": {
                "type": "function_call_output",
                "call_id": call_id,
                "output": json.dumps(result)
            }
        }
        
        await self.send_websocket_message(function_output)
        
        # Trigger response generation
        response_create = {"type": "response.create"}
        await self.send_websocket_message(response_create)
        
        self.get_logger().info(f"✅ Function result sent for call_id: {call_id}")
    
    def handle_completed_message(self, item):
        """Handle completed message"""
        content = item.get("content", [])
        published_text = False
        if content and isinstance(content, list):
            for part in content:
                if part.get("type") == "text":
                    text = part.get("text", "")
                    if text:
                        # Publish full response
                        self.signal_response_active()
                        self.publish_full_text(text)
                        published_text = True
                        self.get_logger().info(f"✅ Complete response: {text}")
        if published_text:
            self.publish_end_of_response()
    
    def handle_response_done(self, data):
        """Handle response completion"""
        response_id = self._extract_response_id(data)
        was_cancelled = self._is_response_cancelled(response_id)
        if response_id:
            self.cancelled_responses.discard(response_id)
            if response_id == self.current_response_id:
                self.current_response_id = None
        self.get_logger().debug("✅ Response completed")
        had_active = self.response_active
        self.response_active = False
        self.audio_playback_started = False
        self.drop_audio_frames = True
        self.publish_status(False)
        if had_active:
            self.publish_end_of_response()
        if was_cancelled:
            self.reset_audio_output()
    
    async def send_websocket_message(self, message: Dict):
        """Send message to WebSocket"""
        if self.websocket and not self.websocket.closed and self.is_connected:
            try:
                await self.websocket.send_str(json.dumps(message))
            except Exception as e:
                self.get_logger().error(f"❌ Failed to send WebSocket message: {e}")
        else:
            self.get_logger().warning("⚠️ Cannot send message: websocket not connected")
    
    def speech_text_callback(self, msg: String):
        """Handle incoming speech text from STT node"""
        text = msg.data.strip()
        if not text:
            return
            
        self.get_logger().info(f"🎤 Received speech: {text}")
        
        # Send text to Realtime API
        if self.event_loop and self.is_connected:
            asyncio.run_coroutine_threadsafe(
                self.send_user_message(text),
                self.event_loop
            )
    
    async def send_user_message(self, text: str):
        """Send user message to Realtime API"""
        user_message = {
            "type": "conversation.item.create",
            "item": {
                "type": "message",
                "role": "user",
                "content": [
                    {
                        "type": "input_text",
                        "text": text
                    }
                ]
            }
        }
        
        await self.send_websocket_message(user_message)
        
        # Trigger response generation
        response_create = {"type": "response.create"}
        await self.send_websocket_message(response_create)
        
        self.get_logger().info(f"📤 Sent user message: {text}")
    
    def interrupt_callback(self, msg: Bool):
        """Handle interrupt signal"""
        if msg.data and self.event_loop and self.is_connected:
            asyncio.run_coroutine_threadsafe(
                self.interrupt_response(),
                self.event_loop
            )
    
    async def interrupt_response(self):
        """Interrupt current response"""
        interrupt_message = {
            "type": "response.cancel"
        }
        await self.send_websocket_message(interrupt_message)
        had_active = self.response_active
        if self.current_response_id:
            self.cancelled_responses.add(self.current_response_id)
        if had_active:
            self.get_logger().info("⛔ Response interrupted")
        self.response_active = False
        self.audio_playback_started = False
        self.drop_audio_frames = True
        self.publish_status(False)
        if had_active:
            self.publish_end_of_response()
        self.reset_audio_output()
        self.current_response_id = None
    
    def destroy_node(self):
        """Clean up resources"""
        try:
            # Stop connection
            self.is_connected = False
            
            if self.audio_output_enabled and hasattr(self, 'output_stream'):
                self.output_stream.stop_stream()
                self.output_stream.close()
                self.audio_output.terminate()
                
            if self.websocket and not getattr(self.websocket, 'closed', True) and self.event_loop:
                try:
                    future = asyncio.run_coroutine_threadsafe(self.websocket.close(), self.event_loop)
                    future.result(timeout=2)
                except Exception:
                    pass

            if self.event_loop:
                try:
                    # Cancel all running tasks
                    for task in asyncio.all_tasks(self.event_loop):
                        task.cancel()
                    # Stop the event loop gracefully
                    self.event_loop.call_soon_threadsafe(self.event_loop.stop)
                except Exception as loop_e:
                    self.get_logger().error(f"❌ Event loop cleanup error: {loop_e}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Cleanup error: {e}")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OpenAIRealtimeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()