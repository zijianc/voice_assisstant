#!/usr/bin/env python3
import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import vosk

from ament_index_python.packages import get_package_share_directory


# 显式设置 PULSE_SERVER 环境变量
os.environ['PULSE_SERVER'] = 'docker.for.mac.localhost:4713'

class VoskSTTNode(Node):
    def __init__(self):
        super().__init__('vosk_stt_node')
        # 创建发布者，发布识别结果到话题 'speech_text'
        self.publisher_ = self.create_publisher(String, 'speech_text', 10)

        # 获取当前文件所在目录，设置英语模型路径（相对于该文件所在位置）
        this_dir = os.path.dirname(os.path.realpath(__file__))
        default_model_path = os.path.join(this_dir, "..", "models", "vosk-model-en-us-0.22")
        # 优先使用环境变量指定的路径
        model_path_en = os.environ.get("VOSK_MODEL_EN_PATH",
                                       "/workspaces/ros2_ws/src/my_voice_assistant/models/vosk-model-en-us-0.22")
        # ---- 可选禁用 RNNLM 以避免 word_feats.txt 报错 ----
        disable_rnnlm = os.environ.get("DISABLE_RNNLM", "1") == "1"  # 缺省启用禁用逻辑，可通过环境变量关闭
        if disable_rnnlm:
            rnn_dir = os.path.join(model_path_en, "rnnlm")
            if os.path.isdir(rnn_dir):
                try:
                    self.get_logger().warn("Disabling RNNLM (renaming rnnlm -> rnnlm.disabled) to avoid word_feats.txt error")
                    os.rename(rnn_dir, rnn_dir + ".disabled")
                except Exception as e:
                    self.get_logger().error(f"Failed to rename rnnlm directory: {e}")
        self.get_logger().info("加载英语模型：{}".format(model_path_en))
        try:
            self.model_en = vosk.Model(model_path_en)
        except Exception as e:
            self.get_logger().fatal(f"Vosk model load failed: {e}")
            raise RuntimeError("Unable to load Vosk model")

        # 加载英语模型
        self.recognizer_en = vosk.KaldiRecognizer(self.model_en, 24000)
        self.recognizer_en.SetWords(True)
        self.recognizer_en.SetMaxAlternatives(5)

        # 初始化音频输入
        self.audio = pyaudio.PyAudio()
        self.stream = None # Initialize stream attribute
        target_rate = 24000  # Vosk model's sample rate
        frames_per_buffer = 2048 # Number of frames per buffer

        # 打印可用的音频设备信息以供调试
        self.get_logger().info("Available PyAudio audio devices:")
        try:
            for i in range(self.audio.get_device_count()):
                info = self.audio.get_device_info_by_index(i)
                self.get_logger().info(
                    f"  Device {i}: {info['name']}, "
                    f"Input Channels: {info['maxInputChannels']}, "
                    f"Default Sample Rate: {info['defaultSampleRate']}"
                )
        except Exception as e:
            self.get_logger().error(f"Error enumerating PyAudio devices: {e}")

        # 列出所有设备并记录首选索引（可由环境变量指定）
        preferred_str = os.getenv("PREFERRED_INPUT_INDEX")
        preferred = int(preferred_str) if preferred_str is not None and preferred_str.isdigit() else -1 # Use -1 if not set or invalid
        device_index = None

         # 尝试查找首选设备或第一个可用的输入设备
        try:
            for i in range(self.audio.get_device_count()):
                info = self.audio.get_device_info_by_index(i)
                if info.get('maxInputChannels', 0) > 0:
                    if preferred == i:
                        device_index = i
                        self.get_logger().info(f"Using preferred audio device {device_index}: {info['name']}")
                        break
                    if device_index is None: # Fallback to first available input device
                        device_index = i
            
            if device_index is not None and preferred != -1 and preferred != device_index :
                 self.get_logger().info(f"Preferred device {preferred} not found or not suitable. Using first available input device {device_index}: {self.audio.get_device_info_by_index(device_index)['name']}")
            elif device_index is None and self.audio.get_device_count() > 0:
                self.get_logger().warning("No input channels found on any enumerated PyAudio device. Will try default.")
            elif self.audio.get_device_count() == 0:
                self.get_logger().warning("PyAudio found no audio devices at all. Will try default.")
        except Exception as e:
            self.get_logger().error(f"Error during PyAudio device selection: {e}. Will try default.")
            device_index = None

        if device_index is not None:
            self.get_logger().info(f"Attempting to open specific device {device_index} ({self.audio.get_device_info_by_index(device_index)['name']}) at {target_rate}Hz.")
            try:
                self.stream = self.audio.open(
                    format=pyaudio.paInt16,
                    channels=1,
                    rate=target_rate,
                    input=True,
                    input_device_index=device_index,
                    frames_per_buffer=frames_per_buffer
                )
                self.get_logger().info(f"Successfully opened audio stream on device {device_index}.")
            except Exception as e:
                self.get_logger().error(f"Failed to open audio stream on device {device_index} at {target_rate}Hz: {e}")
                self.get_logger().info("Falling back to default device attempt.")
                self.stream = None # Ensure stream is None if opening failed

        if self.stream is None: # Either device_index was None, or opening specific device failed
            self.get_logger().info(f"Attempting to open default audio device at {target_rate}Hz.")
            try:
                self.stream = self.audio.open(
                    format=pyaudio.paInt16,
                    channels=1,
                    rate=target_rate,
                    input=True,
                    frames_per_buffer=frames_per_buffer
                )
                self.get_logger().info(f"Successfully opened default audio stream at {target_rate}Hz.")
            except Exception as e:
                self.get_logger().error(f"Failed to open default audio stream at {target_rate}Hz: {e}")
                try:
                    default_info = self.audio.get_default_input_device_info()
                    self.get_logger().error(
                        f"Default PyAudio input device info: Index: {default_info['index']}, "
                        f"Name: {default_info['name']}, MaxInputChannels: {default_info['maxInputChannels']}, "
                        f"DefaultSampleRate: {default_info['defaultSampleRate']}"
                    )
                except Exception as e_info:
                    self.get_logger().error(f"Could not get default PyAudio input device info: {e_info}")
                
                self.get_logger().error("Audio stream could not be opened. Node will not function.")
                # Clean up PyAudio instance if stream opening failed critically before returning
                self.audio.terminate()
                return

        self.stream.start_stream()
        self.get_logger().info(f"Microphone stream started (device: {'default' if device_index is None else device_index}, rate: {target_rate}Hz, buffer: {frames_per_buffer} frames).")

        # 设置唤醒词（转换为小写用于匹配）
        self.wake_word = "hey captain"

        # 创建定时器，每0.1秒读取音频数据
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):  # ✅ 现在这是类的方法，不是嵌套函数
        if not self.stream or not self.stream.is_active(): # Check if stream is valid and active
            return
        try:
            # Read frames_per_buffer (e.g., 2048 frames)
            data = self.stream.read(2048, exception_on_overflow=False) # Use the same frames_per_buffer value as in open()
            
            # 添加调试信息：检查音频数据
            if len(data) > 0:
                # 计算音频数据的RMS（音量大小）
                import struct
                import math
                audio_data = struct.unpack('<' + ('h' * (len(data) // 2)), data)
                rms = math.sqrt(sum(x*x for x in audio_data) / len(audio_data))
                if rms > 100:  # 只在有足够音量时打印调试信息
                    self.get_logger().info(f"音频数据: 长度={len(data)}, RMS音量={rms:.1f}")
            
        except IOError as e:
            if hasattr(e, 'errno') and e.errno == pyaudio.paInputOverflowed:
                self.get_logger().warn("Input overflowed. Some audio data may have been lost.")
            # Add more specific IOError checks if needed e.g pa አንዳንድ땦 paUnanticipatedHostError
            elif "Stream closed" in str(e) or (hasattr(e, 'errno') and e.errno == pyaudio.paStreamIsStopped): # Check if stream is closed
                 self.get_logger().error(f"Audio stream closed or stopped unexpectedly: {e}")
                 # Attempt to stop and close the stream gracefully, then terminate PyAudio
                 # You might want to add logic to attempt a restart of the stream or node
                 if self.stream:
                     try:
                         if self.stream.is_active():
                             self.stream.stop_stream()
                         self.stream.close()
                     except Exception as ex_close:
                         self.get_logger().error(f"Exception during stream cleanup: {ex_close}")
                     finally:
                         self.stream = None
                 # self.audio.terminate() # Terminate PyAudio if error is persistent
                 # rclpy.shutdown() # Or shutdown the ROS node
                 return
            else:
                self.get_logger().error(f"IOError reading audio stream: {e}")
            return
        except Exception as e: # Catch any other unexpected error
            self.get_logger().error(f"Generic error reading audio stream: {e}")
            return
        
        # 处理音频数据进行语音识别
        if self.recognizer_en.AcceptWaveform(data):
            result = json.loads(self.recognizer_en.Result())
            if result.get('text'):
                text = result['text'].strip()
                self.get_logger().info(f"🎤 完整识别: {text}")
                
                # 检查是否包含唤醒词
                if self.wake_word.lower() in text.lower():
                    self.get_logger().info(f"🔥 检测到唤醒词: {self.wake_word}")
                    # 发布识别结果
                    msg = String()
                    msg.data = text
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"📢 已发布到 speech_text: {text}")
        else:
            # 获取部分识别结果（实时转录）
            partial_result = json.loads(self.recognizer_en.PartialResult())
            if partial_result.get('partial'):
                partial_text = partial_result['partial'].strip()
                if partial_text:
                    self.get_logger().info(f"🔄 实时识别: {partial_text}")

    def destroy_node(self):
        self.get_logger().info("Shutting down VoskSTTNode.")
        if self.timer:
            self.timer.cancel()
        if hasattr(self, 'stream') and self.stream:
            try:
                if self.stream.is_active():
                    self.stream.stop_stream()
                self.stream.close()
                self.get_logger().info("Audio stream closed.")
            except Exception as e:
                self.get_logger().error(f"Error closing audio stream: {e}")
            finally:
                self.stream = None
        if hasattr(self, 'audio') and self.audio:
            try:
                self.audio.terminate()
                self.get_logger().info("PyAudio terminated.")
            except Exception as e:
                self.get_logger().error(f"Error terminating PyAudio: {e}")
            finally:
                self.audio = None
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VoskSTTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()