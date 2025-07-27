
# Voice Assistant in Docker Environment (ROS 2)

This guide outlines the ## 🔧 Environment Variables

For OpenAI-based nodes, you need to set the following environment variables:

```bash
# Required for openai_stt_node and openai_tts_node
export OPENAI_API_KEY="your-openai-api-key"

# Optional: specify the STT model (default: whisper-1)
export OPENAI_STT_MODEL="whisper-1"  # or "gpt-4o-mini-transcribe"

# Optional: enable audio file saving mode (useful in Docker)
export TTS_SAVE_MODE=true  # saves audio files to audio_output/ directory
```

## 🔊 Audio Output Solutions

If you can't hear audio in Docker (common with headphones):

```bash
# Option 1: Use file saving mode
./start_openai_tts.sh --save-files
# Audio files will be saved to /workspaces/ros2_ws/audio_output/
# Play these files on your host machine

# Option 2: Test audio file generation
./test_audio_save.sh
# Then check the audio_output/ directory for generated MP3 files
```teps to build and run a voice assistant inside a Docker container using ROS 2.

## 🎤 语音助手系统演示

```bash
# 完整系统演示
./demo_voice_assistant.sh

# 这个脚本将演示:
# 1. 唤醒词检测 ('Hi Captain', 'Hey Captain', 'Hello Captain')
# 2. LLM 流式响应处理
# 3. OpenAI TTS 语音生成和播放
# 4. TTS 状态控制 (暂停/恢复监听)
```

---

## 🛠 Environment Setup (After Each Rebuild)

After every container rebuild, run the following commands:

```bash
# Activate Python virtual environment
source venv/bin/activate

# Clean previous build artifacts
rm -rf build install log

# Build the workspace
colcon build

# Source the environment setup
source install/setup.bash
```

---

## 🚀 Run ROS 2 Nodes

```bash
# Run Vosk-based speech-to-text node
ros2 run my_voice_assistant vosk_stt_node

# Run OpenAI-based speech-to-text node (requires OPENAI_API_KEY)
# Use the convenience script that sets up the environment:
./start_openai_stt.sh

# Or run manually with proper environment setup:
# source venv/bin/activate && source install/setup.bash && \
# export PYTHONPATH=/workspaces/ros2_ws/venv/lib/python3.10/site-packages:$PYTHONPATH && \
# ros2 run my_voice_assistant openai_stt_node

# Run LLM-based dialogue processing node
ros2 run my_voice_assistant llm_node

# Run real-time interaction node (with audio generation)
ros2 run my_voice_assistant realtime_llm_node

# Run reinforcement learning STT node
ros2 run my_voice_assistant rl_stt_node

# Run text-to-speech (TTS) node
./start_tts.sh

# Run OpenAI TTS node (requires OPENAI_API_KEY)
./start_openai_tts.sh

```

---

## � Environment Variables

For OpenAI-based nodes, you need to set the following environment variables:

```bash
# Required for openai_stt_node and openai_tts_node
export OPENAI_API_KEY="your-openai-api-key"

# Optional: specify the STT model (default: whisper-1)
export OPENAI_STT_MODEL="whisper-1"  # or "gpt-4o-mini-transcribe"
```

---

## �🔍 Check Installed Nodes

```bash
ros2 pkg executables my_voice_assistant
```

---

## 🧪 Test Voice Message Publishing

```bash
ros2 topic pub --once /speech_text std_msgs/msg/String "data: 'Hi,captain, who are you'"
ros2 topic pub --once /speech_text std_msgs/msg/String "data: 'Hello'"
```

---

## 🎤 Check Audio Input Devices

```bash
pactl list sources short
```

---

## 🐞 Common Debug Tips

### ❗ ModuleNotFoundError: No module named 'dotenv'

```bash
echo 'export PYTHONPATH=/workspaces/ros2_ws/venv/lib/python3.10/site-packages:$PYTHONPATH' >> ~/.bashrc
source ~/.bashrc
```

### ❗ Missing eSpeak or eSpeak-ng

```bash
apt update && apt install -y espeak
```

### ❗ Vosk model fails with "No line could be read from the file"

If you see this error and `word_feats.txt` is empty, remove `rnnlm/final.raw` or the entire `rnnlm/` folder to disable RNNLM support:

```bash
rm -rf src/my_voice_assistant/models/vosk-model-en-us-0.22/rnnlm
```

---

## ✅ Quick Rebuild Script

Create a file named `rebuild.sh` with the following content to automate cleanup and rebuild:

```bash
#!/bin/bash
source venv/bin/activate
rm -rf build install log
colcon build
source install/setup.bash
echo "✅ Rebuild complete."
```

Then run it with:

```bash
chmod +x rebuild.sh  # Grant permission
source rebuild.sh         # Execute rebuild anytime
```
