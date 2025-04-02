# voice_assisstant
# Voice Assistant in Docker Environment (ROS 2)

This guide outlines the necessary steps to build and run a voice assistant inside a Docker container using ROS 2.

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

# Run LLM-based dialogue processing node
ros2 run my_voice_assistant llm_node

# Run real-time interaction node (with audio generation)
ros2 run my_voice_assistant realtime_llm_node

# Run reinforcement learning STT node
ros2 run my_voice_assistant rl_stt_node

# Run text-to-speech (TTS) node
ros2 run my_voice_assistant tts_node
```

---

## 🔍 Check Installed Nodes

```bash
ros2 pkg executables my_voice_assistant
```

---

## 🧪 Test Voice Message Publishing

```bash
ros2 topic pub --once /speech_text std_msgs/msg/String "data: 'Hello, who are you'"
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
./rebuild.sh         # Execute rebuild anytime
```
