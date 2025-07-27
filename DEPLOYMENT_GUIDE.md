# 🚀 语音助手快速部署指南

基于我们的性能优化工作，这是一个完整的部署和使用指南。

## 📋 快速开始

### 1. 环境准备
```bash
# 1. 克隆项目 (如果需要)
git clone <your-repo>
cd ros2_ws

# 2. 设置API密钥
cp .env.example .env
# 编辑 .env 文件，添加你的 OpenAI API 密钥
```

### 2. 安装依赖
```bash
# ROS2 依赖
source /opt/ros/humble/setup.bash

# Python 依赖
pip install openai pygame pyaudio numpy python-dotenv

# 系统依赖 (Ubuntu)
sudo apt-get update
sudo apt-get install -y portaudio19-dev python3-pyaudio
```

### 3. 编译项目
```bash
colcon build --packages-select my_voice_assistant
source install/setup.bash
```

## ⚙️ 配置说明

### 最优性能配置 (`.env`)
```bash
# OpenAI API配置
OPENAI_API_KEY=your_api_key_here

# TTS优化配置 (基于性能测试结果)
TTS_MODEL=gpt-4o-mini-tts     # 最新最快模型 (2.25秒响应)
TTS_VOICE=coral               # 推荐语音，自然清晰
TTS_FORMAT=opus               # 最低延迟格式 (32KB文件)
TTS_SPEED=1.1                 # 优化语音速度
TTS_SAVE_MODE=false           # 不保存临时文件

# STT配置
OPENAI_STT_MODEL=whisper-1    # OpenAI Whisper模型
```

### 替代配置方案

#### 高质量配置 (适合录制)
```bash
TTS_MODEL=tts-1-hd
TTS_VOICE=nova
TTS_FORMAT=mp3
TTS_SPEED=1.0
```

#### 极速配置 (适合实时对话)
```bash
TTS_MODEL=gpt-4o-mini-tts
TTS_VOICE=coral
TTS_FORMAT=wav
TTS_SPEED=1.2
```

## 🎯 启动服务

### 方式一：分别启动 (推荐用于开发)
```bash
# 终端1: 启动优化TTS节点
./start_tts_fast.sh

# 终端2: 启动LLM节点 (含内容过滤)
./start_llm.sh

# 终端3: 启动实时STT节点 (含VAD)
./start_realtime_stt.sh
```

### 方式二：一键启动脚本
创建 `start_all.sh`:
```bash
#!/bin/bash
echo "🚀 启动语音助手系统"

# 检查环境
if [ ! -f .env ]; then
    echo "❌ .env 文件不存在，请先配置API密钥"
    exit 1
fi

source .env
source install/setup.bash

# 启动所有节点
echo "启动 TTS 节点..."
./start_tts_fast.sh &
TTS_PID=$!

echo "启动 LLM 节点..."
./start_llm.sh &
LLM_PID=$!

echo "启动 STT 节点..."
./start_realtime_stt.sh &
STT_PID=$!

echo "✅ 所有节点已启动"
echo "PID: TTS=$TTS_PID, LLM=$LLM_PID, STT=$STT_PID"

# 等待用户中断
trap "echo '🛑 停止所有节点...'; kill $TTS_PID $LLM_PID $STT_PID 2>/dev/null; exit 0" INT
wait
```

## 🧪 测试验证

### 1. 基础功能测试
```bash
# TTS基础测试
./test_tts_simple.sh

# 预期结果: 
# ✅ 测试成功!
# 生成时间: ~2秒
# 播放正常
```

### 2. 性能基准测试
```bash
# 完整性能测试
./test_tts_performance.sh

# 预期结果:
# 🧪 最新模型+最快格式: 2.25秒 ✅
# 🧪 传统模型+MP3: 2.63秒 ✅
# 🧪 高质量模型: 4.07秒 ⚠️
```

### 3. 内容过滤测试
```bash
# 过滤功能测试
python3 src/my_voice_assistant/my_voice_assistant/test/test_content_filter.py

# 预期结果:
# 测试 1: 75 -> 49 字符 (过滤【】内容)
# 测试 2: 111 -> 85 字符 (过滤数字引用)
```

### 4. 端到端测试
```bash
# 发送测试消息
ros2 topic pub /llm_response std_msgs/String \
  "data: 'Hello Captain! 【742442319583238†L62-L65】 How are you?'" --once

# 预期结果:
# - LLM节点过滤掉【】内容
# - TTS节点生成"Hello Captain! How are you?"
# - 音频正常播放
```

## 📊 系统监控

### ROS2话题监控
```bash
# 监控话题活动
ros2 topic list
# 预期输出:
# /llm_response
# /speech_text  
# /tts_status

# 监控消息内容
ros2 topic echo /llm_response    # LLM输出
ros2 topic echo /speech_text     # STT识别结果
ros2 topic echo /tts_status      # TTS状态
```

### 性能监控
```bash
# 系统资源使用
top -p $(pgrep -f "openai_tts_node|llm_node|realtime_stt")

# 网络延迟监控
ping api.openai.com

# 磁盘空间 (临时文件)
du -sh /tmp/*.wav /tmp/*.opus 2>/dev/null || echo "无临时文件"
```

## 🔧 常见问题解决

### 1. API密钥错误
```bash
# 症状: "❌ OPENAI_API_KEY 环境变量未设置"
# 解决:
echo $OPENAI_API_KEY  # 检查是否设置
cat .env | grep OPENAI_API_KEY  # 检查文件配置
source .env  # 重新加载环境变量
```

### 2. TTS响应慢
```bash
# 症状: 生成时间 > 3秒
# 解决:
echo $TTS_MODEL    # 确认使用 gpt-4o-mini-tts
echo $TTS_FORMAT   # 确认使用 opus 或 wav
curl -w "%{time_total}" https://api.openai.com/v1/models  # 检查网络
```

### 3. 音频播放问题
```bash
# 症状: 无声音或重复播放
# 解决:
python3 -c "import pygame; print('pygame版本:', pygame.version.ver)"
aplay /usr/share/sounds/alsa/Front_Left.wav  # 测试系统音频
sudo apt-get install --reinstall alsa-utils  # 重装音频驱动
```

### 4. STT不够敏感
```bash
# 症状: 语音检测不到
# 解决:
# 编辑 realtime_stt_node.py 中的阈值
self.rms_threshold = 800  # 降低阈值 (默认1000)
```

### 5. 内容过滤过度
```bash
# 症状: 正常内容被错误过滤
# 解决:
# 调整 llm_node.py 中的过滤规则
# 或临时禁用过滤: 注释掉 filter_content() 调用
```

## 📈 性能优化建议

### 1. 网络优化
```bash
# 使用HTTP/2连接池
export OPENAI_HTTP_CLIENT="httpx"

# DNS缓存
echo "nameserver 8.8.8.8" | sudo tee -a /etc/resolv.conf
```

### 2. 系统优化
```bash
# 提高进程优先级
sudo nice -n -10 ./start_tts_fast.sh

# 增加音频缓冲
export SDL_AUDIODRIVER=pulse
```

### 3. 缓存优化
```bash
# 启用TTS结果缓存 (修改代码)
TTS_CACHE_ENABLED=true
TTS_CACHE_SIZE=100
```

## 🔄 版本升级

### 从旧版本升级
```bash
# 1. 备份配置
cp .env .env.backup

# 2. 更新代码
git pull origin main

# 3. 重新编译
colcon build --packages-select my_voice_assistant

# 4. 更新配置 (如果需要)
# 对比 .env.example 和 .env，添加新配置项

# 5. 重启服务
./restart_all.sh
```

### 配置迁移
```bash
# 从旧配置格式迁移
if grep -q "OLD_TTS_MODEL" .env; then
    echo "检测到旧配置，请更新为新格式"
    echo "TTS_MODEL=gpt-4o-mini-tts" >> .env
fi
```

## 📚 使用场景

### 1. 开发调试
```bash
# 启动单个组件进行调试
./start_tts_fast.sh
# 在另一个终端发送测试消息
ros2 topic pub /llm_response std_msgs/String "data: 'test message'" --once
```

### 2. 生产部署
```bash
# 使用systemd服务
sudo cp voice-assistant.service /etc/systemd/system/
sudo systemctl enable voice-assistant
sudo systemctl start voice-assistant
```

### 3. 演示展示
```bash
# 运行完整演示
./demo.sh
# 包含: 自动测试 + 交互演示 + 性能展示
```

## 🎯 下一步改进

### 短期目标 (1-2周)
- [ ] 实现TTS结果缓存
- [ ] 添加语音情感控制
- [ ] 优化VAD算法

### 中期目标 (1个月)
- [ ] 支持多语言TTS
- [ ] 实现流式TTS
- [ ] 添加语音克隆功能

### 长期目标 (3个月)
- [ ] 本地TTS模型
- [ ] 实时语音转换
- [ ] 完整的语音对话系统

---

## 📞 技术支持

### 日志分析
```bash
# ROS2日志
ros2 node info /openai_tts_node
ros2 node info /llm_node
ros2 node info /realtime_stt_node

# 系统日志
journalctl -u voice-assistant -f
```

### 性能分析
```bash
# 生成性能报告
./generate_performance_report.sh > performance_$(date +%Y%m%d).txt
```

### 联系方式
- 文档: 查看 `PERFORMANCE_OPTIMIZATION_GUIDE.md`
- 技术细节: 查看 `TECHNICAL_IMPLEMENTATION_DETAILS.md`
- 问题反馈: 创建 GitHub Issue

---

*部署指南 v1.0*  
*最后更新: 2025年7月27日*  
*基于性能优化 v2.0*
