# Qwen TTS 中文方言语音合成使用指南

## 📖 项目概述

本项目集成了阿里云通义千问（Qwen TTS）的中文方言语音合成功能，支持多种中文方言音色，特别是Jada（吴语-女）音色，为语音助手添加了更丰富的语音表达能力。

## 🎭 支持的音色

### 标准音色
- **Chelsie**（女） - 标准普通话女声
- **Cherry**（女） - 标准普通话女声  
- **Ethan**（男） - 标准普通话男声
- **Serena**（女） - 标准普通话女声

### 方言音色 ⭐
- **Dylan**（北京话-男） - 适合北京话、京腔
- **Jada**（吴语-女） - 适合上海话、苏州话等吴语方言 ✨ **默认推荐**
- **Sunny**（四川话-女） - 适合四川话、川渝方言

## 🚀 快速开始

### 1. 环境准备
```bash
# 确认API密钥已配置
grep DASHSCOPE_API_KEY /workspaces/ros2_ws/.env

# 构建项目
cd /workspaces/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select my_voice_assistant
```

### 2. 启动Qwen TTS节点
```bash
# 使用启动脚本（推荐）
./start_qwen_tts.sh

# 或手动启动
source install/setup.bash
ros2 run my_voice_assistant qwen_tts_node
```

### 3. 测试功能
```bash
# 运行完整测试
./test_qwen_tts.sh

# 手动发送测试消息
ros2 topic pub --once /llm_response std_msgs/msg/String "data: '侬好！我是上海宁！'"
```

## ⚙️ 配置选项

### 环境变量配置
在 `.env` 文件中设置：
```bash
# API认证（必需）
DASHSCOPE_API_KEY=你的API密钥

# 音色配置
QWEN_TTS_MODEL=qwen-tts-latest    # 模型版本
QWEN_TTS_VOICE=Jada              # 音色选择

# 输出配置  
TTS_SAVE_MODE=true               # 保存音频文件
```

### 详细配置
查看完整配置选项：
```bash
cat qwen_tts_config.env
```

## 🎵 音频输出

### 文件格式
- **格式**: WAV
- **采样率**: 24kHz
- **声道**: 单声道
- **位深**: 16-bit

### 保存位置
```bash
# 音频文件保存目录
/workspaces/ros2_ws/audio_output/

# 文件命名格式
qwen_tts_YYYYMMDD_HHMMSS.wav
```

### 播放方式
```bash
# 在宿主机上播放（推荐）
# 将文件复制到宿主机后使用音频播放器

# 检查生成的文件
ls -la /workspaces/ros2_ws/audio_output/qwen_tts_*.wav
```

## 🔧 集成指南

### 与现有TTS节点对比

| 特性 | OpenAI TTS | Qwen TTS |
|------|------------|----------|
| 多语言支持 | ✅ 多种语言 | ✅ 中英混合 |
| 中文方言 | ❌ 不支持 | ✅ 支持 3种方言 |
| 实时性 | ✅ 很好 | ✅ 好（400ms内首包）|
| 音质 | ✅ 优秀 | ✅ 优秀 |
| 成本 | 💰 较高 | 💰 较低 |

### 话题接口
```bash
# 订阅话题
/llm_response (std_msgs/String) - 接收要合成的文本

# 发布话题  
/tts_status (std_msgs/Bool) - 发布播放状态
```

### 切换TTS引擎
```bash
# 使用OpenAI TTS
./start_openai_tts.sh

# 使用Qwen TTS（中文方言）
./start_qwen_tts.sh

# 两者可以同时运行，但建议只运行一个避免冲突
```

## 💡 使用技巧

### 1. 提高方言识别率
```python
# ✅ 好的做法 - 使用地方化文本
"侬好！今朝天气蛮好个！"     # 上海话
"您好！今儿天气真不错！"      # 北京话  
"你好！今天天气巴适得很！"    # 四川话

# ❌ 避免 - 过于正式的文本可能切换为普通话
"根据气象部门发布的天气预报显示..."
```

### 2. 文本分段处理
```python
# 长文本建议按标点符号分段
text_segments = text.split('。')
for segment in text_segments:
    if segment.strip():
        send_to_tts(segment + '。')
```

### 3. 音色选择建议
```python
# 不同场景的音色推荐
场景配置 = {
    "上海地区": "Jada",      # 吴语-女
    "北京地区": "Dylan",     # 北京话-男  
    "四川地区": "Sunny",     # 四川话-女
    "通用场景": "Cherry",    # 标准普通话-女
}
```

## 🐛 故障排除

### 常见问题

1. **API调用失败**
```bash
# 检查API密钥
echo $DASHSCOPE_API_KEY

# 检查网络连接
curl -I https://dashscope.aliyuncs.com

# 检查账户额度
# 登录阿里云百炼控制台查看余额
```

2. **音频文件未生成**
```bash
# 检查目录权限
ls -la /workspaces/ros2_ws/audio_output/

# 检查磁盘空间
df -h /workspaces/ros2_ws

# 查看详细错误日志
ros2 run my_voice_assistant qwen_tts_node --ros-args --log-level debug
```

3. **方言音色说普通话**
```markdown
这是正常现象，原因：
- 面对正式文本时，AI模型会自动切换为标准普通话
- 解决方案：使用更地方化、口语化的文本
- 或者按句子分段，增加方言使用概率
```

### 调试命令
```bash
# 查看节点状态
ros2 node list | grep qwen

# 查看话题列表  
ros2 topic list | grep tts

# 监听音频状态
ros2 topic echo /tts_status

# 查看详细日志
ros2 run my_voice_assistant qwen_tts_node --ros-args --log-level debug
```

## 📊 性能特点

### 延迟指标
- **首包延迟**: < 400ms（理论值）
- **总处理时间**: 2-5秒（取决于文本长度）
- **网络影响**: 依赖外网连接速度

### 成本效率
- **计费规则**: 每1秒音频 = 50 Token
- **免费额度**: 100万Token（180天有效期）
- **预估成本**: 1分钟音频 ≈ 3000 Token

### 质量评估
- **音质**: 24kHz高品质音频
- **自然度**: 真人级别的停顿和语气
- **稳定性**: 支持中英文长难句
- **方言准确性**: 地方化文本准确率高

## 🔄 版本信息

- **Qwen TTS版本**: qwen-tts-latest
- **DashScope SDK**: 1.24.1+
- **支持音色**: 7种（4种标准 + 3种方言）
- **更新日期**: 2025年8月

## 📚 相关文档

- [阿里云Qwen TTS官方文档](https://help.aliyun.com/zh/model-studio/qwen-tts)
- [DashScope API参考](https://help.aliyun.com/zh/model-studio/qwen-tts-api)
- [项目配置文件](qwen_tts_config.env)
- [测试脚本说明](test_qwen_tts.sh)

---

🎭 **享受中文方言语音合成的魅力！** 如有问题，请查看故障排除部分或查阅官方文档。
