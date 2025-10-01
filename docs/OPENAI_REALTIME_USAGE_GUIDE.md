# OpenAI Realtime API 语音助手使用指南

## 🎙️ 系统概述

OpenAI Realtime 节点是对传统 LLM + TTS 架构的重大升级，整合了语言模型和语音合成功能，通过 WebSocket 连接实现低延迟的实时语音交互。

## 📊 架构对比

### 传统架构（LLM + TTS）
```
STT节点 → LLM节点 → TTS节点 → 音频输出
       ↑         ↑         ↑
   语音转文本   文本生成   文本转语音
```

### Realtime架构
```
STT节点 → Realtime节点 ← → OpenAI Realtime API
              ↓
         直接音频输出
```

## 🚀 主要优势

### 1. **超低延迟**
- **传统方案**: STT → LLM → TTS 三步处理，总延迟 3-8秒
- **Realtime方案**: 直接WebSocket连接，端到端延迟 < 1秒

### 2. **原生语音质量**
- **传统方案**: 文本 → 语音转换，语调较为机械
- **Realtime方案**: 直接语音生成，自然语调和情感表达

### 3. **实时流式处理**
- **传统方案**: 等待完整响应后才开始TTS
- **Realtime方案**: 边生成边播放，持续流式输出

### 4. **智能中断处理**
- **传统方案**: 中断处理复杂，需要协调多个节点
- **Realtime方案**: 原生中断支持，实时响应用户输入

## ⚙️ 配置说明

### 环境变量配置
```bash
# 必需配置
export OPENAI_API_KEY="your_api_key_here"

# Realtime API 配置
export REALTIME_MODEL="gpt-4o-realtime-preview-2024-10-01"
export REALTIME_VOICE="alloy"  # alloy, echo, fable, onyx, nova, shimmer
export ENABLE_REALTIME_FUNCTION_CALLING="1"
export REALTIME_AUDIO_OUTPUT="1"

# 网络搜索配置
export ENABLE_WEB_SEARCH="1"
```

### 语音选择
- **alloy**: 中性、清晰，适合信息播报
- **echo**: 温和、友好，适合对话交互
- **fable**: 英式口音，正式场合
- **onyx**: 深沉、稳重，适合权威信息
- **nova**: 活力、年轻，适合互动应用
- **shimmer**: 柔和、温暖，适合客服场景

## 🔧 功能特性

### 1. **ExtendedUWATools集成**
Realtime 节点集成了完整的校园服务功能：

**基础搜索工具**:
- `search_web()` - 通用网络搜索
- `search_uwa_transport()` - UWA交通信息
- `search_current_weather()` - 实时天气查询
- `search_uwa_events()` - UWA活动信息

**扩展校园工具** (🆕 Google搜索策略):
- `search_uwa_locations()` - 校园建筑和设施位置
- `get_uwa_hours()` - 设施开放时间查询
- `search_campus_dining()` - 校园餐饮选择
- `check_parking_availability()` - 停车位信息
- `find_nearby_services()` - 附近服务设施

### 2. **ROS2 消息兼容性**
保持与现有系统的兼容性：

**订阅话题**:
- `speech_text` - 接收STT识别结果
- `realtime_interrupt` - 接收中断信号

**发布话题**:
- `realtime_response` - 流式文本响应
- `realtime_response_full` - 完整响应
- `realtime_response_end` - 响应结束信号
- `realtime_status` - 系统状态

### 3. **智能对话管理**
- 自动上下文管理
- 多轮对话支持
- 智能打断和恢复
- 错误处理和重连机制

## 📋 使用步骤

### 1. 启动 Realtime 节点
```bash
cd /workspaces/ros2_ws
./start_openai_realtime.sh
```

### 2. 启动 STT 节点（在新终端）
```bash
# 选择STT节点（推荐TEN VAD）
cd /workspaces/ros2_ws

# 传统RMS VAD STT（已更新"new way 4"唤醒词）
./start_realtime_stt.sh

# 或TEN深度学习VAD STT（更高精度）
./start_ten_vad_stt.sh
```

### 3. 测试系统
**自动化测试**:
```bash
cd /workspaces/ros2_ws/src/my_voice_assistant/my_voice_assistant
python3 test_realtime_questions.py
```

**手动测试**:
```bash
# 测试唤醒词和功能
ros2 topic pub /speech_text std_msgs/String "data: 'new way 4, where is Reid Library?'" --once
ros2 topic pub /speech_text std_msgs/String "data: 'neway 4, what time does the library close?'" --once
```

## 🧪 测试案例

### 基础对话测试
```
用户: "new way 4, how are you today?"
系统: [流式语音响应] "Hello! I'm doing great, thank you for asking..."
```

### 网络搜索测试
```
用户: "new way 4, what's the weather like in Perth today?"
系统: [触发search_current_weather] → [基于实时天气数据回答]
```

### 校园信息查询测试
```
用户: "new way 4, where is Reid Library?"
系统: [触发search_uwa_locations] → [Google实时搜索] → [详细位置信息]

用户: "new way 4, what time does the library close?"
系统: [触发get_uwa_hours] → [Google实时搜索] → [最新开放时间]

用户: "new way 4, where can I get coffee on campus?"
系统: [触发search_campus_dining] → [Google实时搜索] → [餐饮选择]
```

### 唤醒词变体测试
```
支持的唤醒词变体:
- "new way 4"
- "new way four" 
- "neway 4"
- "neway four"
- "nu way 4"
- "new way for"
- "neway for"
等等...
```

### 中断处理测试
```
用户: "Tell me a long story..."
系统: [开始响应]
用户: [中断信号]
系统: [立即停止，准备接收新输入]
```

## 📈 性能监控

### 关键指标
- **连接延迟**: WebSocket建立时间
- **响应延迟**: 从输入到开始播放的时间
- **音频质量**: 语音自然度和清晰度
- **功能调用**: 网络搜索成功率

### 日志监控
```bash
# 查看实时日志
ros2 topic echo /realtime_response

# 监控系统状态
ros2 topic echo /realtime_status
```

## 🔄 与传统节点对比

| 特性 | 传统LLM+TTS | Realtime API |
|------|-------------|--------------|
| 延迟 | 3-8秒 | <1秒 |
| 音频质量 | 机械合成 | 自然语音 |
| 中断处理 | 复杂协调 | 原生支持 |
| 流式处理 | 分段处理 | 真正流式 |
| 资源占用 | 多进程 | 单连接 |
| 网络搜索 | ✅ | ✅ |
| 自定义性 | 高 | 中 |

## 🛠️ 故障排除

### 常见问题

1. **连接失败**
   ```bash
   # 检查API密钥
   echo $OPENAI_API_KEY
   
   # 检查网络连接
   ping api.openai.com
   ```

2. **音频输出问题**
   ```bash
   # 检查音频设备
   pactl list short sinks
   
   # 测试音频输出
   speaker-test -t sine -f 1000 -l 1
   ```

3. **功能调用失败**
   ```bash
   # 检查web_search_tools
   python3 -c "from web_search_tools import WebSearchTools; print('OK')"
   ```

### 调试技巧
```bash
# 启用详细日志
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG

# 监控WebSocket消息
# （在realtime_node.py中已包含事件日志）
```

## 🔮 未来扩展

### 已实现功能 (2025年9月28日更新)
- ✅ **ExtendedUWATools**: 5个校园专用工具完全集成
- ✅ **Google搜索策略**: 解决静态数据问题，提供实时信息
- ✅ **唤醒词优化**: "new way 4"变体，提高识别宽容度
- ✅ **Function Calling**: 完整的网络搜索和校园服务功能

### 计划功能
- **多语言支持**: 中文语音输入输出
- **情感识别**: 基于语音的情感分析
- **个性化**: 用户偏好学习和适应
- **更多工具集成**: 交通、学术、生活服务工具扩展

### 集成建议
- **视觉界面**: 配合GUI显示对话状态
- **移动应用**: 开发移动端配套应用
- **IoT集成**: 连接智能设备控制

## 📞 技术支持

如需技术支持或功能建议，请：
1. 查看日志输出定位问题
2. 运行测试脚本验证功能
3. 检查环境配置和依赖
4. 参考OpenAI官方文档

## 📝 更新日志

### 2025年9月28日 - v2.0 重大更新
- 🚀 **ExtendedUWATools集成**: 添加5个校园专用工具
  - `search_uwa_locations()` - 校园建筑位置
  - `get_uwa_hours()` - 设施开放时间
  - `search_campus_dining()` - 餐饮服务
  - `check_parking_availability()` - 停车信息
  - `find_nearby_services()` - 附近服务
- 🔍 **Google搜索策略**: 从UWA官网限制搜索转换为Google实时搜索
- 📱 **唤醒词升级**: 从"nUWAy"更新为"new way 4"，支持多种变体
- 🎯 **宽容度提升**: 降低识别阈值从0.86到0.75，提高识别成功率
- 📊 **工具验证**: 每个功能提供4+条Google验证方法指导
- 🐛 **问题修复**: 解决时间不准确、温度信息错误等用户反馈问题

### 初始版本
- ⚡ OpenAI Realtime API基础集成
- 🎙️ 低延迟语音对话功能
- 🔄 ROS2消息兼容性
- 🌐 基础网络搜索工具

---

**注意**: Realtime API 目前处于预览阶段，功能和接口可能会有变化。建议定期更新相关代码和配置。