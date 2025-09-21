# 实时 STT 节点启动脚本使用说明

## 📁 可用脚本

### 1. `start_realtime_stt.sh` - 完整版启动脚本
功能齐全的启动脚本，包含环境检查、参数配置和错误处理。

**基本使用:**
```bash
./start_realtime_stt.sh
```

**自定义参数:**
```bash
# 高灵敏度配置
./start_realtime_stt.sh --vad-threshold 0.010

# 快速响应配置
./start_realtime_stt.sh --silence-duration 1.0

# 大缓冲区配置
./start_realtime_stt.sh --buffer-history 2.5

# 调试模式
./start_realtime_stt.sh --debug

# 组合参数
./start_realtime_stt.sh --vad-threshold 0.012 --buffer-history 2.0 --debug
```

**支持的参数:**
- `--vad-threshold <值>`: VAD 阈值 (默认: 0.015)
- `--silence-duration <值>`: 静音检测时间 (默认: 2.0秒)
- `--min-speech-duration <值>`: 最小语音时长 (默认: 0.2秒)
- `--buffer-history <值>`: 前缓冲区时间 (默认: 1.5秒)
- `--debug`: 启用详细日志

### 2. `start_realtime_stt_simple.sh` - 简化版启动脚本
快速启动，最少配置，适合日常使用。

```bash
./start_realtime_stt_simple.sh
```

### 3. `test_realtime_stt.sh` - 交互式测试脚本
提供多种配置选项的测试界面。

```bash
./test_realtime_stt.sh
```

## 🔧 参数配置指南

### VAD 阈值 (vad_threshold)
控制语音活动检测的敏感度:
- **0.010**: 非常敏感，适合安静环境
- **0.015**: 默认值，平衡性能
- **0.025**: 较低敏感度，适合嘈杂环境

### 静音检测时间 (silence_duration)
语音结束后等待的静音时间:
- **1.0秒**: 快速响应，适合短句
- **2.0秒**: 默认值，适合正常对话
- **3.0秒**: 较长等待，适合思考性对话

### 前缓冲区时间 (buffer_history)
语音开始前保留的音频时间:
- **1.0秒**: 较短缓冲区
- **1.5秒**: 默认值，确保捕获完整唤醒词
- **2.5秒**: 大缓冲区，适合确保完整性

## 🎯 使用场景推荐

### 安静环境 + 快速响应
```bash
./start_realtime_stt.sh --vad-threshold 0.010 --silence-duration 1.0
```

### 嘈杂环境 + 稳定性优先
```bash
./start_realtime_stt.sh --vad-threshold 0.025 --buffer-history 2.5
```

### 调试和优化
```bash
./start_realtime_stt.sh --debug --vad-threshold 0.012
```

## 🚨 故障排除

### 常见问题

1. **节点启动失败**
   - 检查 OPENAI_API_KEY 是否设置
   - 确保已构建包: `colcon build --packages-select my_voice_assistant`

2. **音频权限错误**
   - 检查 Docker 容器的音频设备访问权限
   - 确保 pyaudio 正确安装

3. **VAD 过于敏感/不敏感**
   - 调整 `--vad-threshold` 参数
   - 使用 `--debug` 模式观察 RMS 值

4. **唤醒词识别不准确**
   - 增加 `--buffer-history` 值
   - 降低 `--vad-threshold` 值
   - 确保清晰发音

### 监控和调试

监听转录结果:
```bash
ros2 topic echo /speech_text
```

检查节点状态:
```bash
ros2 node list | grep stt
```

查看详细日志:
```bash
./start_realtime_stt.sh --debug
```

## 🔇 关闭唤醒词检测

如果你通过硬件开关来控制麦克风拾音，可以关闭唤醒词检测：
```bash
export STT_ENABLE_WAKE_WORD=0
# 或在启动时通过 ROS 参数禁用
ros2 run my_voice_assistant openai_stt_node_with_vad --ros-args -p enable_wake_word:=false
```
禁用后所有识别片段都会发布到 `/speech_text`，同时仍保留 TTS 自回放过滤和 SNR 门控，避免音频回环。

## ⚡ 启用实时打断 (Barge-in)

开启实时打断后，助手播放语音时麦克风仍保持监听，一旦检测到用户再次说话会立即发送 `tts_interrupt` 终止当前回复。
```bash
export STT_ALLOW_BARGE_IN=1
export STT_BARGE_IN_INTERRUPT=1  # 可选，控制是否自动发送中断信号
ros2 run my_voice_assistant openai_stt_node_with_vad --ros-args -p allow_barge_in:=true
```
- `STT_BARGE_IN_COOLDOWN` 可调节连续打断的冷却时间（秒，默认 0.7）。
- 若希望仅保持监听、不主动发出中断信号，可将 `STT_BARGE_IN_INTERRUPT` 设为 0。
- 建议搭配硬件静音或耳机收音，避免扬声器回放被重新识别。

## 📊 性能对比

| 配置 | 延迟 | 准确性 | 资源消耗 | 适用场景 |
|------|------|--------|----------|----------|
| 高敏感度 | 低 | 中 | 高 | 安静环境 |
| 默认配置 | 中 | 高 | 中 | 一般使用 |
| 低敏感度 | 高 | 高 | 低 | 嘈杂环境 |

## 🔗 相关话题

- **发布**: `/speech_text` - 转录的文本结果
- **订阅**: `/tts_status` - TTS 播放状态

## 💡 最佳实践

1. **环境准备**: 确保相对安静的环境
2. **清晰发音**: 唤醒词需要清晰准确
3. **适当停顿**: 语句间保持适当停顿
4. **参数调优**: 根据环境调整 VAD 参数
5. **监控日志**: 使用调试模式优化配置
