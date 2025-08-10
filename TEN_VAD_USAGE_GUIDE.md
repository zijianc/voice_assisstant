# TEN VAD 集成使用指南

## 概述

TEN VAD 是一个高性能的语音活动检测系统，专为企业级对话AI应用设计。相比传统的RMS能量VAD，TEN VAD提供更精确的语音边界检测和更低的延迟。

## TEN VAD vs 当前RMS VAD 对比

| 特性 | 当前RMS VAD | TEN VAD |
|------|-------------|---------|
| **检测原理** | 能量阈值 + 静音检测 | 深度学习 + 帧级检测 |
| **精确度** | 中等，容易受噪声影响 | 高，比WebRTC/Silero更精确 |
| **延迟** | 低 | 更低（专为对话AI优化） |
| **噪声鲁棒性** | 一般 | 优秀 |
| **短暂静音处理** | 较差 | 优秀，能识别短暂间隔 |
| **计算复杂度** | 极低 | 低（比Silero VAD更低） |
| **内存占用** | 极小 | 小 |
| **库大小** | 无额外依赖 | ~300KB |

## 安装步骤

### 1. 安装TEN VAD
```bash
# 运行自动安装脚本
./install_ten_vad.sh

# 或手动安装
pip install git+https://github.com/TEN-framework/ten-vad.git
```

### 2. 安装系统依赖（Linux）
```bash
sudo apt update
sudo apt install libc++1
```

### 3. 编译ROS2包
```bash
colcon build --packages-select my_voice_assistant
source install/setup.bash
```

## 配置选项

在 `tts_config.env` 中添加以下配置：

```bash
# TEN VAD 配置
TEN_VAD_THRESHOLD=0.5              # VAD阈值 (0.0-1.0)
TEN_MIN_VOICE_FRAMES=5             # 最小语音帧数 (80ms)
TEN_MAX_SILENCE_FRAMES=50          # 最大静音帧数 (800ms)
TEN_BUFFER_HISTORY_FRAMES=30       # 历史缓冲帧数 (480ms)
```

### 参数调优建议

- **TEN_VAD_THRESHOLD**: 
  - `0.3-0.4`: 更敏感，适合安静环境
  - `0.5`: 默认平衡值
  - `0.6-0.7`: 更严格，适合噪声环境

- **TEN_MIN_VOICE_FRAMES**: 
  - `3-5`: 更快响应，可能有误触发
  - `5-8`: 平衡响应速度和准确性
  - `8-12`: 更保守，减少误触发

- **TEN_MAX_SILENCE_FRAMES**: 
  - `30-40`: 更快语音结束检测
  - `40-60`: 平衡值，适合正常语速
  - `60-80`: 更宽容，适合慢语速或有停顿

## 使用方法

### 启动TEN VAD STT节点
```bash
./start_ten_vad_stt.sh
```

### 性能测试
```bash
python3 vad_performance_comparison.py
```

### 切换到TEN VAD
1. 停止当前的realtime_stt_node
2. 启动ten_vad_stt_node
3. 其他节点（LLM、TTS）保持不变

## 性能优势

### 1. 精确的语音边界检测
- **当前RMS VAD**: 基于能量阈值，容易在噪声环境中误触发
- **TEN VAD**: 深度学习模型，能准确识别语音vs非语音

### 2. 快速转换检测
- **当前RMS VAD**: 需要较长静音时间确认语音结束
- **TEN VAD**: 快速检测语音到非语音转换，减少延迟

### 3. 短暂静音处理
- **当前RMS VAD**: 可能在说话间隙切断语音
- **TEN VAD**: 能识别相邻语音段间的短暂静音

### 4. 噪声鲁棒性
- **当前RMS VAD**: 容易受背景噪声、空调声等影响
- **TEN VAD**: 更好的噪声抑制能力

## 技术细节

### 音频要求
- **采样率**: 16kHz（TEN VAD固定要求）
- **格式**: 16-bit PCM
- **声道**: 单声道
- **帧大小**: 256 samples (16ms)

### 处理流程
1. 音频采集（16kHz, 256样本帧）
2. TEN VAD处理（输出概率和标志）
3. 状态机管理（静音→语音→处理）
4. 缓冲区管理（前置历史帧）
5. OpenAI Whisper转录
6. 唤醒词检测

### 内存和性能
- **RTF**: ~0.01-0.02（实时因子）
- **内存**: ~几MB
- **延迟**: 比当前RMS VAD更低
- **CPU**: 轻量级，适合实时应用

## 故障排除

### 1. 安装问题
```bash
# 检查TEN VAD是否正确安装
python3 -c "from ten_vad import TenVad; print('✅ TEN VAD OK')"

# 重新安装
pip uninstall ten-vad
pip install --force-reinstall git+https://github.com/TEN-framework/ten-vad.git
```

### 2. 运行时错误
```bash
# 检查系统依赖
dpkg -l | grep libc++1

# 检查音频设备
python3 -c "import pyaudio; p=pyaudio.PyAudio(); print(f'设备数: {p.get_device_count()}')"
```

### 3. 性能调优
- 如果误触发过多：提高 `TEN_VAD_THRESHOLD`
- 如果响应太慢：降低 `TEN_MAX_SILENCE_FRAMES`
- 如果截断语音：增加 `TEN_MAX_SILENCE_FRAMES`

## 迁移建议

### 渐进式迁移
1. **并行测试**: 同时运行两个VAD系统，比较结果
2. **A/B测试**: 在不同场景下测试两种方案
3. **逐步切换**: 先在测试环境验证，再切换生产环境

### 备份方案
- 保留原有的realtime_stt_node.py作为备份
- 可以通过启动脚本快速切换VAD方法
- 保持相同的ROS2接口，无需修改其他节点

## 监控和指标

### 关键指标
- **检测延迟**: 从语音开始到检测的时间
- **误检率**: 噪声被误识别为语音的比例
- **漏检率**: 语音未被检测到的比例
- **分割准确性**: 语音边界检测的准确性

### 日志监控
- 观察 "语音开始/结束" 日志的时机
- 监控唤醒词检测的成功率
- 检查是否有异常的短语音或长静音

## 总结

TEN VAD为你的语音助手系统提供了显著的性能提升：

1. **更精确的VAD**: 减少误触发和漏检
2. **更低的延迟**: 快速响应用户语音
3. **更好的用户体验**: 自然的对话流程
4. **企业级可靠性**: 在各种环境下稳定工作

建议逐步迁移到TEN VAD，以获得更好的语音助手性能。
