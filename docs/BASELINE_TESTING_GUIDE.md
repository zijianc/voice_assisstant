# Baseline Performance Testing Guide

## 概述

这套工具用于测试传统 STT → LLM → TTS 管道的性能，并与 OpenAI Realtime 方法进行对比，为学术报告提供严谨的对比数据。

## 文件说明

### 核心脚本
- `baseline_performance_test.py` - 基准性能测试主程序
- `run_baseline_test.sh` - 基准测试启动脚本
- `compare_performance.py` - 性能对比分析程序
- `run_complete_comparison.sh` - 完整对比测试脚本

### 节点组件
测试使用以下传统管道节点：
- `openai_stt_node` - 语音转文本
- `llm_node` - 大语言模型处理
- `openai_tts_node` - 文本转语音

## 使用方法

### 方法1: 完整自动化测试（推荐）

```bash
# 运行完整的对比测试（自动查找现有realtime数据）
./run_complete_comparison.sh --samples 30

# 或指定特定的realtime结果文件
./run_complete_comparison.sh --samples 30 --realtime-file rigorous_latency_data_20250929_032524.csv
```

### 方法2: 分步手动测试

#### 步骤1: 运行基准测试
```bash
# 基本测试（30个样本）
./run_baseline_test.sh

# 自定义样本数和输出文件
./run_baseline_test.sh --samples 50 --output my_baseline_results.json

# 如果节点已在运行，跳过启动
./run_baseline_test.sh --no-nodes
```

#### 步骤2: 运行对比分析
```bash
# 基本对比
python3 compare_performance.py \
    --baseline baseline_results.json \
    --realtime rigorous_latency_data_20250929_032524.csv

# 包含可视化图表
python3 compare_performance.py \
    --baseline baseline_results.json \
    --realtime rigorous_latency_data_20250929_032524.csv \
    --viz
```

## 测试指标

### 基准测试指标
- **Speech → First Response**: 从语音输入到首个LLM响应的延迟
- **Speech → TTS Start**: 从语音输入到TTS开始播放的延迟  
- **Total Response Time**: 完整响应周期的总时间

### 对比分析指标
- **改进百分比**: 相对于基准的性能提升
- **绝对改进**: 延迟减少的毫秒数
- **统计置信度**: 包含置信区间的统计分析

## 输出文件

### 基准测试输出
- `baseline_results_TIMESTAMP.json` - 完整测试结果和统计数据
- `baseline_results_TIMESTAMP.csv` - 原始测试数据
- `baseline_results_TIMESTAMP_report.txt` - 可读格式报告

### 对比分析输出
- `performance_comparison_TIMESTAMP.json` - 对比分析结果
- `performance_comparison_TIMESTAMP_report.txt` - 对比报告
- `performance_comparison.png` - 可视化图表（需要matplotlib）

## 学术报告使用

### Section 5.1 修改示例

原文：
```markdown
This section presents the experimental findings and performance evaluation of the speech‑enabled shuttle assistant, comparing the OpenAI Realtime node approach against the conventional STT → LLM → TTS baseline.
```

修改后：
```markdown
This section presents the experimental findings and performance evaluation of the speech‑enabled shuttle assistant, comparing the OpenAI Realtime node approach against the conventional STT → LLM → TTS baseline. Direct comparative testing demonstrates X% latency improvement (Y.Yms vs Z.Zms median, n=30 each) over the conventional pipeline.
```

### 数据引用
```markdown
- **Baseline (STT→LLM→TTS)**: X.Xms median [A.A–B.Bms IQR] (n=30)
- **Realtime (OpenAI API)**: Y.Yms median [C.C–D.Dms IQR] (n=53)  
- **Improvement**: Z.Z% latency reduction, confirming architectural advantages

Data source: baseline_results_TIMESTAMP.csv, performance_comparison_TIMESTAMP.json
```

## 故障排除

### 常见问题

1. **节点启动失败**
   ```bash
   # 检查ROS2环境
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   
   # 重新构建工作空间
   colcon build --packages-select my_voice_assistant
   ```

2. **API密钥错误**
   ```bash
   # 检查.env文件
   cat .env | grep OPENAI_API_KEY
   
   # 手动设置
   export OPENAI_API_KEY=your_key_here
   ```

3. **测试超时**
   - 检查网络连接
   - 增加测试间隔时间
   - 减少样本数量进行初步测试

4. **数据对比失败**
   - 确认realtime数据文件存在且格式正确
   - 检查字段名称匹配
   - 验证数据文件不为空

### 调试模式

```bash
# 启动单个节点进行调试
ros2 run my_voice_assistant openai_stt_node
ros2 run my_voice_assistant llm_node  
ros2 run my_voice_assistant openai_tts_node

# 监控ROS话题
ros2 topic echo speech_text
ros2 topic echo llm_response
ros2 topic echo tts_status

# 检查节点状态
ros2 node list
ros2 topic list
```

## 性能优化建议

1. **减少网络延迟影响**
   - 在网络条件良好时测试
   - 多次测试取平均值
   - 考虑网络基线延迟

2. **确保测试一致性**
   - 使用相同的测试提示词
   - 保持相同的环境配置
   - 避免并发网络活动

3. **统计有效性**
   - 使用足够的样本数（n≥30）
   - 排除明显异常值
   - 报告置信区间

## 结果验证

测试完成后，验证以下关键点：
- [ ] 基准测试有效样本数 ≥ 25（83%成功率）
- [ ] 延迟改进百分比 ≥ 35%（接近声称的40%）
- [ ] 统计数据包含置信区间
- [ ] 数据文件完整且可读
- [ ] 对比报告逻辑清晰

通过这套工具，你可以获得学术级别的性能对比数据，为报告的Section 5提供严谨的实验支撑。