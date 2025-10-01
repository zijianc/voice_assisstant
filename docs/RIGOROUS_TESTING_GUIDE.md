# 严谨测试框架说明

## 🎯 目标
为学术报告提供统计学上严谨、可重现的测试数据，解决之前测试中的数据不匹配问题。

## 📊 测试设计原则

### 1. 统计显著性
- **样本量**: 每项测试n≥30，确保统计显著性
- **重复性**: 支持多轮测试和结果聚合
- **置信区间**: 计算95%置信区间
- **异常值处理**: IQR方法检测和处理异常值

### 2. 学术规范
- **数据溯源**: 所有结果可追溯到具体测试参数
- **时间戳**: 完整的时间戳记录
- **原始数据**: 保留原始数据供验证
- **统计报告**: 符合学术标准的统计描述

### 3. 测试覆盖
- **Wake-Word测试**: 多SNR级别 × 30样本 × 3类别 = 270+样本
- **延迟测试**: 50+样本，包含完整的端到端延迟链
- **中断测试**: 60+样本，多种中断时机专门测试

## 🧪 测试脚本

### 1. `rigorous_wakeword_test.py`
**目标**: 获得统计学上显著的唤醒词性能数据

**特点**:
- 每个SNR级别30个样本（统计学要求n≥30）
- 正样本、负样本、混淆样本三类平衡测试
- 支持多轮测试和结果聚合
- 严格的数据记录和验证

**用法**:
```bash
python3 rigorous_wakeword_test.py --snr-levels 20 10 0 --samples-per-snr 30 --rounds 1
```

**输出**:
- `rigorous_wakeword_raw_*.json`: 原始测试数据
- `rigorous_wakeword_summary_*.json`: 汇总统计指标
- `rigorous_wakeword_data_*.csv`: CSV格式数据
- `rigorous_wakeword_report_*.txt`: 人类可读报告

### 2. `rigorous_latency_test.py`
**目标**: 获得统计学上可靠的延迟性能数据

**特点**:
- 50+样本确保统计显著性
- 同时测量多个延迟指标
- 包含置信区间计算
- 异常值检测和处理
- 网络基线延迟测量

**用法**:
```bash
python3 rigorous_latency_test.py --samples 50 --with-interruption --inter-trial-delay 2.0
```

**测量指标**:
- Speech-to-first-response latency
- Speech-to-TTS-start latency  
- Total response time
- Interruption latency (如启用)
- Network baseline correction

### 3. `rigorous_interruption_test.py`
**目标**: 专门测试中断响应性能

**特点**:
- 大样本中断测试 (n≥50)
- 多种中断时机测试 (0.2s, 0.5s, 1.0s, 2.0s)
- 中断成功率分析
- 系统恢复时间测量
- P95/P99百分位数分析

**用法**:
```bash
python3 rigorous_interruption_test.py --samples 60 --interruption-times 0.2 0.5 1.0 2.0
```

**测量指标**:
- Interruption latency (中断信号→TTS停止)
- System cleanup time (TTS停止→系统就绪)
- Recovery response time (系统就绪→新请求响应)
- Total recovery time (中断信号→完全恢复)
- Success rates (中断成功率、恢复成功率)

### 4. `rigorous_test_manager.py`
**目标**: 统一管理和执行所有严谨测试

**特点**:
- 前提条件检查
- 测试执行管理
- 进度监控和异常处理
- 综合执行报告

**用法**:
```bash
# 运行所有测试
python3 rigorous_test_manager.py --run-all

# 运行特定测试
python3 rigorous_test_manager.py --wakeword --latency

# 仅运行中断测试
python3 rigorous_test_manager.py --interruption-only

# 检查已有结果
python3 rigorous_test_manager.py --check-results

# 列出可用测试
python3 rigorous_test_manager.py --list
```

## 📈 数据质量保证

### 统计要求
- **样本量**: n≥30 (Central Limit Theorem)
- **置信水平**: 95%置信区间
- **异常值**: IQR方法检测，<5%异常值率
- **重复性**: 结果在多次运行间变异系数<10%

### 报告格式
所有结果以学术标准格式报告：
```
中位数 [IQR] (n=样本数)
例: 0.279s [0.260-0.295] (n=50)
```

### 数据验证
- 时间戳连续性检查
- 数值范围合理性验证  
- 统计分布正态性测试
- 跨测试一致性验证

## 🔄 推荐执行流程

### 1. 环境检查
```bash
python3 rigorous_test_manager.py --list
python3 rigorous_test_manager.py --check-results
```

### 2. 执行测试
```bash
# 如果时间充足，执行完整测试套件
python3 rigorous_test_manager.py --run-all

# 如果时间有限，优先执行关键测试
python3 rigorous_test_manager.py --wakeword --latency
```

### 3. 结果验证
- 检查生成的报告文件
- 验证样本数是否达到要求
- 确认异常值比例合理
- 对比多次运行结果的一致性

### 4. 报告集成
- 将JSON/CSV数据导入到最终报告
- 使用标准统计描述格式
- 包含数据来源和测试时间
- 明确标注测试条件和限制

## ⚠️ 注意事项

### 测试环境
- 确保ROS2环境正确配置
- 网络连接稳定（用于API调用）
- 足够的存储空间保存结果文件
- 测试期间避免系统负载过高

### 时间规划
- Wake-word测试: ~15分钟
- Latency测试: ~20分钟
- Interruption测试: ~15分钟
- 总计: ~50分钟（完整测试套件）

### 数据使用
- 所有生成的数据可直接用于学术报告
- 时间戳和配置信息支持结果复现
- CSV格式便于进一步统计分析
- JSON格式保留完整元数据

## 🎓 学术价值

这套测试框架确保：
1. **统计严谨性**: 足够的样本量和正确的统计方法
2. **可重现性**: 详细的配置记录和标准化流程
3. **数据完整性**: 原始数据、中间结果、最终统计全保留
4. **学术规范**: 符合工程学科实验报告标准
5. **透明度**: 测试方法、参数、限制全部明确记录

通过这套严谨测试框架，你的学术报告将具有可靠的实验数据支撑。