# 项目结构说明

## 重组后的项目结构

```
/workspaces/ros2_ws/
├── src/                          # ROS2 源代码
│   └── my_voice_assistant/       # 主要语音助手包
├── scripts/                      # 脚本文件
│   ├── start/                   # 启动脚本
│   │   ├── start_llm.sh
│   │   ├── start_openai_stt.sh
│   │   ├── start_openai_tts.sh
│   │   ├── start_realtime_stt.sh
│   │   ├── start_ten_vad_stt.sh
│   │   ├── start_tts_fast.sh
│   │   ├── start_tts.sh
│   │   └── start_baseline_llm.sh
│   ├── test/                    # 测试运行脚本
│   │   ├── run_baseline_test.sh
│   │   ├── run_complete_comparison.sh
│   │   ├── run_pure_baseline_test.sh
│   │   ├── run_short_baseline_test.sh
│   │   ├── test_openai_stt.sh
│   │   ├── test_realtime_stt.sh
│   │   └── test_tts_performance.sh
│   ├── setup/                   # 安装和配置脚本
│   │   ├── install_ten_vad.sh
│   │   └── rebuild.sh
│   └── run_from_root.sh         # 便捷运行器
├── tests/                        # 测试代码
│   ├── performance/             # 性能测试
│   │   ├── baseline_performance_test.py
│   │   ├── compare_performance.py
│   │   └── real_e2e_latency_test.py
│   ├── functionality/           # 功能测试
│   │   ├── test_all_extended_tools.py
│   │   ├── test_enhanced_weather.py
│   │   ├── test_realtime_accuracy.py
│   │   ├── test_web_search.py
│   │   ├── wake_word_precision_test.py
│   │   └── wake_word_precision_test_per_snr.py
│   └── rigorous/               # 严格测试框架
│       ├── rigorous_interruption_test.py
│       ├── rigorous_latency_test.py
│       ├── rigorous_test_manager.py
│       └── rigorous_wakeword_test.py
├── data/                         # 测试数据和结果
│   ├── experimental/            # 实验数据
│   │   ├── rigorous_latency_data_20250930_091753.csv
│   │   ├── rigorous_wakeword_data_20250929_032437.csv
│   │   ├── rigorous_interruption_data_20250929_032957.csv
│   │   └── *.json (原始数据和摘要)
│   ├── baseline/               # 基准测试数据
│   │   ├── short_baseline_results_20250930_033709.csv
│   │   └── short_baseline_results_20250930_033709.json
│   └── reports/                # 测试报告
│       ├── rigorous_*_report_*.txt
│       └── short_baseline_*_report.txt
├── figures/                      # 图片和可视化
│   ├── scripts/                # 图片生成脚本
│   │   ├── create_enhanced_figures.py
│   │   ├── create_figures_4_5.py
│   │   ├── create_report_figures.py
│   │   └── figure_summary.py
│   └── output/                 # 生成的图片
│       ├── Figure2_Enhanced_Latency_Performance.png/pdf
│       ├── Figure3_Enhanced_Wake_Word_Performance.png/pdf
│       ├── Figure4_Enhanced_VAD_Performance.png/pdf
│       └── Figure5_Enhanced_Interruption_Performance.png/pdf
├── config/                       # 配置文件
│   ├── qwen_tts_config.env
│   ├── ten_vad_config_guide.env
│   └── tts_config.env
├── docs/                        # 文档
│   ├── BASELINE_TESTING_GUIDE.md
│   ├── DESIGN_PROCESS_IMPROVEMENTS.md
│   ├── OPENAI_REALTIME_USAGE_GUIDE.md
│   ├── REPORT_DATA_UPDATE_SUMMARY.md
│   ├── RIGOROUS_TESTING_GUIDE.md
│   ├── TEN_VAD_USAGE_GUIDE.md
│   ├── TTS_PERFORMANCE_GUIDE.md
│   └── UWA_RAG_USAGE_GUIDE.md
└── README.md                     # 主要说明文档
```

## 使用方法

### 1. 运行脚本
使用便捷运行器从项目根目录运行任何脚本：
```bash
# 运行图片生成脚本
./scripts/run_from_root.sh figures/scripts/create_enhanced_figures.py

# 运行基准测试
./scripts/run_from_root.sh scripts/test/run_baseline_test.sh --samples 10

# 启动服务
./scripts/run_from_root.sh scripts/start/start_llm.sh
```

### 2. 或者直接切换到相应目录运行
```bash
# 运行图片生成
cd figures/scripts && python3 create_enhanced_figures.py

# 运行测试
cd scripts/test && bash run_baseline_test.sh --samples 10
```

## 路径修复说明

1. **图片生成脚本** (`figures/scripts/`):
   - 数据文件路径更新为 `../../data/experimental/` 和 `../../data/baseline/`
   - 输出路径更新为 `../output/`

2. **测试脚本** (`scripts/test/`):
   - Python测试文件路径更新为 `../../tests/performance/`

3. **所有脚本** 现在都假设从各自的目录运行，路径已相应调整

## 优势

- 🗂️ **清晰的结构**: 不同类型的文件分类存放
- 🔍 **易于查找**: 相关文件集中在对应文件夹
- 🚀 **易于运行**: 便捷运行器支持从任何位置运行脚本
- 📝 **专业标准**: 符合软件开发的最佳实践
- 🔄 **版本控制友好**: 清晰的结构便于git管理