# 🎓 UWA RAG 智能语音助手使用指南

## 📋 概述

您的 UWA 语音助手现在已经配备了先进的 **RAG (Retrieval-Augmented Generation)** 技术，能够提供准确的校园信息！

### 🎯 主要特性
- **智能知识检索**: 44+ 条 UWA 校园信息
- **实时信息查询**: 建筑开放时间、交通路线、服务位置
- **中文友好**: 支持中英文混合查询
- **高准确性**: 基于官方信息源，避免AI幻觉

## 🚀 快速开始

### 1. 启动 RAG 增强的 LLM 节点
```bash
cd /workspaces/ros2_ws
./start_llm_rag.sh
```

### 2. 测试知识库功能
```bash
# 快速测试
cd /workspaces/ros2_ws/src/my_voice_assistant/my_voice_assistant
python3 simple_rag_test.py

# 完整测试
python3 test_rag_integration.py
```

## 📚 支持的查询类型

### 🏢 建筑与设施
- *"图书馆几点开门？"*
- *"Student Central 在哪里？"*
- *"医疗中心在哪个楼？"*
- *"哪里可以买教科书？"*

### 🚌 交通出行
- *"怎么坐公交车到UWA？"*
- *"有哪些公交路线？"*
- *"校园内怎么停车？"*
- *"从火车站怎么到校园？"*

### 🍽️ 餐饮服务
- *"校园内哪里可以吃饭？"*
- *"有什么食物选择？"*
- *"咖啡厅在哪里？"*
- *"Food court 什么时候开？"*

### 🎓 学术支持
- *"哪里可以得到学习帮助？"*
- *"写作中心在哪？"*
- *"怎么预约图书馆小组室？"*
- *"有免费辅导吗？"*

### 🏥 健康服务
- *"医疗中心开放时间？"*
- *"药房在哪里？"*
- *"心理咨询服务怎么预约？"*
- *"急救电话是多少？"*

### 🏠 住宿信息
- *"校内宿舍有哪些？"*
- *"校外租房推荐区域？"*
- *"住宿费用大概多少？"*

### 🎯 学生生活
- *"有什么学生社团？"*
- *"体育设施有什么？"*
- *"Orientation Week 是什么时候？"*

### 💻 技术支持
- *"WiFi 怎么连接？"*
- *"电脑实验室在哪？"*
- *"IT帮助台在哪里？"*

## 🔧 系统架构

### RAG 工作流程
```
语音输入 → STT → 知识库搜索 → 上下文增强 → LLM生成 → TTS → 语音输出
```

### 知识库统计
- **总文档数**: 44+
- **覆盖类别**: 14个 (学术、交通、餐饮、住宿等)
- **信息源**: UWA官方指南和新生手册
- **更新频率**: 可实时添加新信息

## 📊 性能优势

| 功能 | 传统LLM | RAG增强LLM |
|------|---------|------------|
| 信息准确性 | 可能过时/错误 | ✅ 基于最新官方信息 |
| 具体细节 | 泛泛而谈 | ✅ 精确时间/地点/电话 |
| 本地化程度 | 通用回答 | ✅ UWA专属信息 |
| 幻觉风险 | 高 | ✅ 显著降低 |

## 🛠️ 高级功能

### 1. 添加自定义信息
```python
from uwa_knowledge_base import UWAKnowledgeBase

kb = UWAKnowledgeBase()
kb.add_custom_info(
    content="新增的校园信息",
    category="custom",
    building="具体建筑"
)
```

### 2. 分类搜索
```python
# 只搜索交通相关信息
results = kb.search("公交", category_filter="transport")
```

### 3. 知识库统计
```python
stats = kb.get_stats()
print(f"总文档: {stats['total_documents']}")
print(f"分类: {stats['categories']}")
```

## 🔍 故障排除

### 常见问题

1. **找不到相关信息**
   - 尝试使用不同的关键词
   - 检查拼写是否正确
   - 确认查询的是UWA相关内容

2. **响应不够准确**
   - 可以要求更具体的信息
   - 提供更多上下文
   - 尝试重新表述问题

3. **RAG系统启动失败**
   ```bash
   # 检查依赖
   pip install chromadb sentence-transformers
   
   # 重新初始化知识库
   python3 expand_knowledge_base.py
   ```

### 调试命令
```bash
# 🆕 使用新的管理工具 (推荐)
python3 manage_knowledge_base.py stats    # 查看知识库统计
python3 manage_knowledge_base.py test     # 测试知识库功能
python3 manage_knowledge_base.py reset    # 重置知识库
python3 manage_knowledge_base.py expand   # 扩展知识库
python3 manage_knowledge_base.py backup   # 备份知识库

# 传统命令 (仍然可用)
python3 simple_rag_test.py               # 测试知识库连接
./start_llm_rag.sh --debug               # 查看详细日志

# 完全重建知识库
python3 -c "from uwa_knowledge_base import UWAKnowledgeBase; kb = UWAKnowledgeBase(); kb.delete_collection()"
```

## 📝 知识库管理指南

### 🔧 知识库更新方法

#### 方法1: 使用管理脚本 (推荐)
```bash
# 查看当前状态
python3 manage_knowledge_base.py stats

# 扩展知识库内容
python3 manage_knowledge_base.py expand

# 测试功能
python3 manage_knowledge_base.py test
```

#### 方法2: 添加自定义内容
```bash
# 编辑新数据模板
nano add_new_knowledge.py

# 添加到知识库
python3 add_new_knowledge.py

# 查看模板指南
python3 add_new_knowledge.py guide
```

#### 方法3: 完全重构
```bash
# 备份现有数据
python3 manage_knowledge_base.py backup

# 重置知识库
python3 manage_knowledge_base.py reset

# 重新扩展内容
python3 manage_knowledge_base.py expand
```

#### 方法4: 单条信息快速添加
```python
from uwa_knowledge_base import UWAKnowledgeBase
kb = UWAKnowledgeBase()
kb.add_custom_info(
    content="新的校园信息",
    category="academic",
    building="Building 25"
)
```

### 📋 知识库文件说明

| 文件 | 用途 | 何时使用 |
|------|------|----------|
| `uwa_knowledge_base.py` | 核心知识库类 | 作为其他脚本的基础库 |
| `expand_knowledge_base.py` | 批量扩展数据 | 添加大量结构化内容 |
| `manage_knowledge_base.py` | 🆕 管理工具 | 日常管理和维护 |
| `add_new_knowledge.py` | 🆕 新数据模板 | 添加少量自定义内容 |

## 🚀 使用示例

### 典型对话流程
```
用户: "图书馆什么时候开门？"
系统: [搜索知识库] → [找到相关信息] → [生成回答]
助手: "UWA主图书馆Reid Library开放时间是周一到周四早8点到晚10点，周五早8点到晚6点，周六早9点到下午5点，周日上午11点到晚10点。另外，Science Library在物理楼，开放时间是周一到周五早8点到晚6点。"
```

### 系统内部处理
1. **语音识别**: "图书馆什么时候开门？"
2. **知识检索**: 搜索 "library", "开放时间", "时间"
3. **上下文构建**: 组合相关的图书馆信息
4. **LLM生成**: 基于检索到的准确信息生成回答
5. **语音合成**: 转换为自然语音输出

## 📈 未来扩展

### 计划中的功能
- [ ] **多语言支持**: 支持更多语言查询
- [ ] **实时更新**: 自动获取最新校园信息  
- [ ] **个性化**: 基于用户专业定制信息
- [ ] **位置感知**: 结合GPS提供路线指导
- [ ] **事件提醒**: 重要日期和截止时间提醒

### 知识库扩展
- [ ] 课程信息和选课指导
- [ ] 教授办公时间和联系方式
- [ ] 考试安排和成绩查询
- [ ] 奖学金和就业信息
- [ ] 校园活动和社团招新

## 🎉 总结

您的 UWA 语音助手现在具备了：

✅ **企业级RAG技术** - 准确可靠的信息检索  
✅ **44+条专业知识** - 覆盖新生所需的各类信息  
✅ **中英文支持** - 适合国际学生使用  
✅ **实时响应** - 快速准确的语音交互  
✅ **可扩展架构** - 支持持续添加新信息  

无论是找路线、查时间、找服务还是了解校园生活，您的智能助手都能提供准确、及时的帮助！

---

*UWA RAG 智能语音助手 v2.0*  
*技术栈: ROS2 + OpenAI + ChromaDB + SentenceTransformers*  
*更新日期: 2025年7月28日*
