# 🔧 语音助手技术实现细节

本文档补充了具体的代码实现细节和技术决策说明。

## 📝 核心代码变更总览

### 1. TTS节点优化 (`openai_tts_node.py`)

#### 关键变更
```python
# 修复线程池命名冲突 (第50行)
- self.executor = ThreadPoolExecutor(max_workers=2)
+ self.thread_executor = ThreadPoolExecutor(max_workers=2)

# 使用最新API配置 (第52-56行)
self.tts_model = os.environ.get("TTS_MODEL", "gpt-4o-mini-tts")
self.tts_voice = os.environ.get("TTS_VOICE", "coral")
self.tts_format = os.environ.get("TTS_FORMAT", "wav")
self.tts_speed = float(os.environ.get("TTS_SPEED", "1.1"))

# 异步TTS调用 (第92行)
- self.executor.submit(self.call_openai_tts_async, text)
+ self.thread_executor.submit(self.call_openai_tts_async, text)
```

#### 播放器改进
```python
def play_audio_with_pygame(self, audio_content: bytes):
    """改进的pygame音频播放器"""
    try:
        # 确保清理之前的音频状态
        pygame.mixer.quit()
        pygame.mixer.init(frequency=22050, size=-16, channels=2, buffer=512)
        
        # 创建临时文件
        with tempfile.NamedTemporaryFile(delete=False, suffix=f'.{self.tts_format}') as temp_file:
            temp_file.write(audio_content)
            temp_filename = temp_file.name
        
        # 播放音频
        pygame.mixer.music.load(temp_filename)
        pygame.mixer.music.play()
        
        # 等待播放完成
        while pygame.mixer.music.get_busy():
            time.sleep(0.1)
            
    finally:
        # 清理资源
        if temp_filename and os.path.exists(temp_filename):
            os.unlink(temp_filename)
        pygame.mixer.quit()
```

### 2. STT实时节点 (`realtime_stt_node.py`)

#### VAD实现核心
```python
class RealtimeSTTNode(Node):
    def __init__(self):
        super().__init__('realtime_stt_node')
        
        # VAD参数配置
        self.rms_threshold = 1000          # 初始RMS阈值
        self.silence_duration = 2.0        # 静音判定时间
        self.min_speech_duration = 0.5     # 最小语音长度
        self.max_speech_duration = 10.0    # 最大语音长度
        
        # 音频参数
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        
    def is_speech(self, audio_chunk):
        """基于RMS的语音活动检测"""
        audio_data = np.frombuffer(audio_chunk, dtype=np.int16)
        rms = np.sqrt(np.mean(audio_data**2))
        
        # 动态阈值调整
        if rms < self.rms_threshold * 0.3:
            self.rms_threshold *= 0.95  # 逐渐降低阈值
        elif rms > self.rms_threshold * 2:
            self.rms_threshold *= 1.05  # 逐渐提高阈值
            
        return rms > self.rms_threshold
```

#### 语音缓冲管理
```python
def process_audio_stream(self):
    """智能音频流处理"""
    audio_buffer = []
    silence_count = 0
    speech_detected = False
    
    while not self.stop_recording:
        try:
            data = self.stream.read(self.chunk, exception_on_overflow=False)
            
            if self.is_speech(data):
                if not speech_detected:
                    self.get_logger().info("🎤 检测到语音开始")
                    speech_detected = True
                    
                audio_buffer.append(data)
                silence_count = 0
            else:
                if speech_detected:
                    silence_count += 1
                    audio_buffer.append(data)  # 保留一些静音用于自然结束
                    
                    # 检查是否达到静音阈值
                    if silence_count > (self.silence_duration * self.rate / self.chunk):
                        self.submit_audio_for_recognition(audio_buffer)
                        audio_buffer = []
                        speech_detected = False
                        silence_count = 0
                        
        except Exception as e:
            self.get_logger().error(f"音频处理错误: {e}")
```

### 3. LLM智能对话系统 (`llm_node.py`)

#### 🧠 对话上下文管理
```python
class LLMNode(Node):
    def __init__(self):
        # 初始化对话历史存储
        self.conversation_history: list[dict[str, str]] = []  # 存储用户和助手的对话
        self.max_history_messages = 20   # 保持最近20条消息 (10轮对话)
        
        # 初始化RAG知识库
        try:
            self.knowledge_base = UWAKnowledgeBase()
            self.get_logger().info("✅ RAG Knowledge Base initialized successfully")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize Knowledge Base: {e}")
            self.knowledge_base = None

    def call_chatgpt_with_rag(self, prompt: str) -> str:
        """增强版ChatGPT调用，集成RAG和对话历史"""
        
        # 步骤1: 搜索相关知识
        search_results = self.search_knowledge_base(prompt, n_results=3)
        rag_context = self.format_rag_context(search_results)
        
        # 步骤2: 构建包含历史的消息列表
        messages = [{"role": "system", "content": system_prompt}]
        
        # 添加最近的对话历史 (控制token使用)
        if self.conversation_history:
            messages.extend(self.conversation_history[-self.max_history_messages:])
        
        # 添加当前用户消息
        messages.append({"role": "user", "content": prompt})
        
        # 步骤3: 调用OpenAI API
        response = self.client.chat.completions.create(
            model="ft:gpt-4.1-mini-2025-04-14:personal:my-voice-assistant:BxxCKJUa",
            messages=messages,
            temperature=0.7,
            max_tokens=1024,
            stream=True
        )
        
        # 步骤4: 更新对话历史
        self.conversation_history.append({"role": "user", "content": prompt})
        self.conversation_history.append({"role": "assistant", "content": final_filtered})
        
        # 步骤5: 维护历史长度
        if len(self.conversation_history) > self.max_history_messages:
            excess = len(self.conversation_history) - self.max_history_messages
            self.conversation_history = self.conversation_history[excess:]
```

#### 🔍 RAG知识检索系统
```python
def search_knowledge_base(self, query: str, n_results: int = 3):
    """搜索UWA知识库获取相关信息"""
    if not self.knowledge_base:
        return []
        
    try:
        results = self.knowledge_base.search(query, n_results=n_results)
        
        if results:
            self.get_logger().info(f"🔍 Found {len(results)} relevant knowledge entries")
            for i, result in enumerate(results):
                self.get_logger().debug(f"  {i+1}. [{result['metadata']['category']}] "
                                      f"{result['content'][:50]}... (distance: {result['distance']:.3f})")
        return results
        
    except Exception as e:
        self.get_logger().error(f"❌ Knowledge search error: {e}")
        return []

def format_rag_context(self, search_results: list) -> str:
    """将搜索结果格式化为LLM上下文"""
    if not search_results:
        return ""
        
    context_parts = ["📚 RELEVANT UWA INFORMATION:"]
    
    for i, result in enumerate(search_results, 1):
        building = result['metadata'].get('building', 'Unknown')
        category = result['metadata'].get('category', 'general')
        content = result['content']
        
        context_parts.append(f"{i}. [{category.upper()}] {content}")
        if building != 'Campus General' and building != 'unknown':
            context_parts[-1] += f" (Located: {building})"
    
    context_parts.append("\nPlease use this information to provide accurate, helpful responses about UWA.")
    
    return "\n".join(context_parts)
```

#### 🧹 内容过滤器详细实现
```python
def filter_content(self, text: str) -> str:
    """多层次内容过滤器"""
    if not text:
        return text
        
    original_length = len(text)
    
    # 第一层：移除【】括号内容
    # 匹配模式：【任何非】字符】
    filtered_text = re.sub(r'【[^】]*】', '', text)
    
    # 第二层：移除数字引用模式
    # 移除3位以上连续数字
    filtered_text = re.sub(r'\b\d{3,}\b', '', filtered_text)
    
    # 第三层：移除特殊符号
    # † ‑ 等特殊引用符号
    filtered_text = re.sub(r'[†‑]+', '', filtered_text)
    
    # 第四层：移除行号引用
    # L数字-L数字 模式
    filtered_text = re.sub(r'L\d+-L\d+', '', filtered_text)
    
    # 第五层：清理格式
    # 合并多个空格
    filtered_text = re.sub(r'\s+', ' ', filtered_text)
    # 清理连续句号
    filtered_text = re.sub(r'\s*\.\s*\.+', '.', filtered_text)
    # 移除首尾空白
    filtered_text = filtered_text.strip()
    
    # 记录过滤效果
    if len(filtered_text) != original_length:
        self.get_logger().info(f"🧹 Content filtered: {original_length} -> {len(filtered_text)} chars")
    
    return filtered_text
```

#### 🎯 智能功能特点总结

**对话上下文管理**:
- ✅ 自动维护最近20条消息历史
- ✅ 智能token管理，避免超出API限制
- ✅ 对话历史自动截断和更新
- ✅ 支持多轮连续对话，保持上下文连贯性

**RAG知识增强**:
- ✅ 集成UWA校园知识库（44+文档）
- ✅ 实时知识检索，相关度评分排序
- ✅ 动态上下文注入，提升回答准确性  
- ✅ 支持建筑物位置、服务时间等具体信息查询

**内容质量保证**:
- ✅ 多层过滤算法，自动清理LLM输出噪声
- ✅ 实时内容过滤，保证用户体验
- ✅ 过滤效果跟踪和日志记录
- ✅ 流式响应处理，降低延迟感知

#### 🎥 用户交互增强
```python
def listener_callback(self, msg: String):
    """增强版用户输入处理"""
    input_text = msg.data.strip()
    if not input_text:
        return

    # 清晰显示用户输入
    print("\n" + "="*60)
    print("🎤 用户语音输入:")
    print("-"*60)
    print(f"'{input_text}'")
    print("="*60)
    
    # 显示RAG搜索结果
    search_results = self.search_knowledge_base(input_text, n_results=3)
    if search_results:
        print("\n" + "="*60)
        print("🔍 RAG 知识库搜索结果:")
        print("-"*60)
        for i, result in enumerate(search_results, 1):
            category = result['metadata']['category']
            building = result['metadata']['building']
            content = result['content'][:80] + "..." if len(result['content']) > 80 else result['content']
            distance = result['distance']
            
            print(f"{i}. [{category}] {content}")
            if building not in ['Campus General', 'unknown']:
                print(f"   📍 位置: {building}")
            print(f"   📊 相关度: {(1-distance)*100:.1f}%")
        print("="*60)
    
    # 显示完整LLM回答
    print("\n" + "="*60)
    print("🤖 LLM 完整回答:")
    print("-"*60)
    print(final_filtered)
    print("="*60 + "\n")
```

### 4. 实时语音识别+VAD (`realtime_stt_node.py`)

#### VAD语音活动检测
```python
def process_audio_chunk(self, audio_data):
    """处理音频块并检测语音活动"""
    # 计算RMS音量
    rms = np.sqrt(np.mean(audio_data**2))
    
    # 动态阈值检测
    if rms > self.vad_threshold:
        self.is_speaking = True
        self.speech_buffer.extend(audio_data)
        self.silence_counter = 0
    else:
        self.silence_counter += 1
        
        # 语音结束检测
        if self.is_speaking and self.silence_counter > self.silence_limit:
            self.process_speech_segment()
            self.is_speaking = False
```

## 🧪 测试验证方法

### 1. 性能测试脚本分析

#### `test_tts_performance.sh` 关键部分
```bash
# 动态测试函数
test_tts_config() {
    local model=$1
    local voice=$2
    local format=$3
    local speed=$4
    local description=$5
    
    # 设置测试环境
    export TTS_MODEL=$model
    export TTS_VOICE=$voice
    export TTS_FORMAT=$format
    export TTS_SPEED=$speed
    
    # 生成Python测试脚本
    cat > temp_tts_test.py << EOF
#!/usr/bin/env python3
import time
import os
from openai import OpenAI

client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))

start_time = time.time()
response = client.audio.speech.create(
    model="$model",
    voice="$voice", 
    input="$TEST_TEXT",
    response_format="$format",
    speed=$speed
)
generation_time = time.time() - start_time
print(f"生成时间: {generation_time:.2f} 秒")
EOF
    
    # 执行测试
    python3 temp_tts_test.py
    rm -f temp_tts_test.py
}
```

### 2. 内容过滤测试

#### 测试用例设计
```python
test_cases = [
    # 基本过滤测试
    "正常文本【需要过滤的内容】保留文本",
    
    # 复杂引用过滤
    "文本内容【742442319583238†L62-L65】继续文本",
    
    # 多重过滤
    "开始【第一个】中间【第二个】结束",
    
    # 边界情况
    "【】空括号测试",
    "【嵌套【内容】测试】",
    
    # 数字和符号
    "包含12345大数字和†‑特殊符号",
]
```

### 3. 集成测试流程

#### 完整测试序列
```bash
#!/bin/bash
echo "🧪 开始完整系统测试"

# 1. 环境检查
test_environment() {
    echo "检查环境变量..."
    [ -z "$OPENAI_API_KEY" ] && echo "❌ API密钥缺失" && return 1
    echo "✅ 环境检查通过"
}

# 2. 组件测试
test_components() {
    echo "测试各组件..."
    
    # TTS测试
    ./test_tts_simple.sh || return 1
    
    # 内容过滤测试
    python3 test_content_filter.py || return 1
    
    echo "✅ 组件测试通过"
}

# 3. 性能验证
test_performance() {
    echo "性能基准测试..."
    ./test_tts_performance.sh | grep "生成时间" | \
    awk -F: '{print $2}' | awk '{if($1 > 3.0) exit 1}'
    
    [ $? -eq 0 ] && echo "✅ 性能达标" || echo "❌ 性能不达标"
}

# 4. 集成测试
test_integration() {
    echo "ROS2集成测试..."
    
    # 启动节点
    ./start_tts_fast.sh &
    TTS_PID=$!
    sleep 3
    
    # 发送测试消息
    ros2 topic pub /llm_response std_msgs/String \
        "data: 'Hello【test123】world'" --once
    
    # 清理
    kill $TTS_PID 2>/dev/null
    echo "✅ 集成测试完成"
}

# 执行测试序列
test_environment && \
test_components && \
test_performance && \
test_integration

echo "🏁 系统测试完成"
```

## ⚙️ 配置管理

### 环境变量优先级
```bash
# 1. 系统环境变量 (最高优先级)
export TTS_MODEL=gpt-4o-mini-tts

# 2. .env 文件配置
TTS_MODEL=gpt-4o-mini-tts

# 3. 代码默认值 (最低优先级)
self.tts_model = os.environ.get("TTS_MODEL", "tts-1")
```

### 动态配置更新
```python
class ConfigManager:
    def __init__(self):
        self.config_file = ".env"
        self.load_config()
        
    def load_config(self):
        """从.env文件加载配置"""
        if os.path.exists(self.config_file):
            load_dotenv(self.config_file)
            
    def update_config(self, key: str, value: str):
        """动态更新配置"""
        os.environ[key] = value
        self.save_to_file(key, value)
        
    def save_to_file(self, key: str, value: str):
        """保存配置到文件"""
        # 实现配置持久化
        pass
```

## 🔍 故障诊断指南

### 常见问题排查

#### 1. TTS响应慢
```bash
# 检查模型配置
echo $TTS_MODEL  # 应该是 gpt-4o-mini-tts

# 检查格式配置  
echo $TTS_FORMAT  # 推荐 opus 或 wav

# 检查网络延迟
curl -w "@curl-format.txt" -o /dev/null -s "https://api.openai.com/v1/models"
```

#### 2. 内容过滤失效
```python
# 测试过滤器
def test_filter():
    test_text = "Hello【test123】world"
    result = filter_content(test_text)
    assert "【" not in result, "过滤器失效"
    print(f"过滤测试: '{test_text}' -> '{result}'")
```

#### 3. STT VAD敏感度
```python
# 调整VAD参数
def tune_vad():
    # 环境噪音大时提高阈值
    self.rms_threshold = 1500
    
    # 安静环境时降低阈值
    self.rms_threshold = 800
    
    # 动态调整
    self.adaptive_threshold = True
```

## 📊 性能监控

### 关键指标
- **TTS延迟**: < 2.5秒
- **内存使用**: < 500MB
- **CPU使用**: < 50%
- **错误率**: < 1%

### 监控实现
```python
class PerformanceMonitor:
    def __init__(self):
        self.metrics = {
            'tts_latency': [],
            'memory_usage': [],
            'error_count': 0
        }
    
    def record_tts_latency(self, latency: float):
        self.metrics['tts_latency'].append(latency)
        if latency > 3.0:
            self.alert_slow_tts(latency)
    
    def get_average_latency(self) -> float:
        latencies = self.metrics['tts_latency']
        return sum(latencies) / len(latencies) if latencies else 0
```

---

*技术实现文档 v1.0*  
*更新时间: 2025年7月27日*
