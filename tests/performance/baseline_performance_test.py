#!/usr/bin/env python3
"""
Baseline Performance Test for Conventional STT → LLM → TTS Pipeline
================================================================

This script tests                # 等待测试完成
                timeout = 120  # 120秒超时（baseline pipeline需要更长时间）
                start_wait = time.time()
                while len(self.results) <= i and (time.time() - start_wait) < timeout:
                    rclpy.spin_once(self, timeout_sec=0.1)conventional pipeline using:
- openai_stt_node (STT)
- llm_node (LLM processing)  
- openai_tts_node (TTS)

Measures end-to-end latency and compares with OpenAI Realtime approach.

Usage:
    python3 baseline_performance_test.py --samples 30 --output baseline_results.json
"""

import argparse
import json
import csv
import time
import statistics
import numpy as np
from datetime import datetime, timedelta
from pathlib import Path
import subprocess
import threading
import queue
import sys
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class BaselineLatencyTest(Node):
    def __init__(self, num_samples=30):
        super().__init__('baseline_latency_test')
        self.num_samples = num_samples
        self.results = []
        self.current_test = {}
        self.test_queue = queue.Queue()
        self.measurement_lock = threading.Lock()
        
        # Test prompts (same as rigorous test for consistency)
        self.test_prompts = [
            "new way 4, what's the weather like today?",
            "new way 4, when does the next shuttle arrive?", 
            "new way 4, show me the campus dining options",
            "new way 4, what time does the library close?",
            "new way 4, how do I get to the engineering building?",
            "new way 4, what events are happening on campus?",
            "new way 4, where can I find available parking?",
            "new way 4, what's the shuttle schedule for tomorrow?",
            "new way 4, tell me about study spaces",
            "new way 4, how late is the cafeteria open?",
            "new way 4, what's the current campus occupancy?",
            "new way 4, show me the quickest route to the library",
            "new way 4, what health services are available?",
            "new way 4, when is the next campus tour?",
            "new way 4, where are the computer labs located?",
        ]
        
        # Publishers and subscribers for conventional pipeline
        self.speech_pub = self.create_publisher(String, 'speech_text', 10)
        self.llm_response_sub = self.create_subscription(
            String, 'llm_response', self.llm_response_callback, 10)
        self.tts_status_sub = self.create_subscription(
            Bool, 'tts_status', self.tts_status_callback, 10)
        self.llm_response_full_sub = self.create_subscription(
            String, 'llm_response_full', self.llm_response_full_callback, 10)
        
        # Timing variables
        self.test_start_time = None
        self.first_response_time = None
        self.tts_start_time = None
        self.tts_end_time = None
        self.current_test = {}
        self.current_test_id = 0
        self.test_completed = False
        
        self.get_logger().info('Baseline Latency Test Node initialized')

    def speech_callback(self, msg):
        """记录语音输入时间"""
        if self.test_start_time is None:
            self.test_start_time = time.time()
            self.get_logger().info(f'Speech input detected: {msg.data[:50]}...')

    def llm_response_callback(self, msg):
        """记录第一个LLM响应时间"""
        # 验证这个响应是否属于当前测试
        if self.test_completed or self.test_start_time is None:
            self.get_logger().warning(f'Ignoring late/invalid LLM response for test {self.current_test_id}: {msg.data[:30]}...')
            return
            
        # 检查响应时间是否合理（不应该太快）
        if self.test_start_time is not None:
            response_time = (time.time() - self.test_start_time) * 1000
            if response_time < 100:  # 少于100ms的响应不合理
                self.get_logger().warning(f'Suspiciously fast LLM response ({response_time:.1f}ms), ignoring: {msg.data[:30]}...')
                return
            
        if self.first_response_time is None:
            self.first_response_time = time.time()
            self.get_logger().info(f'First LLM response: {msg.data[:50]}...')

    def llm_response_full_callback(self, msg):
        """记录完整LLM响应"""
        if self.test_start_time is not None:
            self.get_logger().info('Full LLM response received')

    def tts_status_callback(self, msg):
        """记录TTS状态变化"""
        # 验证这个事件是否属于当前测试
        if self.test_completed or self.test_start_time is None:
            self.get_logger().warning(f'Ignoring late/invalid TTS event for test {self.current_test_id}')
            return
            
        current_time = time.time()
        if msg.data:  # TTS started
            # 检查TTS开始时间是否合理（不应该太快）
            if self.test_start_time is not None:
                tts_start_time = (current_time - self.test_start_time) * 1000
                if tts_start_time < 100:  # 少于100ms的TTS开始不合理
                    self.get_logger().warning(f'Suspiciously fast TTS start ({tts_start_time:.1f}ms), ignoring')
                    return
                    
            if self.tts_start_time is None:
                self.tts_start_time = current_time
                self.get_logger().info('TTS playback started')
        else:  # TTS ended
            if self.tts_end_time is None and self.tts_start_time is not None:
                self.tts_end_time = current_time
                self.get_logger().info('TTS playback ended')
                self.complete_measurement()
                self.test_completed = True

    def complete_measurement(self):
        """完成一次测量并记录结果"""
        with self.measurement_lock:
            if self.test_start_time is None:
                return
                
            try:
                # 计算各种延迟指标
                speech_to_first_response = None
                speech_to_tts_start = None
                total_response_time = None
                
                if self.first_response_time:
                    speech_to_first_response = (self.first_response_time - self.test_start_time) * 1000
                    
                if self.tts_start_time:
                    speech_to_tts_start = (self.tts_start_time - self.test_start_time) * 1000
                    
                if self.tts_end_time:
                    total_response_time = (self.tts_end_time - self.test_start_time) * 1000

                measurement = {
                    'test_id': len(self.results) + 1,
                    'timestamp': datetime.now().isoformat(),
                    'speech_to_first_response_ms': speech_to_first_response,
                    'speech_to_tts_start_ms': speech_to_tts_start,
                    'total_response_time_ms': total_response_time,
                    'prompt': self.current_test.get('prompt', 'unknown'),
                    'success': True,
                    'timeout': False
                }
                
                self.results.append(measurement)
                self.get_logger().info(f'Measurement {len(self.results)} completed:')
                
                # Safe formatting - only show values that are not None
                if speech_to_first_response is not None:
                    self.get_logger().info(f'  Speech→First Response: {speech_to_first_response:.1f}ms')
                else:
                    self.get_logger().info(f'  Speech→First Response: Not measured')
                    
                if speech_to_tts_start is not None:
                    self.get_logger().info(f'  Speech→TTS Start: {speech_to_tts_start:.1f}ms')
                else:
                    self.get_logger().info(f'  Speech→TTS Start: Not measured')
                    
                if total_response_time is not None:
                    self.get_logger().info(f'  Total Response: {total_response_time:.1f}ms')
                else:
                    self.get_logger().info(f'  Total Response: Not measured')
                
            except Exception as e:
                self.get_logger().error(f'Error completing measurement: {e}')
            finally:
                self.reset_measurement()

    def reset_measurement(self):
        """重置测量变量"""
        self.test_start_time = None
        self.first_response_time = None
        self.tts_start_time = None
        self.tts_end_time = None
        self.current_test = {}
        self.test_completed = False

    def run_test_sequence(self):
        """运行测试序列"""
        self.get_logger().info(f'Starting baseline test with {self.num_samples} samples')
        
        for i in range(self.num_samples):
            try:
                # 强制等待，确保上一个测试的所有异步操作完成
                if i > 0:
                    self.get_logger().info(f'Waiting for previous test cleanup...')
                    time.sleep(5.0)  # 简单等待，避免复杂的ROS2 spin操作
                
                # 选择测试提示词
                prompt = self.test_prompts[i % len(self.test_prompts)]
                self.current_test = {'prompt': prompt}
                self.current_test_id = i + 1
                
                self.get_logger().info(f'Test {i+1}/{self.num_samples}: {prompt[:50]}...')
                
                # 重置测量状态
                self.reset_measurement()
                
                # 发送语音输入
                self.test_start_time = time.time()
                msg = String()
                msg.data = prompt
                self.speech_pub.publish(msg)
                
                # 等待测量完成或超时
                timeout = 120  # 120秒超时（baseline pipeline需要更长时间）
                start_wait = time.time()
                while not self.test_completed and (time.time() - start_wait) < timeout:
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                if not self.test_completed:
                    self.get_logger().warning(f'Test {i+1} timed out after {timeout}s')
                    self.results.append({
                        'test_id': i + 1,
                        'timestamp': datetime.now().isoformat(),
                        'speech_to_first_response_ms': None,
                        'speech_to_tts_start_ms': None,
                        'total_response_time_ms': None,
                        'prompt': prompt,
                        'success': False,
                        'timeout': True
                    })
                    self.test_completed = True
                
                # 测试间隔
                time.sleep(2.0)
                
            except KeyboardInterrupt:
                self.get_logger().info('Test interrupted by user')
                break
            except Exception as e:
                self.get_logger().error(f'Error in test {i+1}: {e}')
                continue

    def calculate_statistics(self):
        """计算统计结果"""
        if not self.results:
            return {}
            
        # 过滤有效数据
        valid_results = [r for r in self.results if not r.get('timeout', False)]
        
        stats = {
            'total_samples': len(self.results),
            'valid_samples': len(valid_results),
            'timeout_count': len(self.results) - len(valid_results)
        }
        
        if not valid_results:
            return stats
            
        # 计算各项指标的统计数据
        metrics = ['speech_to_first_response_ms', 'speech_to_tts_start_ms', 'total_response_time_ms']
        
        for metric in metrics:
            values = [r[metric] for r in valid_results if r[metric] is not None]
            if values:
                stats[metric] = {
                    'count': len(values),
                    'mean': statistics.mean(values),
                    'median': statistics.median(values),
                    'std': statistics.stdev(values) if len(values) > 1 else 0,
                    'min': min(values),
                    'max': max(values),
                    'p95': np.percentile(values, 95) if len(values) > 1 else values[0],
                    'confidence_interval_95': self.calculate_ci(values, 0.95) if len(values) > 10 else None
                }
        
        return stats

    def calculate_ci(self, data, confidence=0.95):
        """计算置信区间"""
        if len(data) < 2:
            return None
            
        n = len(data)
        mean = statistics.mean(data)
        std_err = statistics.stdev(data) / (n ** 0.5)
        
        # 使用t分布（简化版本）
        t_value = 2.0 if n > 30 else 2.5  # 简化的t值
        margin = t_value * std_err
        
        return [mean - margin, mean + margin]

    def save_results(self, output_file):
        """保存测试结果"""
        stats = self.calculate_statistics()
        
        output_data = {
            'test_info': {
                'timestamp': datetime.now().isoformat(),
                'test_type': 'baseline_conventional_pipeline',
                'architecture': 'STT → LLM → TTS',
                'num_samples': self.num_samples,
                'description': 'Baseline performance test for conventional STT→LLM→TTS pipeline'
            },
            'statistics': stats,
            'raw_results': self.results
        }
        
        # 保存JSON结果
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(output_data, f, indent=2, ensure_ascii=False)
        
        # 保存CSV结果
        csv_file = output_file.replace('.json', '.csv')
        with open(csv_file, 'w', newline='', encoding='utf-8') as f:
            if self.results:
                # 定义固定的字段名，确保所有记录都有相同字段
                fieldnames = [
                    'test_id', 'timestamp', 'speech_to_first_response_ms', 
                    'speech_to_tts_start_ms', 'total_response_time_ms', 
                    'prompt', 'success', 'timeout'
                ]
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                
                # 确保每个结果都有所有字段
                for result in self.results:
                    clean_result = {}
                    for field in fieldnames:
                        clean_result[field] = result.get(field, None)
                    writer.writerow(clean_result)
        
        # 生成报告
        self.generate_report(output_file.replace('.json', '_report.txt'), stats)
        
        self.get_logger().info(f'Results saved to {output_file}')
        self.get_logger().info(f'CSV data saved to {csv_file}')

    def generate_report(self, report_file, stats):
        """生成文本报告"""
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write("Baseline Performance Test Report\n")
            f.write("=" * 50 + "\n\n")
            f.write(f"Test Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Architecture: Conventional STT → LLM → TTS Pipeline\n")
            f.write(f"Total Samples: {stats.get('total_samples', 0)}\n")
            f.write(f"Valid Samples: {stats.get('valid_samples', 0)}\n")
            f.write(f"Timeouts: {stats.get('timeout_count', 0)}\n\n")
            
            # 详细统计
            metrics = {
                'speech_to_first_response_ms': 'Speech → First Response (ms)',
                'speech_to_tts_start_ms': 'Speech → TTS Start (ms)', 
                'total_response_time_ms': 'Total Response Time (ms)'
            }
            
            for metric_key, metric_name in metrics.items():
                if metric_key in stats:
                    metric_stats = stats[metric_key]
                    f.write(f"{metric_name}:\n")
                    f.write(f"  Count: {metric_stats.get('count', 0)}\n")
                    f.write(f"  Median: {metric_stats.get('median', 0):.1f}ms\n")
                    f.write(f"  Mean: {metric_stats.get('mean', 0):.1f}ms\n")
                    f.write(f"  Std Dev: {metric_stats.get('std', 0):.1f}ms\n")
                    f.write(f"  Min: {metric_stats.get('min', 0):.1f}ms\n")
                    f.write(f"  Max: {metric_stats.get('max', 0):.1f}ms\n")
                    f.write(f"  P95: {metric_stats.get('p95', 0):.1f}ms\n")
                    if metric_stats.get('confidence_interval_95'):
                        ci = metric_stats['confidence_interval_95']
                        f.write(f"  95% CI: [{ci[0]:.1f}, {ci[1]:.1f}]ms\n")
                    f.write("\n")


def check_prerequisites():
    """检查测试前提条件"""
    print("🔍 Checking prerequisites...")
    
    # 检查ROS2环境
    if not os.path.exists('/opt/ros/humble/setup.bash'):
        raise Exception("ROS2 Humble not found. Please install ROS2 Humble.")
    
    # 检查工作空间
    if not os.path.exists('/workspaces/ros2_ws/install/setup.bash'):
        raise Exception("ROS2 workspace not built. Please run 'colcon build'.")
    
    # 检查API密钥
    if not os.environ.get('OPENAI_API_KEY'):
        raise Exception("OPENAI_API_KEY not set. Please set in .env file.")
    
    print("✅ Prerequisites check passed")


def start_baseline_nodes():
    """启动baseline节点"""
    print("🚀 Starting baseline pipeline nodes...")
    
    nodes_to_start = [
        'ros2 run my_voice_assistant openai_stt_node',
        'ros2 run my_voice_assistant llm_node', 
        'ros2 run my_voice_assistant openai_tts_node'
    ]
    
    processes = []
    for cmd in nodes_to_start:
        try:
            print(f"Starting: {cmd}")
            proc = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            processes.append(proc)
            time.sleep(2)  # 给节点启动时间
        except Exception as e:
            print(f"Failed to start {cmd}: {e}")
    
    print("✅ Baseline nodes started")
    return processes


def main():
    parser = argparse.ArgumentParser(description='Baseline Performance Test')
    parser.add_argument('--samples', type=int, default=30, help='Number of test samples')
    parser.add_argument('--output', type=str, default='baseline_results.json', help='Output file')
    parser.add_argument('--no-nodes', action='store_true', help='Skip starting nodes (assume already running)')
    
    args = parser.parse_args()
    
    try:
        # 检查前提条件
        check_prerequisites()
        
        # 启动ROS2
        rclpy.init()
        
        # 启动baseline节点（除非指定跳过）
        processes = []
        if not args.no_nodes:
            processes = start_baseline_nodes()
            time.sleep(5)  # 等待节点完全启动
        
        # 创建测试节点
        test_node = BaselineLatencyTest(num_samples=args.samples)
        
        print(f"🎯 Starting baseline performance test with {args.samples} samples...")
        print("Press Ctrl+C to stop early")
        
        # 运行测试
        test_node.run_test_sequence()
        
        # 保存结果
        test_node.save_results(args.output)
        
        # 打印汇总
        stats = test_node.calculate_statistics()
        print("\n📊 Test Summary:")
        print(f"Valid samples: {stats.get('valid_samples', 0)}/{stats.get('total_samples', 0)}")
        
        if 'speech_to_first_response_ms' in stats:
            median = stats['speech_to_first_response_ms'].get('median', 0)
            print(f"Speech → First Response: {median:.1f}ms (median)")
        
        if 'total_response_time_ms' in stats:
            median = stats['total_response_time_ms'].get('median', 0) 
            print(f"Total Response Time: {median:.1f}ms (median)")
        
    except KeyboardInterrupt:
        print("\n⏹️  Test stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        # 清理
        rclpy.shutdown()
        
        # 停止启动的节点
        for proc in processes:
            try:
                proc.terminate()
                proc.wait(timeout=5)
            except:
                proc.kill()


if __name__ == '__main__':
    main()