#!/usr/bin/env python3
"""
真实的端到端延迟测量脚本
通过ROS 2话题监听，记录关键时间点：
1. STT检测"speech end"时间戳
2. 第一条realtime_response文本增量时间戳  
3. tts_status True/False切换时间戳
4. realtime_status变化时间戳
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time
import csv
import json
from datetime import datetime
import threading
import queue
import statistics
import os

class E2ELatencyMeasurement(Node):
    def __init__(self):
        super().__init__('e2e_latency_test')
        self.get_logger().info('启动端到端延迟测量节点')
        
        # 数据存储
        self.measurements = []
        self.current_test = {}
        self.test_start_time = None
        
        # 订阅关键话题
        self.speech_sub = self.create_subscription(
            String, 'speech_text', self.speech_callback, 10)
        self.realtime_response_sub = self.create_subscription(
            String, 'realtime_response', self.realtime_response_callback, 10)
        self.realtime_response_full_sub = self.create_subscription(
            String, 'realtime_response_full', self.realtime_response_full_callback, 10)
        self.tts_status_sub = self.create_subscription(
            Bool, 'tts_status', self.tts_status_callback, 10)
        self.realtime_status_sub = self.create_subscription(
            Bool, 'realtime_status', self.realtime_status_callback, 10)
        
        # 发布测试语音文本
        self.speech_pub = self.create_publisher(String, 'speech_text', 10)
        
        # 测试队列
        self.test_queue = queue.Queue()
        self.current_test_id = 0
        
        # 预设测试语句
        self.test_utterances = [
            "new way 4, what's the weather like today?",
            "new way 4, when does the next shuttle arrive?", 
            "new way 4, what dining options are available on campus?",
            "new way 4, tell me about UWA events this week",
            "new way 4, what time does the library close?"
        ]

        # 每个测试的超时时间（秒），避免某轮卡住
        try:
            self.test_timeout_sec = float(os.getenv('E2E_TEST_TIMEOUT', '20.0'))
        except Exception:
            self.test_timeout_sec = 10.0
        # 每轮测试之间的间隔（秒），默认5秒，可通过环境变量调整
        try:
            self.test_gap_sec = float(os.getenv('E2E_TEST_GAP_SEC', '5.0'))
        except Exception:
            self.test_gap_sec = 5.0
        # 是否在TTS播放期间自动发出中断以测量中断时延
        self.measure_interrupt = os.getenv('E2E_MEASURE_INTERRUPT', '0') in ('1', 'true', 'True')
        try:
            self.interrupt_after_sec = float(os.getenv('E2E_INTERRUPT_AFTER_SEC', '0.4'))
        except Exception:
            self.interrupt_after_sec = 0.4

        self._test_timer = None
        self._interrupt_timer = None
        # 发布中断的Publisher（用于测量中断时延）
        self.realtime_interrupt_pub = self.create_publisher(Bool, 'realtime_interrupt', 10)
        
    def speech_callback(self, msg):
        """记录语音文本发送时间"""
        timestamp = time.time()
        if msg.data.strip():
            self.get_logger().info(f'语音文本: {msg.data[:50]}...')
            
            # 如果是我们发送的测试语句，记录为speech_end
            for i, utterance in enumerate(self.test_utterances):
                if utterance in msg.data:
                    self.current_test = {
                        'test_id': self.current_test_id,
                        'utterance': utterance,
                        'speech_end_time': timestamp,
                        'speech_text': msg.data
                    }
                    self.get_logger().info(f'测试 {self.current_test_id}: 语音结束时间记录')
                    break
    
    def realtime_response_callback(self, msg):
        """记录第一条Realtime响应时间"""
        timestamp = time.time()
        if msg.data.strip() and 'speech_end_time' in self.current_test:
            if 'first_response_time' not in self.current_test:
                self.current_test['first_response_time'] = timestamp
                self.current_test['first_response_content'] = msg.data
                
                # 计算语音结束到首次响应的延迟
                latency = timestamp - self.current_test['speech_end_time']
                self.current_test['speech_to_response_latency'] = latency
                
                self.get_logger().info(f'测试 {self.current_test_id}: 首次响应延迟 {latency:.3f}s')
    
    def realtime_response_full_callback(self, msg):
        """记录完整响应时间"""
        timestamp = time.time()
        if msg.data.strip() and 'speech_end_time' in self.current_test:
            self.current_test['full_response_time'] = timestamp
            self.current_test['full_response_content'] = msg.data
            
            # 计算总延迟
            if 'speech_end_time' in self.current_test:
                total_latency = timestamp - self.current_test['speech_end_time']
                self.current_test['total_latency'] = total_latency
                self.get_logger().info(f'测试 {self.current_test_id}: 总延迟 {total_latency:.3f}s')
    
    def tts_status_callback(self, msg):
        """记录TTS状态变化"""
        timestamp = time.time()
        if 'speech_end_time' in self.current_test:
            if msg.data:  # TTS开始
                if 'tts_start_time' not in self.current_test:
                    self.current_test['tts_start_time'] = timestamp
                    tts_latency = timestamp - self.current_test['speech_end_time']
                    self.current_test['speech_to_tts_latency'] = tts_latency
                    self.get_logger().info(f'测试 {self.current_test_id}: TTS开始延迟 {tts_latency:.3f}s')
                    # 如启用中断测量，则在TTS开始后延时发送中断
                    if self.measure_interrupt:
                        try:
                            if self._interrupt_timer is not None:
                                self._interrupt_timer.cancel()
                        except Exception:
                            pass
                        from threading import Timer as _Timer
                        self._interrupt_timer = _Timer(self.interrupt_after_sec, self._send_interrupt, args=(self.current_test_id,))
                        self._interrupt_timer.start()
            else:  # TTS结束
                if 'tts_start_time' in self.current_test:
                    self.current_test['tts_end_time'] = timestamp
                    if 'tts_start_time' in self.current_test:
                        tts_duration = timestamp - self.current_test['tts_start_time']
                        self.current_test['tts_duration'] = tts_duration
                        self.get_logger().info(f'测试 {self.current_test_id}: TTS持续时间 {tts_duration:.3f}s')
                    # 如之前发送过中断，记录中断时延（从发送中断到TTS结束）
                    if self.current_test.get('interrupt_sent_time') and 'interrupt_latency' not in self.current_test:
                        self.current_test['interrupt_latency'] = timestamp - self.current_test['interrupt_sent_time']
                        self.get_logger().info(f'测试 {self.current_test_id}: 中断时延 {self.current_test["interrupt_latency"]:.3f}s')
                    
                    # 完成当前测试
                    self.complete_test()
    
    def realtime_status_callback(self, msg):
        """记录Realtime状态变化"""
        timestamp = time.time()
        if 'speech_end_time' in self.current_test:
            status_key = 'realtime_status_true' if msg.data else 'realtime_status_false'
            self.current_test[f'{status_key}_time'] = timestamp
    
    def complete_test(self):
        """完成当前测试并保存数据"""
        if self.current_test:
            # 取消本轮超时定时器
            try:
                if self._test_timer is not None:
                    self._test_timer.cancel()
            except Exception:
                pass
            finally:
                self._test_timer = None
            # 取消中断定时器
            try:
                if self._interrupt_timer is not None:
                    self._interrupt_timer.cancel()
            except Exception:
                pass
            finally:
                self._interrupt_timer = None

            self.current_test['timestamp'] = datetime.now().isoformat()
            self.measurements.append(self.current_test.copy())
            
            self.get_logger().info(f'完成测试 {self.current_test_id}')
            self.current_test_id += 1
            self.current_test = {}
            
            # 等待下一次测试
            if self.current_test_id < len(self.test_utterances):
                self.get_logger().info(f'准备测试 {self.current_test_id}，等待{self.test_gap_sec:.1f}秒...')
                threading.Timer(self.test_gap_sec, self.send_next_test).start()
            else:
                self.get_logger().info('所有测试完成，保存结果...')
                self.save_results()
    
    def send_next_test(self):
        """发送下一个测试语句"""
        if self.current_test_id < len(self.test_utterances):
            utterance = self.test_utterances[self.current_test_id]
            msg = String()
            msg.data = utterance
            # 先打印，再发布，避免日志顺序与回调交错导致困惑
            self.get_logger().info(f'发送测试 {self.current_test_id}: {utterance}')
            self.speech_pub.publish(msg)
            # 启动本轮超时定时器，防止卡住
            try:
                if self._test_timer is not None:
                    self._test_timer.cancel()
            except Exception:
                pass
            from threading import Timer as _Timer
            self._test_timer = _Timer(self.test_timeout_sec, self._on_test_timeout, args=(self.current_test_id, utterance))
            self._test_timer.start()

    def _on_test_timeout(self, test_id: int, utterance: str):
        """单轮测试超时兜底，记录并推进下一轮。"""
        # 仅当仍停留在当前轮时才处理
        if self.current_test_id == test_id and self.current_test:
            self.get_logger().warning(f'测试 {test_id} 超时（>{self.test_timeout_sec:.1f}s），自动结束并继续下一轮')
            self.current_test.setdefault('utterance', utterance)
            self.current_test['timeout'] = True
            # 不再等待TTS结束，直接收尾
            self.complete_test()

    def _send_interrupt(self, test_id: int):
        """在TTS期间发送中断信号并记录时间。"""
        # 仅当仍是同一轮且尚未发过中断
        if self.current_test_id == test_id and 'interrupt_sent_time' not in self.current_test:
            msg = Bool()
            msg.data = True
            self.realtime_interrupt_pub.publish(msg)
            ts = time.time()
            self.current_test['interrupt_sent_time'] = ts
            self.get_logger().info(f'测试 {test_id}: 已发送中断请求')
    
    def start_testing(self):
        """开始测试序列"""
        self.get_logger().info('开始端到端延迟测试...')
        self.get_logger().info('请确保STT节点和Realtime节点已启动')
        
        # 等待一定时间让节点稳定
        threading.Timer(self.test_gap_sec, self.send_next_test).start()
    
    def save_results(self):
        """保存测试结果"""
        # 保存为JSON
        with open('e2e_latency_results.json', 'w', encoding='utf-8') as f:
            json.dump({
                'timestamp': datetime.now().isoformat(),
                'total_tests': len(self.measurements),
                'measurements': self.measurements
            }, f, ensure_ascii=False, indent=2)
        
        # 保存为CSV
        if self.measurements:
            fieldnames = set()
            for measurement in self.measurements:
                fieldnames.update(measurement.keys())
            
            with open('e2e_latency_results.csv', 'w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=sorted(fieldnames))
                writer.writeheader()
                writer.writerows(self.measurements)
        
        # 生成统计报告
        self.generate_statistics()
        
        self.get_logger().info('结果已保存到:')
        self.get_logger().info('  - e2e_latency_results.json')
        self.get_logger().info('  - e2e_latency_results.csv')
        self.get_logger().info('  - e2e_latency_statistics.txt')
    
    def generate_statistics(self):
        """生成统计分析"""
        if not self.measurements:
            return
        
        # 提取关键延迟数据
        speech_to_response = [m.get('speech_to_response_latency') for m in self.measurements if m.get('speech_to_response_latency')]
        speech_to_tts = [m.get('speech_to_tts_latency') for m in self.measurements if m.get('speech_to_tts_latency')]
        total_latencies = [m.get('total_latency') for m in self.measurements if m.get('total_latency')]
        interrupt_latencies = [m.get('interrupt_latency') for m in self.measurements if m.get('interrupt_latency')]
        timeouts = sum(1 for m in self.measurements if m.get('timeout'))

        def _fmt(v):
            try:
                if v is None:
                    return 'N/A'
                return f"{float(v):.3f}s"
            except Exception:
                return 'N/A'

        with open('e2e_latency_statistics.txt', 'w', encoding='utf-8') as f:
            f.write("端到端延迟测试统计报告\n")
            f.write("=" * 50 + "\n")
            f.write(f"测试时间: {datetime.now().isoformat()}\n")
            f.write(f"测试数量: {len(self.measurements)}\n")
            f.write(f"成功轮次: {len(self.measurements) - timeouts}\n")
            f.write(f"超时轮次: {timeouts}\n\n")
            
            if speech_to_response:
                f.write("语音结束到首次响应延迟:\n")
                f.write(f"  平均值: {statistics.mean(speech_to_response):.3f}s\n")
                f.write(f"  中位数: {statistics.median(speech_to_response):.3f}s\n")
                f.write(f"  标准差: {statistics.stdev(speech_to_response):.3f}s\n")
                f.write(f"  最小值: {min(speech_to_response):.3f}s\n")
                f.write(f"  最大值: {max(speech_to_response):.3f}s\n\n")
            
            if speech_to_tts:
                f.write("语音结束到TTS开始延迟:\n")
                f.write(f"  平均值: {statistics.mean(speech_to_tts):.3f}s\n")
                f.write(f"  中位数: {statistics.median(speech_to_tts):.3f}s\n")
                f.write(f"  标准差: {statistics.stdev(speech_to_tts):.3f}s\n\n")
            
            if total_latencies:
                f.write("总延迟（语音结束到完整响应）:\n")
                f.write(f"  平均值: {statistics.mean(total_latencies):.3f}s\n")
                f.write(f"  中位数: {statistics.median(total_latencies):.3f}s\n")
                f.write(f"  标准差: {statistics.stdev(total_latencies):.3f}s\n\n")

            if interrupt_latencies:
                f.write("中断时延（发送中断→TTS结束）:\n")
                f.write(f"  平均值: {statistics.mean(interrupt_latencies):.3f}s\n")
                f.write(f"  中位数: {statistics.median(interrupt_latencies):.3f}s\n")
                f.write(f"  标准差: {statistics.stdev(interrupt_latencies):.3f}s\n\n")
            
            f.write("详细测试结果:\n")
            for i, m in enumerate(self.measurements):
                f.write(f"测试 {i+1}: {m.get('utterance', 'N/A')}\n")
                f.write(f"  首次响应延迟: {_fmt(m.get('speech_to_response_latency'))}\n")
                f.write(f"  TTS开始延迟: {_fmt(m.get('speech_to_tts_latency'))}\n")
                f.write(f"  总延迟: {_fmt(m.get('total_latency'))}\n")
                if m.get('interrupt_sent_time') is not None:
                    f.write(f"  中断时延: {_fmt(m.get('interrupt_latency'))}\n")
                if m.get('timeout'):
                    f.write("  超时: True\n")
                f.write("\n")

def main():
    rclpy.init()
    
    node = E2ELatencyMeasurement()
    
    # 启动测试
    node.start_testing()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('测试被中断')
        node.save_results()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
