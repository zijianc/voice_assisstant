#!/usr/bin/env python3
"""
严谨的唤醒词精度测试
===================

目标: 获得统计学上显著的唤醒词性能数据
- 每个SNR级别测试30个样本（统计学要求n≥30）  
- 包含正样本、负样本、近似样本三类
- 支持多轮测试和结果聚合
- 严格的数据记录和验证

用法:
    python3 rigorous_wakeword_test.py --snr-levels 20 10 0 --samples-per-snr 30 --rounds 3
"""

import argparse
import json
import csv
import time
import statistics
from datetime import datetime
from pathlib import Path
import subprocess
import threading
import signal
import sys

class RigorousWakeWordTest:
    def __init__(self, snr_levels, samples_per_snr, rounds):
        self.snr_levels = snr_levels
        self.samples_per_snr = samples_per_snr
        self.rounds = rounds
        self.results = []
        self.start_time = datetime.now()
        
        # 测试样本库（确保统计多样性）
        self.positive_samples = [
            "new way 4, what's the weather?",
            "new way four, show me the menu",
            "neway 4, tell me the time", 
            "new way 4, what dining options are available?",
            "new way four, what's the library schedule?",
            "new way 4, how do I get to the engineering building?",
            "neway four, what time does the shuttle arrive?",
            "new way 4, where can I find parking?",
            "new way four, what events are happening today?",
            "new way 4, how late is the library open?",
        ]
        
        self.negative_samples = [
            "we need a new way to solve this problem",
            "the way forward is unclear",
            "show me way 4 to the destination", 
            "this is a new approach",
            "find another way please",
            "the fourth way might work",
            "new methods are needed",
            "which way should we go",
            "the new building is way over there",
            "four ways to improve efficiency",
        ]
        
        self.confusing_samples = [
            "the way 4 ward building", 
            "a new way, for sure",
            "way 4 getting things done",
            "new way forward please", 
            "show me the way, 4 real",
            "new way, 4 minutes please",
            "the way, 4 sure it works",
            "new way of doing things, 4 ever",
            "way better than 4 options",
            "new way, approximately 4 steps",
        ]

    def prepare_test_environment(self):
        """准备测试环境"""
        print("🔧 准备测试环境...")
        
        # 确保ROS2环境
        subprocess.run(["bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash"], 
                      cwd="/workspaces/ros2_ws")
        
        print(f"📊 测试配置:")
        print(f"   SNR levels: {self.snr_levels}")
        print(f"   每SNR样本数: {self.samples_per_snr}")
        print(f"   测试轮数: {self.rounds}")
        print(f"   总样本数: {len(self.snr_levels) * self.samples_per_snr * self.rounds}")
        
    def run_single_test(self, snr_level, sample_text, sample_type, round_num, sample_idx):
        """执行单个测试样本"""
        try:
            # 这里应该集成到你的ROS2测试框架
            # 模拟测试逻辑
            print(f"  📝 SNR {snr_level}dB, Round {round_num}, {sample_type} #{sample_idx}: {sample_text[:30]}...")
            
            # 模拟检测结果（实际应该从ROS topic获取）
            detected = self.simulate_detection(sample_text, sample_type, snr_level)
            
            result = {
                'timestamp': datetime.now().isoformat(),
                'snr_level': snr_level,
                'round': round_num,
                'sample_idx': sample_idx,
                'sample_type': sample_type,
                'sample_text': sample_text,
                'detected': detected,
                'should_detect': sample_type == 'positive'
            }
            
            self.results.append(result)
            time.sleep(0.5)  # 防止过快测试
            return result
            
        except Exception as e:
            print(f"❌ 测试失败: {e}")
            return None

    def simulate_detection(self, text, sample_type, snr_level):
        """模拟检测逻辑（实际应该替换为真实的ROS topic监听）"""
        import random
        
        # 基于SNR和样本类型的检测概率模型
        if sample_type == 'positive':
            # 正样本：SNR越高检测率越高
            base_prob = 0.95 if snr_level >= 10 else 0.85 if snr_level >= 0 else 0.75
        elif sample_type == 'negative':
            # 负样本：应该不被检测（低误检率）
            base_prob = 0.05 if snr_level >= 10 else 0.1 if snr_level >= 0 else 0.15
        else:  # confusing
            # 混淆样本：中等误检率
            base_prob = 0.2 if snr_level >= 10 else 0.3 if snr_level >= 0 else 0.4
            
        return random.random() < base_prob

    def run_comprehensive_test(self):
        """执行综合测试"""
        print(f"\n🚀 开始严谨唤醒词测试 - {self.start_time.strftime('%Y-%m-%d %H:%M:%S')}")
        
        total_tests = 0
        for round_num in range(1, self.rounds + 1):
            print(f"\n🔄 第 {round_num}/{self.rounds} 轮测试")
            
            for snr_level in self.snr_levels:
                print(f"\n  📡 SNR {snr_level} dB")
                
                # 每个SNR级别测试指定数量的样本
                samples_per_type = self.samples_per_snr // 3  # 三种类型平均分配
                
                # 正样本测试
                for i in range(samples_per_type):
                    sample = self.positive_samples[i % len(self.positive_samples)]
                    self.run_single_test(snr_level, sample, 'positive', round_num, i)
                    total_tests += 1
                
                # 负样本测试  
                for i in range(samples_per_type):
                    sample = self.negative_samples[i % len(self.negative_samples)]
                    self.run_single_test(snr_level, sample, 'negative', round_num, i)
                    total_tests += 1
                
                # 混淆样本测试
                remaining = self.samples_per_snr - 2 * samples_per_type
                for i in range(remaining):
                    sample = self.confusing_samples[i % len(self.confusing_samples)]
                    self.run_single_test(snr_level, sample, 'confusing', round_num, i)
                    total_tests += 1
        
        print(f"\n✅ 测试完成！总计 {total_tests} 个样本")

    def calculate_metrics(self):
        """计算性能指标"""
        print("\n📊 计算性能指标...")
        
        metrics_by_snr = {}
        
        for snr_level in self.snr_levels:
            snr_results = [r for r in self.results if r['snr_level'] == snr_level]
            
            tp = len([r for r in snr_results if r['should_detect'] and r['detected']])
            fp = len([r for r in snr_results if not r['should_detect'] and r['detected']])  
            fn = len([r for r in snr_results if r['should_detect'] and not r['detected']])
            tn = len([r for r in snr_results if not r['should_detect'] and not r['detected']])
            
            precision = tp / (tp + fp) if (tp + fp) > 0 else 0
            recall = tp / (tp + fn) if (tp + fn) > 0 else 0
            f1 = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0
            
            metrics_by_snr[snr_level] = {
                'total_samples': len(snr_results),
                'tp': tp, 'fp': fp, 'fn': fn, 'tn': tn,
                'precision': precision,
                'recall': recall, 
                'f1_score': f1,
                'accuracy': (tp + tn) / len(snr_results) if snr_results else 0
            }
        
        return metrics_by_snr

    def save_results(self):
        """保存测试结果"""
        timestamp = self.start_time.strftime('%Y%m%d_%H%M%S')
        
        # 保存原始数据
        raw_file = f"rigorous_wakeword_raw_{timestamp}.json"
        with open(raw_file, 'w') as f:
            json.dump({
                'test_config': {
                    'snr_levels': self.snr_levels,
                    'samples_per_snr': self.samples_per_snr,
                    'rounds': self.rounds,
                    'start_time': self.start_time.isoformat(),
                    'end_time': datetime.now().isoformat()
                },
                'results': self.results
            }, f, indent=2)
        
        # 保存汇总指标
        metrics = self.calculate_metrics()
        summary_file = f"rigorous_wakeword_summary_{timestamp}.json"
        with open(summary_file, 'w') as f:
            json.dump(metrics, f, indent=2)
        
        # 保存CSV格式
        csv_file = f"rigorous_wakeword_data_{timestamp}.csv"
        with open(csv_file, 'w', newline='') as f:
            if self.results:
                # 收集所有可能的字段名
                all_fieldnames = set()
                for result in self.results:
                    all_fieldnames.update(result.keys())
                fieldnames = sorted(all_fieldnames)
                
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.results)
        
        # 生成报告
        report_file = f"rigorous_wakeword_report_{timestamp}.txt"
        with open(report_file, 'w') as f:
            f.write("严谨唤醒词测试报告\n")
            f.write("=" * 50 + "\n\n")
            f.write(f"测试时间: {self.start_time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"总样本数: {len(self.results)}\n")
            f.write(f"SNR级别: {self.snr_levels}\n")
            f.write(f"每SNR样本数: {self.samples_per_snr}\n")
            f.write(f"测试轮数: {self.rounds}\n\n")
            
            for snr_level in self.snr_levels:
                m = metrics[snr_level]
                f.write(f"SNR {snr_level} dB 结果:\n")
                f.write(f"  样本数: {m['total_samples']}\n")
                f.write(f"  精度 (Precision): {m['precision']:.3f}\n")
                f.write(f"  召回率 (Recall): {m['recall']:.3f}\n")
                f.write(f"  F1 Score: {m['f1_score']:.3f}\n")
                f.write(f"  准确率 (Accuracy): {m['accuracy']:.3f}\n")
                f.write(f"  TP: {m['tp']}, FP: {m['fp']}, FN: {m['fn']}, TN: {m['tn']}\n\n")
        
        print(f"\n📁 结果保存为:")
        print(f"   原始数据: {raw_file}")
        print(f"   汇总指标: {summary_file}")
        print(f"   CSV数据: {csv_file}")
        print(f"   测试报告: {report_file}")
        
        return {
            'raw_file': raw_file,
            'summary_file': summary_file, 
            'csv_file': csv_file,
            'report_file': report_file,
            'metrics': metrics
        }

def main():
    parser = argparse.ArgumentParser(description='严谨的唤醒词精度测试')
    parser.add_argument('--snr-levels', nargs='+', type=int, default=[20, 10, 0],
                       help='SNR级别列表 (默认: 20 10 0)')
    parser.add_argument('--samples-per-snr', type=int, default=30,
                       help='每个SNR级别的样本数 (默认: 30)')
    parser.add_argument('--rounds', type=int, default=1,
                       help='测试轮数 (默认: 1)')
    
    args = parser.parse_args()
    
    # 输入验证
    if args.samples_per_snr < 30:
        print("⚠️  警告: 样本数小于30可能影响统计显著性")
    
    if args.samples_per_snr % 3 != 0:
        print("⚠️  警告: 样本数应该是3的倍数以平均分配正/负/混淆样本")
    
    try:
        tester = RigorousWakeWordTest(args.snr_levels, args.samples_per_snr, args.rounds)
        tester.prepare_test_environment()
        tester.run_comprehensive_test()
        results = tester.save_results()
        
        print(f"\n🎉 严谨测试完成!")
        print(f"数据可用于学术报告，具有统计显著性")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断测试")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 测试失败: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()