#!/usr/bin/env python3
"""
Performance Comparison Script
============================

Compare baseline (STT→LLM→TTS) vs OpenAI Realtime approach performance.

Usage:
    python3 compare_performance.py --baseline baseline_results.json --realtime rigorous_latency_data.csv
"""

import argparse
import json
import csv
import statistics
import numpy as np
from datetime import datetime
from pathlib import Path
import matplotlib.pyplot as plt
import pandas as pd

class PerformanceComparison:
    def __init__(self, baseline_file, realtime_file):
        self.baseline_file = baseline_file
        self.realtime_file = realtime_file
        self.baseline_data = None
        self.realtime_data = None
        
    def load_baseline_data(self):
        """加载baseline测试结果"""
        try:
            with open(self.baseline_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
                self.baseline_data = data
                print(f"✅ Loaded baseline data: {len(data.get('raw_results', []))} samples")
                return True
        except Exception as e:
            print(f"❌ Error loading baseline data: {e}")
            return False
    
    def load_realtime_data(self):
        """加载realtime测试结果"""
        try:
            # 尝试加载JSON格式
            if self.realtime_file.endswith('.json'):
                with open(self.realtime_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    self.realtime_data = data
            # 尝试加载CSV格式
            elif self.realtime_file.endswith('.csv'):
                data = []
                with open(self.realtime_file, 'r', encoding='utf-8') as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        # 转换数值字段
                        for key in row:
                            if 'latency' in key.lower() or 'time' in key.lower():
                                try:
                                    row[key] = float(row[key]) if row[key] else None
                                except:
                                    pass
                        data.append(row)
                self.realtime_data = {'raw_results': data}
            
            print(f"✅ Loaded realtime data: {len(self.realtime_data.get('raw_results', []))} samples")
            return True
        except Exception as e:
            print(f"❌ Error loading realtime data: {e}")
            return False
    
    def extract_baseline_metrics(self):
        """提取baseline指标"""
        if not self.baseline_data:
            return {}
            
        raw_results = self.baseline_data.get('raw_results', [])
        valid_results = [r for r in raw_results if not r.get('timeout', False)]
        
        if not valid_results:
            return {}
            
        metrics = {}
        
        # Speech to first response
        values = [r['speech_to_first_response_ms'] for r in valid_results 
                 if r.get('speech_to_first_response_ms') is not None]
        if values:
            metrics['speech_to_first_response'] = {
                'values': values,
                'median': statistics.median(values),
                'mean': statistics.mean(values),
                'std': statistics.stdev(values) if len(values) > 1 else 0,
                'count': len(values)
            }
        
        # Total response time
        values = [r['total_response_time_ms'] for r in valid_results 
                 if r.get('total_response_time_ms') is not None]
        if values:
            metrics['total_response_time'] = {
                'values': values,
                'median': statistics.median(values),
                'mean': statistics.mean(values),
                'std': statistics.stdev(values) if len(values) > 1 else 0,
                'count': len(values)
            }
            
        return metrics
    
    def extract_realtime_metrics(self):
        """提取realtime指标"""
        if not self.realtime_data:
            return {}
            
        raw_results = self.realtime_data.get('raw_results', [])
        if not raw_results:
            return {}
            
        metrics = {}
        
        # 尝试不同的字段名称
        first_response_fields = ['speech_to_first_response_latency', 'first_response_latency', 'speech_to_first_response_ms']
        total_response_fields = ['total_response_time', 'end_to_end_latency', 'total_response_time_ms']
        
        # Speech to first response
        for field in first_response_fields:
            values = [float(r[field]) for r in raw_results 
                     if r.get(field) is not None and str(r[field]).replace('.','').isdigit()]
            if values:
                metrics['speech_to_first_response'] = {
                    'values': values,
                    'median': statistics.median(values),
                    'mean': statistics.mean(values),
                    'std': statistics.stdev(values) if len(values) > 1 else 0,
                    'count': len(values)
                }
                break
        
        # Total response time
        for field in total_response_fields:
            values = [float(r[field]) for r in raw_results 
                     if r.get(field) is not None and str(r[field]).replace('.','').isdigit()]
            if values:
                metrics['total_response_time'] = {
                    'values': values,
                    'median': statistics.median(values),
                    'mean': statistics.mean(values),
                    'std': statistics.stdev(values) if len(values) > 1 else 0,
                    'count': len(values)
                }
                break
                
        return metrics
    
    def calculate_improvement(self, baseline_value, realtime_value):
        """计算改进百分比"""
        if baseline_value == 0:
            return 0
        return ((baseline_value - realtime_value) / baseline_value) * 100
    
    def generate_comparison_report(self, output_file):
        """生成对比报告"""
        baseline_metrics = self.extract_baseline_metrics()
        realtime_metrics = self.extract_realtime_metrics()
        
        report = {
            'comparison_info': {
                'timestamp': datetime.now().isoformat(),
                'baseline_file': self.baseline_file,
                'realtime_file': self.realtime_file,
                'baseline_architecture': 'Conventional STT → LLM → TTS',
                'realtime_architecture': 'OpenAI Realtime API'
            },
            'metrics_comparison': {},
            'summary': {}
        }
        
        # 对比各项指标
        for metric_name in ['speech_to_first_response', 'total_response_time']:
            if metric_name in baseline_metrics and metric_name in realtime_metrics:
                baseline = baseline_metrics[metric_name]
                realtime = realtime_metrics[metric_name]
                
                improvement = self.calculate_improvement(baseline['median'], realtime['median'])
                
                report['metrics_comparison'][metric_name] = {
                    'baseline': {
                        'median_ms': baseline['median'],
                        'mean_ms': baseline['mean'],
                        'std_ms': baseline['std'],
                        'count': baseline['count']
                    },
                    'realtime': {
                        'median_ms': realtime['median'],
                        'mean_ms': realtime['mean'],
                        'std_ms': realtime['std'],
                        'count': realtime['count']
                    },
                    'improvement': {
                        'percentage': improvement,
                        'absolute_ms': baseline['median'] - realtime['median']
                    }
                }
        
        # 生成汇总
        if 'speech_to_first_response' in report['metrics_comparison']:
            comparison = report['metrics_comparison']['speech_to_first_response']
            report['summary']['primary_latency_improvement'] = {
                'baseline_median_ms': comparison['baseline']['median_ms'],
                'realtime_median_ms': comparison['realtime']['median_ms'],
                'improvement_percentage': comparison['improvement']['percentage'],
                'improvement_absolute_ms': comparison['improvement']['absolute_ms']
            }
        
        # 保存报告
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        
        # 生成文本报告
        self.generate_text_report(output_file.replace('.json', '_report.txt'), report)
        
        return report
    
    def generate_text_report(self, report_file, report_data):
        """生成文本格式报告"""
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write("Performance Comparison Report\n")
            f.write("=" * 50 + "\n\n")
            
            f.write(f"Test Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Baseline Architecture: {report_data['comparison_info']['baseline_architecture']}\n")
            f.write(f"Realtime Architecture: {report_data['comparison_info']['realtime_architecture']}\n\n")
            
            # 指标对比
            f.write("Metrics Comparison:\n")
            f.write("-" * 30 + "\n\n")
            
            for metric_name, comparison in report_data['metrics_comparison'].items():
                metric_display = metric_name.replace('_', ' ').title()
                f.write(f"{metric_display}:\n")
                
                baseline = comparison['baseline']
                realtime = comparison['realtime']
                improvement = comparison['improvement']
                
                f.write(f"  Baseline (STT→LLM→TTS):\n")
                f.write(f"    Median: {baseline['median_ms']:.1f}ms\n")
                f.write(f"    Mean: {baseline['mean_ms']:.1f}ms\n")
                f.write(f"    Std Dev: {baseline['std_ms']:.1f}ms\n")
                f.write(f"    Samples: {baseline['count']}\n\n")
                
                f.write(f"  Realtime (OpenAI API):\n")
                f.write(f"    Median: {realtime['median_ms']:.1f}ms\n")
                f.write(f"    Mean: {realtime['mean_ms']:.1f}ms\n")
                f.write(f"    Std Dev: {realtime['std_ms']:.1f}ms\n")
                f.write(f"    Samples: {realtime['count']}\n\n")
                
                f.write(f"  Improvement:\n")
                f.write(f"    Percentage: {improvement['percentage']:.1f}%\n")
                f.write(f"    Absolute: {improvement['absolute_ms']:.1f}ms faster\n\n")
            
            # 汇总
            if 'primary_latency_improvement' in report_data['summary']:
                summary = report_data['summary']['primary_latency_improvement']
                f.write("Summary:\n")
                f.write("-" * 20 + "\n")
                f.write(f"Primary latency improvement: {summary['improvement_percentage']:.1f}%\n")
                f.write(f"From {summary['baseline_median_ms']:.1f}ms to {summary['realtime_median_ms']:.1f}ms\n")
                f.write(f"Absolute improvement: {summary['improvement_absolute_ms']:.1f}ms\n")
    
    def create_visualization(self, output_dir):
        """创建可视化图表"""
        try:
            import matplotlib.pyplot as plt
            
            baseline_metrics = self.extract_baseline_metrics()
            realtime_metrics = self.extract_realtime_metrics()
            
            if not baseline_metrics or not realtime_metrics:
                print("⚠️  Insufficient data for visualization")
                return
            
            # 创建对比图
            fig, axes = plt.subplots(1, 2, figsize=(15, 6))
            
            # Speech to first response comparison
            if 'speech_to_first_response' in baseline_metrics and 'speech_to_first_response' in realtime_metrics:
                ax = axes[0]
                baseline_vals = baseline_metrics['speech_to_first_response']['values']
                realtime_vals = realtime_metrics['speech_to_first_response']['values']
                
                ax.boxplot([baseline_vals, realtime_vals], 
                          labels=['Baseline\n(STT→LLM→TTS)', 'Realtime\n(OpenAI API)'])
                ax.set_title('Speech to First Response Latency')
                ax.set_ylabel('Latency (ms)')
                ax.grid(True, alpha=0.3)
            
            # Total response time comparison  
            if 'total_response_time' in baseline_metrics and 'total_response_time' in realtime_metrics:
                ax = axes[1]
                baseline_vals = baseline_metrics['total_response_time']['values']
                realtime_vals = realtime_metrics['total_response_time']['values']
                
                ax.boxplot([baseline_vals, realtime_vals], 
                          labels=['Baseline\n(STT→LLM→TTS)', 'Realtime\n(OpenAI API)'])
                ax.set_title('Total Response Time')
                ax.set_ylabel('Latency (ms)')
                ax.grid(True, alpha=0.3)
            
            plt.tight_layout()
            plt.savefig(f'{output_dir}/performance_comparison.png', dpi=300, bbox_inches='tight')
            plt.close()
            
            print(f"✅ Visualization saved to {output_dir}/performance_comparison.png")
            
        except ImportError:
            print("⚠️  matplotlib not available, skipping visualization")
        except Exception as e:
            print(f"❌ Error creating visualization: {e}")


def main():
    parser = argparse.ArgumentParser(description='Compare baseline vs realtime performance')
    parser.add_argument('--baseline', required=True, help='Baseline results JSON file')
    parser.add_argument('--realtime', required=True, help='Realtime results JSON/CSV file')
    parser.add_argument('--output', default='performance_comparison.json', help='Output comparison file')
    parser.add_argument('--viz', action='store_true', help='Create visualization')
    
    args = parser.parse_args()
    
    # 检查输入文件
    if not Path(args.baseline).exists():
        print(f"❌ Baseline file not found: {args.baseline}")
        return 1
    
    if not Path(args.realtime).exists():
        print(f"❌ Realtime file not found: {args.realtime}")
        return 1
    
    print("🔄 Performance Comparison Analysis")
    print("=" * 40)
    
    # 创建比较分析器
    comparator = PerformanceComparison(args.baseline, args.realtime)
    
    # 加载数据
    if not comparator.load_baseline_data():
        return 1
    if not comparator.load_realtime_data():
        return 1
    
    # 生成对比报告
    print("📊 Generating comparison report...")
    report = comparator.generate_comparison_report(args.output)
    
    # 创建可视化
    if args.viz:
        output_dir = Path(args.output).parent
        comparator.create_visualization(output_dir)
    
    # 显示结果摘要
    print("\n📋 Comparison Summary:")
    if 'primary_latency_improvement' in report['summary']:
        summary = report['summary']['primary_latency_improvement']
        print(f"  Baseline (STT→LLM→TTS): {summary['baseline_median_ms']:.1f}ms median")
        print(f"  Realtime (OpenAI API): {summary['realtime_median_ms']:.1f}ms median")
        print(f"  Improvement: {summary['improvement_percentage']:.1f}% ({summary['improvement_absolute_ms']:.1f}ms faster)")
    
    print(f"\n✅ Comparison complete! Results saved to:")
    print(f"   JSON: {args.output}")
    print(f"   Report: {args.output.replace('.json', '_report.txt')}")
    
    return 0


if __name__ == '__main__':
    exit(main())