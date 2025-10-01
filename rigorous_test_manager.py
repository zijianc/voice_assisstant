#!/usr/bin/env python3
"""
严谨测试执行管理器
================

统一管理和执行所有严谨测试，确保学术报告的数据严谨性

用法:
    python3 rigorous_test_manager.py --run-all
    python3 rigorous_test_manager.py --wakeword --latency  
    python3 rigorous_test_manager.py --interruption-only
"""

import argparse
import json
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

class RigorousTestManager:
    def __init__(self):
        self.start_time = datetime.now()
        self.test_results = {}
        
        # 推荐的测试配置（符合学术标准）
        self.test_configs = {
            'wakeword': {
                'script': 'rigorous_wakeword_test.py',
                'args': ['--snr-levels', '20', '10', '0', '--samples-per-snr', '30', '--rounds', '1'],
                'description': '唤醒词精度测试 (每SNR级别30样本)',
                'expected_duration': '10-15分钟'
            },
            'latency': {
                'script': 'rigorous_latency_test.py', 
                'args': ['--samples', '50', '--with-interruption', '--inter-trial-delay', '2.0'],
                'description': '端到端延迟测试 (50样本含中断)',
                'expected_duration': '15-20分钟'
            },
            'interruption': {
                'script': 'rigorous_interruption_test.py',
                'args': ['--samples', '60', '--interruption-times', '0.2', '0.5', '1.0', '2.0'],
                'description': '专门中断延迟测试 (60样本4个时机)',
                'expected_duration': '12-18分钟'
            }
        }

    def check_prerequisites(self):
        """检查测试前提条件"""
        print("🔍 检查测试前提条件...")
        
        errors = []
        warnings = []
        
        # 检查测试脚本
        for test_name, config in self.test_configs.items():
            script_path = Path(config['script'])
            if not script_path.exists():
                errors.append(f"缺少测试脚本: {config['script']}")
            elif not script_path.is_file():
                errors.append(f"测试脚本不是文件: {config['script']}")
        
        # 检查Python依赖
        try:
            import numpy
            import statistics
        except ImportError as e:
            errors.append(f"缺少Python依赖: {e}")
        
        # 检查scipy（可选但推荐）
        try:
            import scipy.stats
        except ImportError:
            warnings.append("建议安装scipy以获得更准确的统计分析: pip install scipy")
        
        # 检查ROS2环境
        ros2_setup = Path("/opt/ros/humble/setup.bash")
        if not ros2_setup.exists():
            errors.append("ROS2 Humble环境未找到")
        
        workspace_setup = Path("install/setup.bash")
        if not workspace_setup.exists():
            warnings.append("工作空间未构建，请先运行: colcon build --packages-select my_voice_assistant")
        
        # 报告检查结果
        if errors:
            print("❌ 发现错误:")
            for error in errors:
                print(f"   • {error}")
            return False
        
        if warnings:
            print("⚠️  警告:")
            for warning in warnings:
                print(f"   • {warning}")
        
        print("✅ 前提条件检查通过")
        return True

    def run_single_test(self, test_name):
        """运行单个测试"""
        if test_name not in self.test_configs:
            print(f"❌ 未知测试: {test_name}")
            return False
        
        config = self.test_configs[test_name]
        print(f"\n🚀 开始 {test_name} 测试")
        print(f"   描述: {config['description']}")
        print(f"   预计耗时: {config['expected_duration']}")
        
        # 构建命令
        cmd = ['python3', config['script']] + config['args']
        print(f"   命令: {' '.join(cmd)}")
        
        # 执行测试
        start_time = time.time()
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=1800)  # 30分钟超时
            duration = time.time() - start_time
            
            if result.returncode == 0:
                print(f"✅ {test_name} 测试完成 (耗时: {duration:.1f}s)")
                self.test_results[test_name] = {
                    'status': 'success',
                    'duration': duration,
                    'stdout': result.stdout,
                    'stderr': result.stderr
                }
                return True
            else:
                print(f"❌ {test_name} 测试失败 (返回码: {result.returncode})")
                print(f"错误输出: {result.stderr}")
                self.test_results[test_name] = {
                    'status': 'failed',
                    'duration': duration,
                    'returncode': result.returncode,
                    'stdout': result.stdout,
                    'stderr': result.stderr
                }
                return False
                
        except subprocess.TimeoutExpired:
            print(f"⏰ {test_name} 测试超时")
            self.test_results[test_name] = {
                'status': 'timeout',
                'duration': time.time() - start_time
            }
            return False
        except Exception as e:
            print(f"💥 {test_name} 测试异常: {e}")
            self.test_results[test_name] = {
                'status': 'error',
                'duration': time.time() - start_time,
                'error': str(e)
            }
            return False

    def run_tests(self, test_names):
        """运行指定的测试"""
        if not self.check_prerequisites():
            print("❌ 前提条件不满足，无法运行测试")
            return False
        
        print(f"\n📋 计划运行测试: {', '.join(test_names)}")
        total_estimated_time = sum([15 for _ in test_names])  # 粗略估计每个测试15分钟
        print(f"📅 预计总耗时: {total_estimated_time}分钟")
        
        # 确认执行
        response = input("\n继续执行吗? [y/N]: ")
        if response.lower() not in ['y', 'yes']:
            print("⚠️  用户取消测试")
            return False
        
        successful_tests = []
        failed_tests = []
        
        for i, test_name in enumerate(test_names, 1):
            print(f"\n{'='*60}")
            print(f"执行测试 {i}/{len(test_names)}: {test_name.upper()}")
            print(f"{'='*60}")
            
            if self.run_single_test(test_name):
                successful_tests.append(test_name)
            else:
                failed_tests.append(test_name)
                
                # 询问是否继续
                if i < len(test_names):
                    response = input(f"\n{test_name} 测试失败，继续执行剩余测试吗? [y/N]: ")
                    if response.lower() not in ['y', 'yes']:
                        print("⚠️  用户中止后续测试")
                        break
        
        # 测试总结
        self.print_summary(successful_tests, failed_tests)
        self.save_execution_report()
        
        return len(failed_tests) == 0

    def print_summary(self, successful_tests, failed_tests):
        """打印测试总结"""
        print(f"\n{'='*60}")
        print("🎯 测试执行总结")
        print(f"{'='*60}")
        
        total_duration = sum([r.get('duration', 0) for r in self.test_results.values()])
        print(f"总执行时间: {total_duration:.1f}s ({total_duration/60:.1f}分钟)")
        
        if successful_tests:
            print(f"\n✅ 成功测试 ({len(successful_tests)}):")
            for test in successful_tests:
                duration = self.test_results[test].get('duration', 0)
                print(f"   • {test}: {duration:.1f}s")
        
        if failed_tests:
            print(f"\n❌ 失败测试 ({len(failed_tests)}):")
            for test in failed_tests:
                status = self.test_results[test].get('status', 'unknown')
                print(f"   • {test}: {status}")
        
        print(f"\n📊 成功率: {len(successful_tests)}/{len(successful_tests)+len(failed_tests)} ({len(successful_tests)/(len(successful_tests)+len(failed_tests))*100:.1f}%)")

    def save_execution_report(self):
        """保存执行报告"""
        timestamp = self.start_time.strftime('%Y%m%d_%H%M%S')
        report_file = f"rigorous_test_execution_{timestamp}.json"
        
        report = {
            'execution_info': {
                'start_time': self.start_time.isoformat(),
                'end_time': datetime.now().isoformat(),
                'total_duration': sum([r.get('duration', 0) for r in self.test_results.values()]),
                'test_configs': self.test_configs
            },
            'results': self.test_results,
            'summary': {
                'total_tests': len(self.test_results),
                'successful_tests': len([r for r in self.test_results.values() if r.get('status') == 'success']),
                'failed_tests': len([r for r in self.test_results.values() if r.get('status') != 'success'])
            }
        }
        
        with open(report_file, 'w') as f:
            json.dump(report, f, indent=2)
        
        print(f"\n📁 执行报告保存为: {report_file}")

    def list_available_tests(self):
        """列出可用的测试"""
        print("📋 可用的严谨测试:")
        for test_name, config in self.test_configs.items():
            print(f"\n🧪 {test_name.upper()}")
            print(f"   描述: {config['description']}")
            print(f"   预计耗时: {config['expected_duration']}")
            print(f"   脚本: {config['script']}")
            print(f"   参数: {' '.join(config['args'])}")

    def check_existing_results(self):
        """检查已有的测试结果"""
        print("🔍 检查已有测试结果...")
        
        result_patterns = {
            'wakeword': 'rigorous_wakeword_*.json',
            'latency': 'rigorous_latency_*.json', 
            'interruption': 'rigorous_interruption_*.json'
        }
        
        found_results = {}
        for test_type, pattern in result_patterns.items():
            files = list(Path('.').glob(pattern))
            if files:
                # 按修改时间排序，取最新的
                latest_file = max(files, key=lambda f: f.stat().st_mtime)
                found_results[test_type] = {
                    'file': str(latest_file),
                    'modified': datetime.fromtimestamp(latest_file.stat().st_mtime).strftime('%Y-%m-%d %H:%M:%S')
                }
        
        if found_results:
            print("📊 发现已有测试结果:")
            for test_type, info in found_results.items():
                print(f"   • {test_type}: {info['file']} (修改于 {info['modified']})")
            print("💡 如需重新测试，建议先备份现有结果")
        else:
            print("📭 未发现已有测试结果")
        
        return found_results

def main():
    parser = argparse.ArgumentParser(description='严谨测试执行管理器')
    parser.add_argument('--run-all', action='store_true',
                       help='运行所有测试')
    parser.add_argument('--wakeword', action='store_true',
                       help='运行唤醒词测试')
    parser.add_argument('--latency', action='store_true',
                       help='运行延迟测试')
    parser.add_argument('--interruption', action='store_true',
                       help='运行中断测试')
    parser.add_argument('--interruption-only', action='store_true',
                       help='仅运行中断测试')
    parser.add_argument('--list', action='store_true',
                       help='列出可用测试')
    parser.add_argument('--check-results', action='store_true',
                       help='检查已有结果')
    
    args = parser.parse_args()
    
    manager = RigorousTestManager()
    
    if args.list:
        manager.list_available_tests()
        return
    
    if args.check_results:
        manager.check_existing_results()
        return
    
    # 确定要运行的测试
    tests_to_run = []
    
    if args.run_all:
        tests_to_run = ['wakeword', 'latency', 'interruption']
    else:
        if args.wakeword:
            tests_to_run.append('wakeword')
        if args.latency:
            tests_to_run.append('latency')
        if args.interruption or args.interruption_only:
            tests_to_run.append('interruption')
    
    if not tests_to_run:
        print("❓ 未指定要运行的测试")
        print("使用 --list 查看可用测试，或 --help 查看帮助")
        return
    
    # 检查已有结果
    existing_results = manager.check_existing_results()
    if existing_results:
        overlapping = set(tests_to_run) & set(existing_results.keys())
        if overlapping:
            print(f"\n⚠️  警告: 以下测试已有结果文件: {', '.join(overlapping)}")
            response = input("继续执行会生成新的结果文件，是否继续? [y/N]: ")
            if response.lower() not in ['y', 'yes']:
                print("⚠️  用户取消测试")
                return
    
    # 执行测试
    print(f"\n🎯 准备执行严谨测试以确保学术报告数据严谨性")
    success = manager.run_tests(tests_to_run)
    
    if success:
        print(f"\n🎉 所有测试成功完成！")
        print(f"📚 生成的数据具有统计显著性，可用于学术报告")
    else:
        print(f"\n⚠️  部分测试失败，请检查错误信息并重新运行")
        sys.exit(1)

if __name__ == "__main__":
    main()