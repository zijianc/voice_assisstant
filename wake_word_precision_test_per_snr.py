#!/usr/bin/env python3
"""
Enhanced Wake-Word Precision/Recall Test with Per-SNR Analysis
Tests wake-word detection performance across different SNR levels
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time
import json
import csv
from datetime import datetime
import threading
import random
import argparse
import numpy as np

class WakeWordPerSNRTest(Node):
    def __init__(self, snr_levels=[20, 10, 0], samples_per_category=12):
        super().__init__('wake_word_per_snr_test')
        self.get_logger().info('Starting per-SNR wake-word precision test')
        
        self.snr_levels = snr_levels
        self.samples_per_category = samples_per_category
        self.test_results = []
        self.current_test = {}
        self.test_id = 0
        
        # Subscribe to speech text topic
        self.speech_sub = self.create_subscription(
            String, 'speech_text', self.speech_callback, 10)
        
        # Publish test speech text
        self.speech_pub = self.create_publisher(String, 'speech_text', 10)
        
        # Test samples
        self.positive_samples = [
            "new way 4, what's the weather?",
            "new way four, tell me the time", 
            "neway 4, show me the menu",
            "new way 4 please help me",
            "hey new way 4, what's happening?",
            "new way 4, when does the shuttle arrive?",
            "new way four, what dining options are available?",
            "neway four, what events are happening?",
            "new way 4, tell me about parking",
            "new way 4, what's the library schedule?",
            "new way 4, how's the traffic?",
            "new way four, find me directions"
        ]
        
        self.negative_samples = [
            "what's the weather like today?",
            "tell me the current time",
            "show me the restaurant menu",
            "I need some help here",
            "what's happening around campus?",
            "when does the next bus arrive?",
            "what dining options do we have?",
            "are there any events today?",
            "where can I park my car?",
            "what are the library hours?",
            "how is traffic right now?",
            "can you give me directions?"
        ]
        
        self.near_miss_samples = [
            "we need a new way to solve this",
            "the new way forward is clear",  
            "way 4 is blocked today",
            "new way to campus is open",
            "the way 4 ward is closed",
            "take the new way home",
            "way 4 traffic is heavy",
            "new way of thinking needed",
            "the way 4 you is blocked", 
            "find a new way around",
            "way 4 construction ongoing",
            "new way system installed"
        ]
        
        self.response_received = False
        self.timeout_duration = 3.0
        
    def speech_callback(self, msg):
        """Handle speech recognition results"""
        if self.current_test:
            self.current_test['detected'] = True
            self.current_test['response_text'] = msg.data
            self.current_test['response_time'] = time.time() - self.current_test['start_time']
            self.response_received = True
            
    def run_test_sample(self, sample_text, sample_type, snr_level):
        """Run a single test sample"""
        self.current_test = {
            'test_id': self.test_id,
            'snr_level': snr_level,
            'sample_type': sample_type,
            'sample_text': sample_text,
            'detected': False,
            'response_text': '',
            'response_time': 0,
            'start_time': time.time(),
            'timestamp': datetime.now().isoformat()
        }
        
        self.response_received = False
        
        # Publish the test utterance
        msg = String()
        msg.data = sample_text
        self.speech_pub.publish(msg)
        
        self.get_logger().info(f'SNR {snr_level}dB, {sample_type}: "{sample_text}"')
        
        # Wait for response or timeout
        start_wait = time.time()
        while not self.response_received and (time.time() - start_wait) < self.timeout_duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if not self.response_received:
            self.current_test['timeout'] = True
            self.get_logger().info(f'Timeout for test {self.test_id}')
        else:
            self.current_test['timeout'] = False
            
        # Record results
        self.test_results.append(self.current_test.copy())
        self.test_id += 1
        
        # Brief pause between tests
        time.sleep(1.0)
        
    def run_all_tests(self):
        """Run comprehensive per-SNR testing"""
        self.get_logger().info(f'Starting comprehensive per-SNR wake-word testing')
        self.get_logger().info(f'SNR levels: {self.snr_levels} dB')  
        self.get_logger().info(f'Samples per category per SNR: {self.samples_per_category}')
        
        total_tests = len(self.snr_levels) * self.samples_per_category * 3  # 3 categories
        self.get_logger().info(f'Total tests to run: {total_tests}')
        
        for snr_level in self.snr_levels:
            self.get_logger().info(f'\n=== Testing at SNR {snr_level} dB ===')
            
            # Test positive samples
            self.get_logger().info(f'Testing {self.samples_per_category} positive samples at {snr_level} dB')
            for i in range(self.samples_per_category):
                sample = self.positive_samples[i % len(self.positive_samples)]
                self.run_test_sample(sample, 'positive', snr_level)
                
            # Test negative samples  
            self.get_logger().info(f'Testing {self.samples_per_category} negative samples at {snr_level} dB')
            for i in range(self.samples_per_category):
                sample = self.negative_samples[i % len(self.negative_samples)]
                self.run_test_sample(sample, 'negative', snr_level)
                
            # Test near-miss samples
            self.get_logger().info(f'Testing {self.samples_per_category} near-miss samples at {snr_level} dB')
            for i in range(self.samples_per_category):
                sample = self.near_miss_samples[i % len(self.near_miss_samples)]
                self.run_test_sample(sample, 'near_miss', snr_level)
                
        self.get_logger().info(f'\nCompleted all {len(self.test_results)} tests')
        
    def calculate_metrics_per_snr(self):
        """Calculate precision, recall, F1 per SNR level"""
        metrics = {}
        
        for snr_level in self.snr_levels:
            snr_results = [r for r in self.test_results if r['snr_level'] == snr_level]
            
            # Count outcomes
            tp = len([r for r in snr_results if r['sample_type'] == 'positive' and r['detected']])
            fp = len([r for r in snr_results if r['sample_type'] in ['negative', 'near_miss'] and r['detected']])  
            fn = len([r for r in snr_results if r['sample_type'] == 'positive' and not r['detected']])
            tn = len([r for r in snr_results if r['sample_type'] in ['negative', 'near_miss'] and not r['detected']])
            
            # Calculate metrics
            precision = tp / (tp + fp) if (tp + fp) > 0 else 0
            recall = tp / (tp + fn) if (tp + fn) > 0 else 0
            f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0
            
            metrics[snr_level] = {
                'tp': tp, 'fp': fp, 'fn': fn, 'tn': tn,
                'precision': precision,
                'recall': recall, 
                'f1': f1,
                'total_samples': len(snr_results)
            }
            
        return metrics
        
    def save_results(self):
        """Save detailed results and metrics"""
        # Save raw results
        with open('wake_word_per_snr_results.csv', 'w', newline='') as f:
            if self.test_results:
                writer = csv.DictWriter(f, fieldnames=self.test_results[0].keys())
                writer.writeheader()
                writer.writerows(self.test_results)
        
        # Calculate and save metrics
        metrics = self.calculate_metrics_per_snr()
        
        # Save metrics summary
        with open('wake_word_per_snr_metrics.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['SNR_dB', 'Precision', 'Recall', 'F1', 'TP', 'FP', 'FN', 'TN', 'Total_Samples'])
            for snr_level, m in metrics.items():
                writer.writerow([snr_level, f"{m['precision']:.3f}", f"{m['recall']:.3f}", 
                               f"{m['f1']:.3f}", m['tp'], m['fp'], m['fn'], m['tn'], m['total_samples']])
                
        # Save summary statistics  
        with open('wake_word_per_snr_summary.txt', 'w') as f:
            f.write(f"Wake-Word Per-SNR Test Results\n")
            f.write(f"===============================\n\n")
            f.write(f"Test Configuration:\n")
            f.write(f"- SNR levels tested: {self.snr_levels} dB\n")
            f.write(f"- Samples per category per SNR: {self.samples_per_category}\n")
            f.write(f"- Total tests: {len(self.test_results)}\n\n")
            
            f.write(f"Results by SNR Level:\n")
            for snr_level, m in metrics.items():
                f.write(f"\nSNR {snr_level} dB:\n")
                f.write(f"  Precision: {m['precision']:.3f} ({m['tp']}/{m['tp'] + m['fp']})\n")
                f.write(f"  Recall:    {m['recall']:.3f} ({m['tp']}/{m['tp'] + m['fn']})\n")
                f.write(f"  F1 Score:  {m['f1']:.3f}\n")
                f.write(f"  Samples:   {m['total_samples']}\n")
                
        self.get_logger().info('Results saved to wake_word_per_snr_*.csv/txt')
        return metrics

def main():
    parser = argparse.ArgumentParser(description='Wake-word per-SNR precision/recall test')
    parser.add_argument('--snr-list', nargs='+', type=int, default=[20, 10, 0],
                       help='SNR levels to test (default: 20 10 0)')
    parser.add_argument('--samples-per-category', type=int, default=12,
                       help='Number of samples per category per SNR (default: 12)')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        test_node = WakeWordPerSNRTest(args.snr_list, args.samples_per_category)
        
        # Allow time for subscriptions to establish
        time.sleep(2.0)
        
        # Run all tests
        test_node.run_all_tests()
        
        # Calculate and save results
        metrics = test_node.save_results()
        
        # Print summary
        print(f"\n=== Wake-Word Per-SNR Test Summary ===")
        for snr_level, m in metrics.items():
            print(f"SNR {snr_level} dB: P={m['precision']:.3f}, R={m['recall']:.3f}, F1={m['f1']:.3f} (n={m['total_samples']})")
            
    except KeyboardInterrupt:
        print("Test interrupted by user")
        
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()