#!/usr/bin/env python3

import re

def filter_content(text: str) -> str:
    """测试内容过滤功能"""
    if not text:
        return text
        
    # 过滤【】括号及其内容（包括数字、符号等）
    # 匹配【任何内容】的模式
    filtered_text = re.sub(r'【[^】]*】', '', text)
    
    # 过滤其他可能的干扰内容
    # 过滤连续的数字和特殊符号组合
    filtered_text = re.sub(r'\b\d{3,}\b', '', filtered_text)  # 移除3位以上连续数字
    filtered_text = re.sub(r'[†‑]+', '', filtered_text)  # 移除特殊符号
    filtered_text = re.sub(r'L\d+-L\d+', '', filtered_text)  # 移除L数字-L数字模式
    
    # 清理多余的空格和标点
    filtered_text = re.sub(r'\s+', ' ', filtered_text)  # 合并多个空格
    filtered_text = re.sub(r'\s*\.\s*\.+', '.', filtered_text)  # 清理多个句号
    filtered_text = filtered_text.strip()
    
    return filtered_text

# 测试用例
test_cases = [
    "I'm Captain, the UWA shuttle bus voice assistant.【742442319583238†L62-L65】.",
    "Hello! Zhu Rongheng is not my father, but he is one of the developers who created me.【464578442067052†L78-L80】.",
    "The REV (Reduced Emissions Vehicle) lab is a student‑run project that designs and builds electric race cars. Since 2009 the team has won multiple international races including the Formula Electric Race Series in 2010 and 2012【222414601972965†L192-L207】.",
    "Normal text without any special characters.",
    "Text with【brackets】in the middle.",
    "Multiple【first】and【second】brackets.",
]

print("🧪 测试内容过滤功能")
print("=" * 60)

for i, test_text in enumerate(test_cases, 1):
    print(f"\n测试 {i}:")
    print(f"原文: {test_text}")
    filtered = filter_content(test_text)
    print(f"过滤后: {filtered}")
    print(f"长度变化: {len(test_text)} -> {len(filtered)}")
