#!/usr/bin/env python3
"""
TEN VAD 参数实时调整工具
=======================

这个工具可以帮助你在不同环境下快速调整TEN VAD参数
使用方法: python ten_vad_tuner.py [场景名称]

支持的场景:
- office: 办公室/会议室
- noisy: 公交车/高噪音环境  
- quiet: 安静家庭环境
- studio: 高质量录音室环境
- custom: 自定义调整模式
"""

import os
import sys
import argparse

# 预定义场景配置
SCENARIOS = {
    "office": {
        "name": "办公室/会议室环境",
        "description": "中等噪音，多人会议场景",
        "config": {
            "TEN_VAD_THRESHOLD": "0.6",
            "TEN_MIN_VOICE_FRAMES": "6", 
            "TEN_MAX_SILENCE_FRAMES": "45",
            "TEN_MIN_AUDIO_ENERGY": "150",
            "WAKE_WORD_SIMILARITY_THRESHOLD": "0.85"
        }
    },
    
    "noisy": {
        "name": "公交车/高噪音环境",
        "description": "强背景噪音，需要严格检测",
        "config": {
            "TEN_VAD_THRESHOLD": "0.7",
            "TEN_MIN_VOICE_FRAMES": "8",
            "TEN_MAX_SILENCE_FRAMES": "40", 
            "TEN_MIN_AUDIO_ENERGY": "300",
            "WAKE_WORD_SIMILARITY_THRESHOLD": "0.88"
        }
    },
    
    "quiet": {
        "name": "安静家庭环境", 
        "description": "低背景噪音，高敏感度检测",
        "config": {
            "TEN_VAD_THRESHOLD": "0.4",
            "TEN_MIN_VOICE_FRAMES": "4",
            "TEN_MAX_SILENCE_FRAMES": "60",
            "TEN_MIN_AUDIO_ENERGY": "80", 
            "WAKE_WORD_SIMILARITY_THRESHOLD": "0.84"
        }
    },
    
    "studio": {
        "name": "高质量录音室环境",
        "description": "专业麦克风，极低噪音",
        "config": {
            "TEN_VAD_THRESHOLD": "0.3",
            "TEN_MIN_VOICE_FRAMES": "3",
            "TEN_MAX_SILENCE_FRAMES": "70",
            "TEN_MIN_AUDIO_ENERGY": "50",
            "WAKE_WORD_SIMILARITY_THRESHOLD": "0.82"
        }
    }
}

def update_env_file(config, env_file=".env"):
    """更新.env文件中的配置"""
    try:
        # 读取现有的.env文件
        existing_lines = []
        if os.path.exists(env_file):
            with open(env_file, 'r') as f:
                existing_lines = f.readlines()
        
        # 创建新的配置字典
        env_dict = {}
        for line in existing_lines:
            line = line.strip()
            if line and not line.startswith('#') and '=' in line:
                key, value = line.split('=', 1)
                env_dict[key.strip()] = value.strip()
        
        # 更新配置
        for key, value in config.items():
            env_dict[key] = value
            
        # 写回文件
        with open(env_file, 'w') as f:
            f.write("# TEN VAD 自动生成配置\n")
            f.write(f"# 生成时间: {os.popen('date').read().strip()}\n\n")
            
            for key, value in env_dict.items():
                f.write(f"{key}={value}\n")
                
        print(f"✅ 配置已更新到 {env_file}")
        return True
        
    except Exception as e:
        print(f"❌ 更新配置文件失败: {e}")
        return False

def interactive_tuning():
    """交互式参数调整"""
    print("🎛️  进入交互式调整模式")
    print("请逐个设置以下参数 (直接按回车保持默认值):\n")
    
    config = {}
    
    # TEN VAD 阈值
    threshold = input("TEN VAD 阈值 (0.0-1.0, 默认0.5): ").strip()
    if threshold:
        try:
            val = float(threshold)
            if 0.0 <= val <= 1.0:
                config["TEN_VAD_THRESHOLD"] = str(val)
            else:
                print("⚠️  阈值必须在0.0-1.0之间")
        except ValueError:
            print("⚠️  无效的数值")
    
    # 最小语音帧数
    min_frames = input("最小语音帧数 (2-15, 默认5): ").strip()
    if min_frames:
        try:
            val = int(min_frames)
            if 2 <= val <= 15:
                config["TEN_MIN_VOICE_FRAMES"] = str(val)
            else:
                print("⚠️  帧数必须在2-15之间")
        except ValueError:
            print("⚠️  无效的数值")
    
    # 最大静音帧数
    max_silence = input("最大静音帧数 (20-100, 默认50): ").strip()
    if max_silence:
        try:
            val = int(max_silence)
            if 20 <= val <= 100:
                config["TEN_MAX_SILENCE_FRAMES"] = str(val)
            else:
                print("⚠️  帧数必须在20-100之间") 
        except ValueError:
            print("⚠️  无效的数值")
    
    # 最小音频能量
    min_energy = input("最小音频能量 (50-500, 默认100): ").strip()
    if min_energy:
        try:
            val = int(min_energy)
            if 50 <= val <= 500:
                config["TEN_MIN_AUDIO_ENERGY"] = str(val)
            else:
                print("⚠️  能量值必须在50-500之间")
        except ValueError:
            print("⚠️  无效的数值")
    
    # 唤醒词相似度
    similarity = input("唤醒词相似度 (0.7-0.95, 默认0.86): ").strip()
    if similarity:
        try:
            val = float(similarity)
            if 0.7 <= val <= 0.95:
                config["WAKE_WORD_SIMILARITY_THRESHOLD"] = str(val)
            else:
                print("⚠️  相似度必须在0.7-0.95之间")
        except ValueError:
            print("⚠️  无效的数值")
    
    if config:
        print(f"\n📝 将要应用的配置:")
        for key, value in config.items():
            print(f"  {key} = {value}")
        
        confirm = input("\n确认应用这些配置? (y/N): ").strip().lower()
        if confirm == 'y':
            if update_env_file(config):
                print("✅ 配置更新完成!")
            else:
                print("❌ 配置更新失败")
        else:
            print("❌ 已取消配置更新")
    else:
        print("📝 没有配置需要更新")

def main():
    parser = argparse.ArgumentParser(description="TEN VAD 参数调整工具")
    parser.add_argument("scenario", nargs='?', choices=list(SCENARIOS.keys()) + ['custom'],
                       help="选择预定义场景或custom进行自定义调整")
    parser.add_argument("--env-file", default=".env", help="环境变量文件路径")
    parser.add_argument("--list", action="store_true", help="列出所有可用场景")
    
    args = parser.parse_args()
    
    # 列出所有场景
    if args.list:
        print("🎯 可用的预定义场景:\n")
        for key, scenario in SCENARIOS.items():
            print(f"📍 {key}: {scenario['name']}")
            print(f"   描述: {scenario['description']}")
            print(f"   参数: VAD阈值={scenario['config']['TEN_VAD_THRESHOLD']}, "
                  f"最小帧={scenario['config']['TEN_MIN_VOICE_FRAMES']}, "
                  f"静音帧={scenario['config']['TEN_MAX_SILENCE_FRAMES']}")
            print()
        print("💡 使用方法: python ten_vad_tuner.py [场景名称]")
        print("💡 自定义调整: python ten_vad_tuner.py custom")
        return
    
    # 没有指定场景时显示帮助
    if not args.scenario:
        print("🎛️  TEN VAD 参数调整工具")
        print("="*50)
        print()
        print("使用 --list 查看所有可用场景")
        print("使用场景名称快速应用预设配置")
        print("使用 custom 进入交互式调整模式")
        print()
        print("示例:")
        print("  python ten_vad_tuner.py office    # 办公室环境")
        print("  python ten_vad_tuner.py noisy     # 嘈杂环境") 
        print("  python ten_vad_tuner.py custom    # 自定义调整")
        return
    
    # 自定义模式
    if args.scenario == "custom":
        interactive_tuning()
        return
    
    # 应用预定义场景
    scenario = SCENARIOS[args.scenario]
    print(f"🎯 应用场景: {scenario['name']}")
    print(f"📝 描述: {scenario['description']}")
    print()
    
    print("📋 将要应用的配置:")
    for key, value in scenario['config'].items():
        print(f"  {key} = {value}")
    print()
    
    confirm = input("确认应用此配置? (Y/n): ").strip().lower()
    if confirm in ['', 'y', 'yes']:
        if update_env_file(scenario['config'], args.env_file):
            print("✅ 场景配置应用成功!")
            print()
            print("🚀 请重启TEN VAD节点以使新配置生效:")
            print("   ./start_ten_vad_stt.sh")
        else:
            print("❌ 配置应用失败")
    else:
        print("❌ 已取消配置应用")

if __name__ == "__main__":
    main()
