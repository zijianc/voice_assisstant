from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_voice_assistant'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools', 'vosk', 'sounddevice', 'pyaudio','python-dotenv', 'dashscope', 'requests', 'websockets'],
    zip_safe=True,
    maintainer='zijian',
    maintainer_email='youremail@example.com',
    description='ROS2 voice assistant using Vosk for STT',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vosk_stt_node = my_voice_assistant.vosk_stt_node:main',
            'llm_node = my_voice_assistant.llm_node:main',
            'llm_node_cn = my_voice_assistant.llm_node_cn:main',
            'tts_node = my_voice_assistant.tts_node:main',
            'openai_tts_node = my_voice_assistant.openai_tts_node:main',
            'qwen_tts_node = my_voice_assistant.qwen_tts_node:main',
            'openai_stt_node = my_voice_assistant.realtime_stt_node:main',  # 🆕 修复入口点名称
            'openai_stt_node_with_vad = my_voice_assistant.realtime_stt_node:main',
            'ten_vad_stt_node = my_voice_assistant.ten_vad_stt_node:main',
            'openai_realtime_node = my_voice_assistant.openai_realtime_node:main',  # 🆕 OpenAI Realtime API节点
        ],
    },
)