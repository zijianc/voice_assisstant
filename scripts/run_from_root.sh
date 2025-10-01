#!/bin/bash
# 便捷脚本运行器 - 从项目根目录运行任何脚本
# 用法: ./scripts/run_from_root.sh <script_path> [args...]

# 获取项目根目录
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 检查参数
if [ $# -lt 1 ]; then
    echo "用法: $0 <script_path> [args...]"
    echo "例如: $0 scripts/test/run_baseline_test.sh --samples 10"
    echo "     $0 figures/scripts/create_enhanced_figures.py"
    exit 1
fi

SCRIPT_PATH="$1"
shift  # 移除第一个参数，剩下的都是脚本参数

# 切换到项目根目录
cd "$PROJECT_ROOT"

# 检查脚本是否存在
if [ ! -f "$SCRIPT_PATH" ]; then
    echo "错误: 脚本 '$SCRIPT_PATH' 不存在"
    exit 1
fi

# 根据文件扩展名运行脚本
case "$SCRIPT_PATH" in
    *.py)
        echo "🐍 运行 Python 脚本: $SCRIPT_PATH"
        python3 "$SCRIPT_PATH" "$@"
        ;;
    *.sh)
        echo "📜 运行 Bash 脚本: $SCRIPT_PATH"
        bash "$SCRIPT_PATH" "$@"
        ;;
    *)
        echo "⚡ 直接执行: $SCRIPT_PATH"
        "./$SCRIPT_PATH" "$@"
        ;;
esac