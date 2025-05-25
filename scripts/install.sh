#!/bin/bash
# ROS MCP Server - 依赖安装脚本
# 根据不同使用场景安装相应的依赖

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查uv是否安装
check_uv() {
    if ! command -v uv &> /dev/null; then
        print_error "uv is not installed. Please install it first:"
        echo "curl -LsSf https://astral.sh/uv/install.sh | sh"
        exit 1
    fi
    print_success "uv is installed"
}

# 显示帮助信息
show_help() {
    echo "ROS MCP Server - 依赖安装脚本"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  server    安装服务器依赖 (用于运行ROS MCP服务器)"
    echo "  client    安装客户端依赖 (用于stdio包装器和客户端工具)"
    echo "  dev       安装开发依赖 (用于测试和开发)"
    echo "  all       安装所有依赖"
    echo "  help      显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 server    # 只安装服务器依赖"
    echo "  $0 client    # 只安装客户端依赖"
    echo "  $0 dev       # 安装开发依赖"
    echo "  $0 all       # 安装所有依赖"
}

# 安装依赖
install_deps() {
    local group=$1
    print_info "Installing $group dependencies..."
    
    case $group in
        "server")
            uv sync --group server
            print_success "Server dependencies installed successfully!"
            print_info "You can now run: python server.py"
            ;;
        "client")
            uv sync --group client
            print_success "Client dependencies installed successfully!"
            print_info "You can now run: python stdio_wrapper.py or python start_stdio.py"
            ;;
        "dev")
            uv sync --group dev
            print_success "Development dependencies installed successfully!"
            print_info "You can now run tests and development tools"
            ;;
        "all")
            uv sync --group all
            print_success "All dependencies installed successfully!"
            print_info "Full development environment is ready!"
            ;;
        *)
            print_error "Unknown dependency group: $group"
            show_help
            exit 1
            ;;
    esac
}

# 主函数
main() {
    # 检查uv
    check_uv
    
    # 检查参数
    if [ $# -eq 0 ] || [ "$1" = "help" ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
        show_help
        exit 0
    fi
    
    # 安装依赖
    install_deps "$1"
}

# 运行主函数
main "$@" 