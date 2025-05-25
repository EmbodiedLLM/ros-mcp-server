#!/bin/bash

# ROS MCP Server Docker 一键部署脚本
set -e

echo "🚀 ROS MCP Server Docker 部署脚本"
echo "=================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 函数：打印彩色消息
print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

# 检查Docker和Docker Compose
check_dependencies() {
    print_info "检查依赖..."
    
    if ! command -v docker &> /dev/null; then
        print_error "Docker 未安装，请先安装 Docker"
        exit 1
    fi
    
    if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null; then
        print_error "Docker Compose 未安装，请先安装 Docker Compose"
        exit 1
    fi
    
    print_success "依赖检查通过"
}

# 创建必要的目录
create_directories() {
    print_info "创建必要的目录..."
    
    mkdir -p logs
    mkdir -p downloads
    mkdir -p config
    mkdir -p nginx/ssl
    
    print_success "目录创建完成"
}

# 复制环境变量文件
setup_env() {
    if [ ! -f .env ]; then
        if [ -f env.example ]; then
            print_info "复制环境变量配置文件..."
            cp env.example .env
            print_warning "请编辑 .env 文件以配置你的环境变量"
        else
            print_warning "未找到 env.example 文件，将使用默认配置"
        fi
    else
        print_info "环境变量文件已存在"
    fi
}

# 构建和启动服务
deploy() {
    local profile=""
    
    # 检查是否启用nginx
    if [ "$1" = "--with-nginx" ]; then
        profile="--profile nginx"
        print_info "启用 Nginx 反向代理"
    fi
    
    print_info "构建 Docker 镜像..."
    docker-compose build
    
    print_info "启动服务..."
    docker-compose up -d $profile
    
    print_success "服务启动完成"
}

# 显示服务状态
show_status() {
    print_info "服务状态："
    docker-compose ps
    
    echo ""
    print_info "服务日志（最近10行）："
    docker-compose logs --tail=10 ros-mcp-server
}

# 显示连接信息
show_connection_info() {
    local host=$(hostname -I | awk '{print $1}' 2>/dev/null || echo "localhost")
    local port=$(grep MCP_PORT .env 2>/dev/null | cut -d'=' -f2 || echo "8000")
    
    echo ""
    print_success "🎉 部署完成！"
    echo "=================================="
    echo ""
    print_info "MCP 服务器连接信息："
    echo "  URL: http://${host}:${port}/mcp"
    echo "  本地: http://localhost:${port}/mcp"
    echo ""
    print_info "Claude Desktop 配置："
    echo '  {'
    echo '    "mcpServers": {'
    echo '      "ros-mcp-server": {'
    echo "        \"url\": \"http://${host}:${port}/mcp\""
    echo '      }'
    echo '    }'
    echo '  }'
    echo ""
    print_info "常用命令："
    echo "  查看日志: docker-compose logs -f"
    echo "  停止服务: docker-compose down"
    echo "  重启服务: docker-compose restart"
    echo "  更新服务: ./deploy-docker.sh"
}

# 测试连接
test_connection() {
    local port=$(grep MCP_PORT .env 2>/dev/null | cut -d'=' -f2 || echo "8000")
    
    print_info "等待服务启动..."
    sleep 10
    
    print_info "测试连接..."
    if curl -f -s http://localhost:${port}/mcp > /dev/null; then
        print_success "服务连接测试成功"
    else
        print_warning "服务连接测试失败，请检查日志"
        print_info "查看日志: docker-compose logs ros-mcp-server"
    fi
}

# 主函数
main() {
    echo ""
    
    # 解析参数
    local with_nginx=false
    local skip_test=false
    
    for arg in "$@"; do
        case $arg in
            --with-nginx)
                with_nginx=true
                ;;
            --skip-test)
                skip_test=true
                ;;
            --help|-h)
                echo "用法: $0 [选项]"
                echo ""
                echo "选项:"
                echo "  --with-nginx    启用 Nginx 反向代理"
                echo "  --skip-test     跳过连接测试"
                echo "  --help, -h      显示帮助信息"
                echo ""
                echo "示例:"
                echo "  $0                    # 基本部署"
                echo "  $0 --with-nginx       # 带 Nginx 的部署"
                echo "  $0 --skip-test        # 跳过测试的部署"
                exit 0
                ;;
        esac
    done
    
    # 执行部署步骤
    check_dependencies
    create_directories
    setup_env
    
    if [ "$with_nginx" = true ]; then
        deploy --with-nginx
    else
        deploy
    fi
    
    show_status
    
    if [ "$skip_test" = false ]; then
        test_connection
    fi
    
    show_connection_info
}

# 运行主函数
main "$@" 