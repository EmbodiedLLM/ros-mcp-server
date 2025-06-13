.PHONY: help install install-server install-client install-dev install-all dev test test-docker build deploy deploy-nginx up down logs clean

# 默认目标
help:
	@echo "ROS MCP Server - 可用命令:"
	@echo ""
	@echo "依赖安装:"
	@echo "  install         - 安装基础依赖"
	@echo "  install-server  - 安装服务器依赖"
	@echo "  install-client  - 安装客户端依赖"
	@echo "  install-dev     - 安装开发依赖"
	@echo "  install-all     - 安装所有依赖"
	@echo ""
	@echo "开发相关:"
	@echo "  dev             - 启动开发服务器"
	@echo "  test            - 运行测试"
	@echo ""
	@echo "Docker相关:"
	@echo "  build           - 构建Docker镜像"
	@echo "  deploy          - 一键Docker部署"
	@echo "  deploy-nginx    - 带Nginx的Docker部署"
	@echo "  up              - 启动Docker服务"
	@echo "  down            - 停止Docker服务"
	@echo "  logs            - 查看Docker日志"
	@echo "  test-docker     - 测试Docker环境"
	@echo ""
	@echo "维护相关:"
	@echo "  clean           - 清理Docker资源"

# 安装基础依赖
install:
	uv sync

# 安装服务器依赖
install-server:
	uv sync --group server

# 安装客户端依赖
install-client:
	uv sync --group client

# 安装开发依赖
install-dev:
	uv sync --group dev

# 安装所有依赖
install-all:
	uv sync --group all

# 启动开发服务器
dev:
	uv run server.py

# 运行测试
test:
	uv run tests/quick_test.py

# Docker构建
build:
	docker-compose build

# 一键部署
deploy:
	./deploy-docker.sh

# 带Nginx的部署
deploy-nginx:
	./deploy-docker.sh --with-nginx

# 启动Docker服务
up:
	docker-compose up -d

# 停止Docker服务
down:
	docker-compose down

# 查看日志
logs:
	docker-compose logs -f ros-mcp-server

# Docker环境测试
test-docker:
	uv run tests/test-docker.py

# 清理Docker资源
clean:
	docker-compose down -v
	docker system prune -f 