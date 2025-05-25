.PHONY: help install dev test test-docker build deploy deploy-nginx up down logs clean

# 默认目标
help:
	@echo "ROS MCP Server - 可用命令:"
	@echo ""
	@echo "开发相关:"
	@echo "  install     - 安装依赖"
	@echo "  dev         - 启动开发服务器"
	@echo "  test        - 运行测试"
	@echo ""
	@echo "Docker相关:"
	@echo "  build       - 构建Docker镜像"
	@echo "  deploy      - 一键Docker部署"
	@echo "  deploy-nginx- 带Nginx的Docker部署"
	@echo "  up          - 启动Docker服务"
	@echo "  down        - 停止Docker服务"
	@echo "  logs        - 查看Docker日志"
	@echo "  test-docker - 测试Docker环境"
	@echo ""
	@echo "维护相关:"
	@echo "  clean       - 清理Docker资源"

# 安装依赖
install:
	uv sync

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