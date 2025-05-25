@echo off
REM ROS MCP Server - 依赖安装脚本 (Windows)
REM 根据不同使用场景安装相应的依赖

setlocal enabledelayedexpansion

REM 检查uv是否安装
where uv >nul 2>nul
if %errorlevel% neq 0 (
    echo [ERROR] uv is not installed. Please install it first:
    echo powershell -c "irm https://astral.sh/uv/install.ps1 | iex"
    exit /b 1
)

REM 显示帮助信息
if "%1"=="" goto :show_help
if "%1"=="help" goto :show_help
if "%1"=="-h" goto :show_help
if "%1"=="--help" goto :show_help

REM 安装依赖
if "%1"=="server" goto :install_server
if "%1"=="client" goto :install_client
if "%1"=="dev" goto :install_dev
if "%1"=="all" goto :install_all

echo [ERROR] Unknown dependency group: %1
goto :show_help

:install_server
echo [INFO] Installing server dependencies...
uv sync --group server
if %errorlevel% equ 0 (
    echo [SUCCESS] Server dependencies installed successfully!
    echo [INFO] You can now run: python server.py
) else (
    echo [ERROR] Failed to install server dependencies
    exit /b 1
)
goto :end

:install_client
echo [INFO] Installing client dependencies...
uv sync --group client
if %errorlevel% equ 0 (
    echo [SUCCESS] Client dependencies installed successfully!
    echo [INFO] You can now run: python stdio_wrapper.py or python start_stdio.py
) else (
    echo [ERROR] Failed to install client dependencies
    exit /b 1
)
goto :end

:install_dev
echo [INFO] Installing development dependencies...
uv sync --group dev
if %errorlevel% equ 0 (
    echo [SUCCESS] Development dependencies installed successfully!
    echo [INFO] You can now run tests and development tools
) else (
    echo [ERROR] Failed to install development dependencies
    exit /b 1
)
goto :end

:install_all
echo [INFO] Installing all dependencies...
uv sync --group all
if %errorlevel% equ 0 (
    echo [SUCCESS] All dependencies installed successfully!
    echo [INFO] Full development environment is ready!
) else (
    echo [ERROR] Failed to install all dependencies
    exit /b 1
)
goto :end

:show_help
echo ROS MCP Server - 依赖安装脚本
echo.
echo 用法: %0 [选项]
echo.
echo 选项:
echo   server    安装服务器依赖 (用于运行ROS MCP服务器)
echo   client    安装客户端依赖 (用于stdio包装器和客户端工具)
echo   dev       安装开发依赖 (用于测试和开发)
echo   all       安装所有依赖
echo   help      显示此帮助信息
echo.
echo 示例:
echo   %0 server    # 只安装服务器依赖
echo   %0 client    # 只安装客户端依赖
echo   %0 dev       # 安装开发依赖
echo   %0 all       # 安装所有依赖
goto :end

:end
endlocal 