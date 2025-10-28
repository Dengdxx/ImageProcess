@echo off
chcp 65001 >nul
REM ============================================
REM 一键编译脚本 (ROS2风格)
REM ============================================

echo.
echo ========================================
echo    ImageProcessor 项目编译
echo ========================================
echo.

REM 进入脚本所在目录
cd /d "%~dp0"

REM 1. 检查并创建 build 目录
echo [1/4] 检查构建目录...
if not exist "build" (
    echo       创建 build 目录...
    mkdir build
) else (
    echo       build 目录已存在
)
echo       √ 完成
echo.

REM 2. 运行 CMake 配置
echo [2/4] 配置项目 (CMake)...
cd build
REM 使用 MinGW Makefiles 生成器，并启用 GUI 构建
cmake .. -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DBUILD_GUI=ON
if errorlevel 1 (
    echo       × CMake 配置失败
    cd ..
    pause
    exit /b 1
)
echo       √ 配置成功
cd ..
echo.

REM 3. 编译项目
echo [3/4] 编译项目...
cmake --build build --config Release -j 16
if errorlevel 1 (
    echo       × 编译失败
    pause
    exit /b 1
)
echo       √ 编译成功
echo.

REM 4. 检查生成的可执行文件
echo [4/4] 检查输出文件...
if exist "install\bin\imageprocessor.exe" (
    echo       √ GUI工具: install\bin\imageprocessor.exe
    set "FOUND=1"
)
if exist "install\bin\video_processor.exe" (
    echo       √ 视频工具: install\bin\video_processor.exe
    set "FOUND=1"
)

if not defined FOUND (
    echo       ! 警告: 未找到可执行文件，请检查编译输出
)
echo.

REM 完成
echo ========================================
echo    编译完成！
echo ========================================
echo.
echo 可执行文件位置:
echo   - GUI工具: install\bin\imageprocessor.exe
echo   - 视频工具: install\bin\video_processor.exe
echo.
echo 使用方法:
echo   运行GUI: run.bat
echo   清理构建: clean.bat
echo.
pause
