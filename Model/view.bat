@echo off
chcp 65001 >nul
cd /d "%~dp0yolo3_smartcar"

echo ========================================
echo    SmartCar 标注数据查看器
echo ========================================
echo.
echo 读取 VOC XML 标注和图片，生成 HTML 浏览页面。
echo.

set /p IMAGE_DIR=请输入图片目录 (直接回车=..\data\JPEGImages):
if "%IMAGE_DIR%"=="" set IMAGE_DIR=..\data\JPEGImages

set /p ANN_DIR=请输入标注目录 (直接回车=%IMAGE_DIR%\..\Annotations):
if "%ANN_DIR%"=="" set ANN_DIR=%IMAGE_DIR%\..\Annotations

set /p OUTPUT_DIR=请输入输出目录 (直接回车=..\view_output):
if "%OUTPUT_DIR%"=="" set OUTPUT_DIR=..\view_output

echo.
echo 正在生成缩略图和 HTML 页面...
echo.

py -3.10 view_annotations.py -image "%IMAGE_DIR%" -ann "%ANN_DIR%" -output "%OUTPUT_DIR%"
if %errorlevel% neq 0 (
    echo.
    echo [错误] 生成失败，请检查错误信息。
    pause
    exit /b %errorlevel%
)

echo.
echo 正在打开浏览器...
for %%i in ("%OUTPUT_DIR%") do set "OUTPUT_FULL=%%~fi"
start "" "%OUTPUT_FULL%\index.html"

echo.
echo ========================================
echo    完成!
echo ========================================
pause
