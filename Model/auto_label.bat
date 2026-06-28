@echo off
chcp 65001 >nul
cd /d "%~dp0yolo3_smartcar"

echo ========================================
echo    自动标注工具
echo ========================================
echo.
echo 使用训练好的模型对图片批量标注，生成 VOC XML 文件。
echo.

set /p IMAGE_DIR=请输入图片目录路径:

:: XML 输出目录默认改为图片路径的 ../Annotations
set /p OUTPUT_DIR=请输入XML输出目录 (直接回车=图片路径的../Annotations):

set /p CONF=置信度阈值 (直接回车=0.12):
if "%CONF%"=="" set CONF=0.12

set /p IOU=NMS IOU阈值 (直接回车=0.45):
if "%IOU%"=="" set IOU=0.45

set /p IMAGE_LIST=指定图片名,逗号分隔 (直接回车=全部):

:: 构建命令
set CMD=py -3.10 auto_label.py -image "%IMAGE_DIR%" -conf %CONF% -iou %IOU%

if "%OUTPUT_DIR%"=="" (
    set CMD=%CMD% -output "%IMAGE_DIR%\..\Annotations"
) else (
    set CMD=%CMD% -output "%OUTPUT_DIR%"
)

if not "%IMAGE_LIST%"=="" (
    set CMD=%CMD% -list "%IMAGE_LIST%"
)

echo.
echo 执行: %CMD%
echo.
%CMD%

echo.
echo ========================================
echo    标注完成!
echo ========================================
pause
