@echo off
chcp 65001 >nul
cd /d "%~dp0yolo3_smartcar"

echo ========================================
echo    YOLOv3 模型训练 & 验证流程
echo ========================================
echo.

echo [1/5] VOC格式转换...
py -3.10 .\voc_convertor.py
if %errorlevel% neq 0 (
    echo [错误] voc_convertor 执行失败!
    pause
    exit /b 1
)
echo.

echo [2/5] K-means聚类锚框计算...
py -3.10 .\kmeans.py
if %errorlevel% neq 0 (
    echo [错误] kmeans 执行失败!
    pause
    exit /b 1
)
echo.

echo [3/5] 开始训练模型...
py -3.10 .\train.py
if %errorlevel% neq 0 (
    echo [错误] train 执行失败!
    pause
    exit /b 1
)
echo.

echo [4/5] 验证模型 (mAP评估)...
py -3.10 .\evaluate.py
if %errorlevel% neq 0 (
    echo [错误] evaluate 执行失败!
    pause
    exit /b 1
)
echo.

echo [5/5] 推理测试...
py -3.10 detect.py -model yolo3_iou_smartcar_final.tflite -image test.jpg
if %errorlevel% neq 0 (
    echo [错误] detect 执行失败!
    pause
    exit /b 1
)
echo.

echo ========================================
echo    全部完成!
echo    模型文件: yolo3_iou_smartcar_final.tflite
echo    检测结果: tflite_detected_img.jpg
echo ========================================
pause
