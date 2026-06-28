import tensorflow as tf 
from tensorflow.keras.models import load_model
from tensorflow.keras.layers import Input
import numpy as np 
import os 
from utils import yolo_cfg
from model_resnet import yolo_eval
import cv2
import argparse
from PIL import Image,ImageDraw, ImageFont  # 新增：导入字体模块
from evaluate import get_yolo_boxes


def _sigmoid(x):
    return 1. / (1. + np.exp(-x))
    
def get_anchors(anchors_path):
    '''loads the anchors from a file'''
    with open(anchors_path) as f:
        anchors = f.readline()
    anchors = [float(x) for x in anchors.split(',')]
    return np.array(anchors).reshape(-1, 6)
    
def decode_output(netout, input_shape, image_shape, anchors, conf_thres):
    grid_h, grid_w = netout.shape[:2]
    nb_box = 3
    netout = netout.reshape((grid_h, grid_w, nb_box, -1))
    net_w, net_h = input_shape
    image_w, image_h = image_shape
    boxes = []
    
    # 全部sigmoid归一化（关键！修复置信度爆炸）
    netout[..., :2]  = _sigmoid(netout[..., :2])
    netout[..., 4:]  = _sigmoid(netout[..., 4:])
    
    # Letterbox 灰边补偿（训练时的预处理，必须对齐）
    if (float(net_w)/image_w) < (float(net_h)/image_h):
        new_w = net_w
        new_h = (image_h * net_w) / image_w
    else:
        new_h = net_h
        new_w = (image_w * net_h) / image_h
    
    x_offset = (net_w - new_w) / 2. / net_w
    x_scale  = float(net_w) / new_w
    y_offset = (net_h - new_h) / 2. / net_h
    y_scale  = float(net_h) / new_h
    
    for i in range(grid_h * grid_w):
        row = int(i / grid_w)
        col = i % grid_w
        
        for b in range(nb_box):
            objectness = netout[row][col][b][4]
            # 置信度过滤
            if objectness <= conf_thres:
                continue
            
            # 解码坐标
            x, y, w, h = netout[row][col][b][:4]
            x = (col + x) / grid_w
            y = (row + y) / grid_h
            w = anchors[b][0] * np.exp(w) / net_w
            h = anchors[b][1] * np.exp(h) / net_h
            
            # 灰边补偿
            x = (x - x_offset) * x_scale
            y = (y - y_offset) * y_scale
            w *= x_scale
            h *= y_scale
            
            # 转换为原图坐标
            x1 = (x - w/2) * image_w
            y1 = (y - h/2) * image_h
            x2 = (x + w/2) * image_w
            y2 = (y + h/2) * image_h
            
            # 获取类别置信度
            class_score = np.max(netout[row][col][b][5:])
            class_idx = np.argmax(netout[row][col][b][5:])
            total_score = objectness * class_score
            
            # 边界合法性校验
            if 0 < x1 < x2 < image_w and 0 < y1 < y2 < image_h:
                boxes.append([x1, y1, x2, y2, total_score, class_idx])
    
    return boxes
    
def letterbox_image(image, input_w, input_h):
    image_w, image_h = image.size
    scale = min(input_w/image_w, input_h/image_h)
    nw = int(image_w * scale)
    nh = int(image_h * scale)
    dx = (input_w - nw) // 2
    dy = (input_h - nh) // 2
    
    # 灰边填充(128,128,128)
    image = image.resize((nw, nh), Image.BICUBIC)
    new_image = Image.new('RGB', (input_w, input_h), (128,128,128))
    new_image.paste(image, (dx, dy))
    return new_image
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser() 
    parser.add_argument('-model', help='trained tflite', default=r'./yolo3_iou_smartcar_final.tflite',type=str)
    parser.add_argument('-image', help='test image', default=r'./1.jpg',type=str)
    args, unknown = parser.parse_known_args()

    cfg = yolo_cfg()
    class_names = cfg.class_names  
    anchors_path = cfg.cluster_anchor
    anchors = get_anchors(anchors_path)
    anchors_num = len(anchors) / 3
    
    anchors = anchors.reshape(-1, 2)
    
    num_classes = cfg.num_classes

    origin_img = Image.open(args.image).convert('RGB')
    image_shape = origin_img.size  # (w, h)
    input_img = letterbox_image(origin_img, 112, 112)
    
    # 数据预处理（严格照搬evaluate.py的int8转换）
    img_data = np.array(input_img) / 255.0
    img_data = img_data[np.newaxis, ...]
    interpreter = tf.lite.Interpreter(model_path=str(args.model))
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()[0]
    output_details = interpreter.get_output_details()[0]

    input_dtype = input_details['dtype']
    output_quant = output_details['quantization_parameters']
    output_scale = output_quant['scales']
    output_zp = output_quant['zero_points']
    
    # int8输入转换（关键！官方源码公式）
    if input_dtype == np.int8:
        test_data = (img_data * 255 - 128).astype(np.int8)
    else:
        test_data = img_data.astype(input_dtype)

    # 推理
    interpreter.set_tensor(input_details['index'], test_data)
    interpreter.invoke()

    # 输出反量化（int8 → float32）
    output_int8 = interpreter.get_tensor(output_details['index'])[0]
    output_float = (output_int8 - output_zp) * output_scale


    pred_boxes = decode_output(output_float,(112,112),image_shape, anchors,0.12)

    # NMS非极大值抑制
    valid_boxes = []
    valid_scores = []
    valid_classes = []
    if len(pred_boxes) > 0:
        pred_boxes = np.array(pred_boxes)
        boxes = pred_boxes[:, :4]
        scores = pred_boxes[:, 4]
        classes = pred_boxes[:, 5].astype(np.int32)
        
        # TF NMS
        indices = tf.image.non_max_suppression(
            boxes, scores, max_output_size=10,
            iou_threshold=0.45, score_threshold=0.12
        ).numpy()
        
        valid_boxes = boxes[indices]
        valid_scores = scores[indices]
        valid_classes = classes[indices]
        
    draw = ImageDraw.Draw(origin_img)
    im = origin_img
    try:
        # 尝试加载系统字体（Windows/Linux/Mac通用）
        font = ImageFont.truetype("simhei.ttf", 15)
    except:
        # 加载失败则使用默认字体
        font = ImageFont.load_default()

    # 同时遍历 框、置信度、类别索引
    for box, score, cls in zip(valid_boxes, valid_scores, valid_classes):
        x1, y1, x2, y2 = map(int, box)
        cls = int(cls)
        
        # 画框
        color = tuple(np.random.randint(0, 255, size=3))
        draw.rectangle([x1, y1, x2, y2], outline=color, width=2)
        
        # 画标签
        label = f"{class_names[cls]} {score:.2f}"
        draw.text((x1, y1-20), label, fill=color, font=font)
        
        print(f"{class_names[cls]} | 置信度: {score:.2f} | 坐标: {x1},{y1},{x2},{y2}")
    im.save('tflite_detected_img.jpg')
    im.show()
    print('output tflite_detected_img.jpg ')