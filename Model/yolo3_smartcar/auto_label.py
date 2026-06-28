"""
自动标注脚本：使用训练好的 TFLite 模型对图片批量推理，生成 VOC XML 标注文件。

用法:
    py -3.10 auto_label.py -image <图片目录> -model <tflite模型路径>

示例:
    # 标注整个文件夹的图片
    py -3.10 auto_label.py -image ../data/JPEGImages -model yolo3_iou_smartcar_final.tflite

    # 只标注指定几张图片
    py -3.10 auto_label.py -image ../data/JPEGImages -model yolo3_iou_smartcar_final.tflite -list 1000.jpg,1002.jpg

输出:
    在图片同目录下生成 .xml 标注文件
"""

import tensorflow as tf
import numpy as np
import os
import sys
import argparse
from PIL import Image
from utils import yolo_cfg
import xml.etree.ElementTree as ET
from xml.dom import minidom


def _sigmoid(x):
    return 1. / (1. + np.exp(-x))


def get_anchors(anchors_path):
    """加载锚框"""
    with open(anchors_path) as f:
        anchors = f.readline()
    anchors = [float(x) for x in anchors.split(',')]
    return np.array(anchors).reshape(-1, 6)


def letterbox_image(image, input_w, input_h):
    """Letterbox 缩放 + 灰边填充 (128,128,128)"""
    image_w, image_h = image.size
    scale = min(input_w / image_w, input_h / image_h)
    nw = int(image_w * scale)
    nh = int(image_h * scale)
    dx = (input_w - nw) // 2
    dy = (input_h - nh) // 2

    image = image.resize((nw, nh), Image.BICUBIC)
    new_image = Image.new('RGB', (input_w, input_h), (128, 128, 128))
    new_image.paste(image, (dx, dy))
    return new_image


def decode_output(netout, input_shape, image_shape, anchors, conf_thres):
    """解码 YOLO 输出为边界框"""
    grid_h, grid_w = netout.shape[:2]
    nb_box = len(anchors)
    netout = netout.reshape((grid_h, grid_w, nb_box, -1))
    net_w, net_h = input_shape
    image_w, image_h = image_shape
    boxes = []

    # Sigmoid 归一化
    netout[..., :2] = _sigmoid(netout[..., :2])
    netout[..., 4:] = _sigmoid(netout[..., 4:])

    # Letterbox 灰边补偿
    if (float(net_w) / image_w) < (float(net_h) / image_h):
        new_w = net_w
        new_h = (image_h * net_w) / image_w
    else:
        new_h = net_h
        new_w = (image_w * net_h) / image_h

    x_offset = (net_w - new_w) / 2. / net_w
    x_scale = float(net_w) / new_w
    y_offset = (net_h - new_h) / 2. / net_h
    y_scale = float(net_h) / new_h

    for i in range(grid_h * grid_w):
        row = int(i / grid_w)
        col = i % grid_w

        for b in range(nb_box):
            objectness = netout[row][col][b][4]
            if objectness <= conf_thres:
                continue

            x, y, w, h = netout[row][col][b][:4]
            x = (col + x) / grid_w
            y = (row + y) / grid_h
            w = anchors[b][0] * np.exp(w) / net_w
            h = anchors[b][1] * np.exp(h) / net_h

            x = (x - x_offset) * x_scale
            y = (y - y_offset) * y_scale
            w *= x_scale
            h *= y_scale

            x1 = (x - w / 2) * image_w
            y1 = (y - h / 2) * image_h
            x2 = (x + w / 2) * image_w
            y2 = (y + h / 2) * image_h

            class_score = np.max(netout[row][col][b][5:])
            class_idx = np.argmax(netout[row][col][b][5:])
            total_score = objectness * class_score

            if 0 < x1 < x2 < image_w and 0 < y1 < y2 < image_h:
                boxes.append([x1, y1, x2, y2, total_score, class_idx])

    return boxes


def generate_xml(image_path, image_shape, detections, class_names, output_dir):
    """根据检测结果生成 VOC 格式 XML 标注文件"""
    image_w, image_h = image_shape
    filename = os.path.basename(image_path)
    folder = os.path.basename(os.path.dirname(image_path))

    # 创建 XML 结构
    annotation = ET.Element('annotation')

    ET.SubElement(annotation, 'filename').text = filename
    ET.SubElement(annotation, 'object_num').text = str(len(detections))

    size = ET.SubElement(annotation, 'size')
    ET.SubElement(size, 'width').text = str(image_w)
    ET.SubElement(size, 'height').text = str(image_h)

    for det in detections:
        xmin, ymin, xmax, ymax, score, cls_id = det
        cls_name = class_names[int(cls_id)]

        obj = ET.SubElement(annotation, 'object')
        ET.SubElement(obj, 'name').text = cls_name
        ET.SubElement(obj, 'difficult').text = '0'
        bndbox = ET.SubElement(obj, 'bndbox')
        ET.SubElement(bndbox, 'xmin').text = str(int(round(xmin)))
        ET.SubElement(bndbox, 'ymin').text = str(int(round(ymin)))
        ET.SubElement(bndbox, 'xmax').text = str(int(round(xmax)))
        ET.SubElement(bndbox, 'ymax').text = str(int(round(ymax)))

    # 格式化输出
    xml_str = ET.tostring(annotation, encoding='utf-8')
    dom = minidom.parseString(xml_str)
    pretty_xml = dom.toprettyxml(indent='  ', encoding='utf-8')

    # 写入文件
    base_name = os.path.splitext(filename)[0]
    xml_path = os.path.join(output_dir, base_name + '.xml')
    with open(xml_path, 'wb') as f:
        f.write(pretty_xml)

    return xml_path


def auto_label(image_dir, model_path, class_names, anchors, conf_thres=0.12,
               iou_thres=0.45, input_size=(112, 112), output_dir=None,
               image_list=None):
    """
    批量自动标注图片

    参数:
        image_dir: 图片目录
        model_path: TFLite 模型路径
        class_names: 类别名称列表
        anchors: 锚框数组 shape=(N, 2)
        conf_thres: 置信度阈值
        iou_thres: NMS IOU 阈值
        input_size: 模型输入尺寸 (w, h)
        output_dir: XML 输出目录（默认与图片同目录）
        image_list: 指定图片名列表（None=全部）
    """
    # 加载 TFLite 模型
    print(f'加载模型: {model_path}')
    interpreter = tf.lite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()[0]
    output_details = interpreter.get_output_details()[0]
    input_dtype = input_details['dtype']

    # 获取输出量化参数
    output_quant = output_details['quantization_parameters']
    output_scale = output_quant['scales']
    output_zp = output_quant['zero_points']

    # 收集图片
    if image_list:
        img_files = image_list
    else:
        img_files = [f for f in os.listdir(image_dir)
                     if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp'))]

    if not img_files:
        print('错误: 未找到图片文件')
        return

    # 输出目录
    if output_dir is None:
        output_dir = image_dir
    os.makedirs(output_dir, exist_ok=True)

    total = len(img_files)
    detected_count = 0
    empty_count = 0

    print(f'开始自动标注: {total} 张图片')
    print(f'  置信度阈值: {conf_thres}')
    print(f'  NMS阈值: {iou_thres}')
    print(f'  类别: {class_names}')
    print('-' * 50)

    for idx, img_file in enumerate(img_files):
        img_path = os.path.join(image_dir, img_file)

        try:
            origin_img = Image.open(img_path).convert('RGB')
        except Exception as e:
            print(f'[{idx+1}/{total}] X {img_file} - 无法打开: {e}')
            continue

        image_shape = origin_img.size  # (w, h)

        # 预处理
        input_img = letterbox_image(origin_img, input_size[0], input_size[1])
        img_data = np.array(input_img) / 255.0
        img_data = img_data[np.newaxis, ...]

        # int8 输入转换
        if input_dtype == np.int8:
            test_data = (img_data * 255 - 128).astype(np.int8)
        else:
            test_data = img_data.astype(input_dtype)

        # 推理
        interpreter.set_tensor(input_details['index'], test_data)
        interpreter.invoke()

        # 输出反量化
        output_int8 = interpreter.get_tensor(output_details['index'])[0]
        output_float = (output_int8 - output_zp) * output_scale

        # 解码
        pred_boxes = decode_output(output_float, input_size, image_shape,
                                   anchors, conf_thres)

        # NMS
        detections = []
        if len(pred_boxes) > 0:
            pred_boxes = np.array(pred_boxes)
            boxes = pred_boxes[:, :4]
            scores = pred_boxes[:, 4]
            classes = pred_boxes[:, 5].astype(np.int32)

            indices = tf.image.non_max_suppression(
                boxes, scores, max_output_size=20,
                iou_threshold=iou_thres, score_threshold=conf_thres
            ).numpy()

            for i in indices:
                detections.append([
                    boxes[i][0], boxes[i][1], boxes[i][2], boxes[i][3],
                    scores[i], classes[i]
                ])

        # 生成 XML
        xml_path = generate_xml(img_path, image_shape, detections,
                                class_names, output_dir)

        if detections:
            detected_count += 1
            labels = ', '.join([f'{class_names[int(d[5])]}' for d in detections])
            print(f'[{idx+1}/{total}] OK {img_file} -> {len(detections)}个目标 [{labels}]')
        else:
            empty_count += 1
            print(f'[{idx+1}/{total}] - {img_file} -> 无目标')

    print('-' * 50)
    print(f'Done! Total: {total}, detected: {detected_count}, empty: {empty_count}')
    print(f'标注文件输出目录: {output_dir}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='自动标注工具')
    parser.add_argument('-image', required=True,
                        help='图片目录路径')
    parser.add_argument('-model', default='yolo3_iou_smartcar_final.tflite',
                        help='TFLite 模型路径')
    parser.add_argument('-conf', type=float, default=0.12,
                        help='置信度阈值 (默认0.12)')
    parser.add_argument('-iou', type=float, default=0.45,
                        help='NMS IOU阈值 (默认0.45)')
    parser.add_argument('-output', default=None,
                        help='XML输出目录 (默认与图片同目录)')
    parser.add_argument('-list', default=None,
                        help='指定图片名，逗号分隔 (默认处理全部)')
    args, unknown = parser.parse_known_args()

    # 加载配置
    cfg = yolo_cfg()
    class_names = cfg.class_names
    anchors_path = cfg.cluster_anchor

    # 加载锚框并 reshape 为 (N, 2)
    anchors = get_anchors(anchors_path)
    anchors = anchors.reshape(-1, 2)

    # 图片列表
    image_list = None
    if args.list:
        image_list = [f.strip() for f in args.list.split(',')]

    auto_label(
        image_dir=args.image,
        model_path=args.model,
        class_names=class_names,
        anchors=anchors,
        conf_thres=args.conf,
        iou_thres=args.iou,
        input_size=(cfg.width, cfg.height),
        output_dir=args.output,
        image_list=image_list
    )
