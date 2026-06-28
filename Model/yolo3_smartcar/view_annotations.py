"""
标注数据查看器生成器
=====================
读取 VOC XML 标注和对应图片，生成独立 HTML 页面用于浏览标注数据、
查看统计信息、筛选标注、检查异常。

用法:
    py -3.10 view_annotations.py -image <图片目录> [-ann <标注目录>] [-output <输出目录>]

参数:
    -image   图片目录路径 (默认: ../data/JPEGImages)
    -ann     标注目录路径 (默认: ../data/Annotations)
    -output  输出目录路径 (默认: ../view_output)

示例:
    py -3.10 view_annotations.py -image C:\imgs
    py -3.10 view_annotations.py -image D:\photos -ann D:\labels -output D:\viewer
"""

import os
import sys
import json
import argparse
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from PIL import Image, ImageDraw, ImageFont

# ============================================================
#  配置（可由命令行参数覆盖）
# ============================================================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ANN_DIR_DEFAULT = os.path.normpath(os.path.join(BASE_DIR, '..', 'data', 'Annotations'))
JPG_DIR_DEFAULT = os.path.normpath(os.path.join(BASE_DIR, '..', 'data', 'JPEGImages'))
OUT_DIR_DEFAULT = os.path.normpath(os.path.join(BASE_DIR, '..', 'view_output'))

CLASS_COLORS_RGB = {
    'sandbag': (230, 25, 75),    # #E6194B  vivid red
    'netball': (60, 180, 75),    # #3CB44B  vivid green
    'bear': (67, 99, 216),       # #4363D8  vivid blue
}
CLASS_COLORS_HEX = {
    'sandbag': '#E6194B',
    'netball': '#3CB44B',
    'bear': '#4363D8',
}
# 其他未在列表中的类别自动分配颜色
_EXTRA_COLORS = [
    (255, 127, 14), (152, 223, 138), (197, 176, 213),
    (255, 255, 25), (0, 255, 255), (255, 0, 255),
]
_EXTRA_COLORS_HEX = [
    '#FF7F0E', '#98DF8A', '#C5B0D5',
    '#FFFF19', '#00FFFF', '#FF00FF',
]


def get_class_colors(class_names):
    """为类别列表分配颜色，已知类别用预定义色，未知类别自动分配"""
    rgb = {}
    hex_ = {}
    for i, cls in enumerate(class_names):
        if cls in CLASS_COLORS_RGB:
            rgb[cls] = CLASS_COLORS_RGB[cls]
            hex_[cls] = CLASS_COLORS_HEX[cls]
        else:
            idx = i % len(_EXTRA_COLORS)
            rgb[cls] = _EXTRA_COLORS[idx]
            hex_[cls] = _EXTRA_COLORS_HEX[idx]
    return rgb, hex_


def load_class_names():
    """从 config.cfg 读取类别名，失败则用默认值"""
    try:
        from utils import yolo_cfg
        cfg = yolo_cfg()
        names = cfg.class_names
        print(f'  从 config.cfg 加载类别: {names}')
        return names
    except Exception as e:
        print(f'  警告: 无法加载 config.cfg ({e})，使用默认类别')
        return ['sandbag', 'netball', 'bear']


# ============================================================
#  XML 解析
# ============================================================
def parse_xml(xml_path):
    """解析单个 VOC XML，返回 dict 或 None（解析失败时）"""
    try:
        tree = ET.parse(xml_path)
        root = tree.getroot()
    except ET.ParseError as e:
        return {'error': f'XML parse error: {e}'}

    filename = root.findtext('filename', '')
    obj_num_tag = root.findtext('object_num', '')
    size_el = root.find('size')
    width = int(size_el.findtext('width', '0')) if size_el is not None else 0
    height = int(size_el.findtext('height', '0')) if size_el is not None else 0

    objects = []
    for obj_el in root.findall('object'):
        name = obj_el.findtext('name', 'unknown')
        bndbox = obj_el.find('bndbox')
        if bndbox is None:
            continue
        box = [
            int(float(bndbox.findtext('xmin', '0'))),
            int(float(bndbox.findtext('ymin', '0'))),
            int(float(bndbox.findtext('xmax', '0'))),
            int(float(bndbox.findtext('ymax', '0'))),
        ]
        difficult = int(obj_el.findtext('difficult', '0'))
        objects.append({'cls': name, 'box': box, 'difficult': difficult})

    return {
        'file': filename,
        'width': width,
        'height': height,
        'objects': objects,
        'obj_num_tag': obj_num_tag,
    }


def parse_all_xmls(ann_dir):
    """遍历所有 XML，返回 (image_data_list, parse_errors)"""
    images = []
    parse_errors = []
    xml_files = [f for f in os.listdir(ann_dir) if f.lower().endswith('.xml')]
    xml_files.sort()

    total = len(xml_files)
    print(f'[1/4] 解析 {total} 个 XML 标注文件...')

    for i, fname in enumerate(xml_files):
        xml_path = os.path.join(ann_dir, fname)
        data = parse_xml(xml_path)
        if 'error' in data:
            parse_errors.append({'file': fname, 'error': data['error']})
        else:
            data['xml_file'] = fname
            data['base_name'] = os.path.splitext(fname)[0]
            images.append(data)

    print(f'  解析完成: {len(images)} 个有效标注, {len(parse_errors)} 个解析错误')
    return images, parse_errors


# ============================================================
#  交叉对照 & 异常检测
# ============================================================
def check_anomalies(images, ann_dir, jpg_dir, class_names):
    """检测异常，返回 anomaly 列表"""
    print('[2/4] 异常检测...')
    anomalies = []
    ann_basenames = set()
    jpg_map = {}
    cls_set = set(class_names)

    # 构建 JPG 索引
    if os.path.isdir(jpg_dir):
        for f in os.listdir(jpg_dir):
            if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp')):
                base = os.path.splitext(f)[0]
                jpg_map[base] = f

    # 检查每张标注
    for img in images:
        fn = img['file']
        bn = os.path.splitext(fn)[0] if fn else img['base_name']
        ann_basenames.add(bn)
        w, h = img['width'], img['height']

        if not w or not h:
            anomalies.append({
                'file': fn, 'severity': 'error',
                'type': '缺少尺寸信息',
                'detail': 'XML 中 <size> 缺失或 width/height 为 0',
            })
            continue

        # object_num 标签与实际数量不匹配
        tag_num = img.get('obj_num_tag', '')
        actual_num = len(img['objects'])
        if tag_num != '' and int(tag_num) != actual_num:
            anomalies.append({
                'file': fn, 'severity': 'warning',
                'type': 'object_num 不匹配',
                'detail': f'标签值={tag_num}, 实际 object 数={actual_num}',
            })

        # 空标注
        if actual_num == 0:
            anomalies.append({
                'file': fn, 'severity': 'warning',
                'type': '空标注', 'detail': '该图没有任何标注目标',
            })

        # 检查每个框
        for obj in img['objects']:
            x1, y1, x2, y2 = obj['box']
            cls = obj['cls']

            if cls not in cls_set:
                anomalies.append({
                    'file': fn, 'severity': 'error',
                    'type': '未知类别', 'detail': f'类别 "{cls}" 不在预定义列表中 [{", ".join(class_names)}]',
                })

            if x1 < 0 or y1 < 0 or x2 > w or y2 > h:
                anomalies.append({
                    'file': fn, 'severity': 'error',
                    'type': '边界框越界',
                    'detail': f'{cls}: [{x1},{y1},{x2},{y2}] 超出图像 {w}x{h}',
                })

            if x1 >= x2 or y1 >= y2:
                anomalies.append({
                    'file': fn, 'severity': 'error',
                    'type': '无效边界框',
                    'detail': f'{cls}: [{x1},{y1},{x2},{y2}] xmin>=xmax 或 ymin>=ymax',
                })

    # 检查 XML 无对应图片
    missing_imgs = ann_basenames - set(jpg_map.keys())
    if missing_imgs:
        for bn in sorted(missing_imgs):
            xml_file = bn + '.xml'
            anomalies.append({
                'file': xml_file, 'severity': 'error',
                'type': 'XML 无对应图片',
                'detail': f'{xml_file} 在图片目录中找不到 {bn}.jpg/png/bmp',
            })

    # 检查图片无对应 XML
    missing_xmls = set(jpg_map.keys()) - ann_basenames
    if missing_xmls:
        for bn in sorted(missing_xmls):
            jpg_file = jpg_map[bn]
            anomalies.append({
                'file': jpg_file, 'severity': 'warning',
                'type': '图片无对应 XML',
                'detail': f'{jpg_file} 在标注目录中找不到 {bn}.xml',
            })

    sev_counts = Counter(a['severity'] for a in anomalies)
    print(f'  发现 {len(anomalies)} 个异常: '
          f'error={sev_counts.get("error", 0)}, '
          f'warning={sev_counts.get("warning", 0)}')
    return anomalies


# ============================================================
#  统计聚合
# ============================================================
def compute_stats(images, class_names):
    """聚合数据集统计"""
    print('[3/4] 统计聚合...')
    total_boxes = 0
    class_counts = Counter()
    box_areas = defaultdict(list)
    obj_per_image = Counter()

    for img in images:
        n = len(img['objects'])
        if n >= 5:
            obj_per_image['5+'] += 1
        else:
            obj_per_image[str(n)] += 1

        for obj in img['objects']:
            cls = obj['cls']
            x1, y1, x2, y2 = obj['box']
            area = (x2 - x1) * (y2 - y1)
            class_counts[cls] += 1
            box_areas[cls].append(area)
            total_boxes += 1

    # 框面积分布 (按类别分桶)
    area_bins = [(0, 500), (500, 1000), (1000, 2000),
                 (2000, 5000), (5000, 10000), (10000, 999999)]
    area_histogram = {}
    for cls in class_names:
        areas = box_areas.get(cls, [])
        bins_for_cls = [0] * len(area_bins)
        for a in areas:
            for bi, (lo, hi) in enumerate(area_bins):
                if lo <= a < hi:
                    bins_for_cls[bi] += 1
                    break
            else:
                bins_for_cls[-1] += 1
        area_histogram[cls] = bins_for_cls

    # 面积统计值
    area_stats = {}
    for cls in class_names:
        areas = box_areas.get(cls, [])
        if areas:
            s = sorted(areas)
            n = len(s)
            area_stats[cls] = {
                'min': s[0],
                'p25': s[int(n * 0.25)],
                'median': s[int(n * 0.5)],
                'p75': s[int(n * 0.75)],
                'max': s[-1],
                'mean': round(sum(s) / n, 1),
                'total': n,
            }
        else:
            area_stats[cls] = {'min': 0, 'p25': 0, 'median': 0,
                               'p75': 0, 'max': 0, 'mean': 0, 'total': 0}

    print(f'  图片: {len(images)}  框总数: {total_boxes}  类别分布: '
          f'{", ".join(f"{c} {class_counts[c]}" for c in class_names)}')

    return {
        'total_images': len(images),
        'total_boxes': total_boxes,
        'class_counts': {c: class_counts[c] for c in class_names},
        'area_histogram': area_histogram,
        'area_bins': [f'{lo}-{hi}' if hi < 999999 else f'{lo}+'
                      for lo, hi in area_bins],
        'area_stats': area_stats,
        'obj_per_image': dict(obj_per_image),
    }


# ============================================================
#  缩略图生成
# ============================================================
def draw_thumbnail(img_path, objects, out_path, font, colors_rgb):
    """在图片上画框+标签，保存缩略图"""
    try:
        img = Image.open(img_path).convert('RGB')
    except Exception:
        return False

    draw = ImageDraw.Draw(img)

    for obj in objects:
        cls = obj['cls']
        x1, y1, x2, y2 = obj['box']
        color = colors_rgb.get(cls, (255, 255, 0))

        # 画框 (2px)
        draw.rectangle([x1, y1, x2, y2], outline=color, width=2)

        # 标签背景 + 文字 (放在框左上角)
        label = cls
        try:
            bbox = draw.textbbox((x1, y1 - 16), label, font=font)
            tb_x1, tb_y1, tb_x2, tb_y2 = bbox
            draw.rectangle([tb_x1 - 2, tb_y1 - 1, tb_x2 + 2, tb_y2 + 1],
                           fill=color)
            draw.text((x1, y1 - 16), label, fill=(255, 255, 255), font=font)
        except Exception:
            pass

    img.save(out_path, 'JPEG', quality=75)
    return True


def generate_thumbnails(images, jpg_dir, thumb_dir, font, colors_rgb):
    """批量生成标注缩略图"""
    print('[4/4] 生成缩略图...')
    os.makedirs(thumb_dir, exist_ok=True)
    total = len(images)

    for idx, img in enumerate(images):
        fn = img['file']
        img_path = os.path.join(jpg_dir, fn)
        ext = os.path.splitext(fn)[1]
        thumb_name = f'{idx:05d}_{fn}'
        if not thumb_name.lower().endswith(('.jpg', '.jpeg')):
            thumb_name = f'{idx:05d}_{fn}{ext or ".jpg"}'
        thumb_path = os.path.join(thumb_dir, thumb_name)

        if os.path.exists(thumb_path):
            img['thumb'] = 'thumbs/' + thumb_name
            continue

        if not os.path.exists(img_path):
            img['thumb'] = None
            continue

        if draw_thumbnail(img_path, img['objects'], thumb_path, font, colors_rgb):
            img['thumb'] = 'thumbs/' + thumb_name
        else:
            img['thumb'] = None

        if (idx + 1) % 200 == 0 or (idx + 1) == total:
            print(f'  缩略图: {idx+1}/{total}')

    success = sum(1 for i in images if i.get('thumb'))
    print(f'  完成: {success}/{total} 张缩略图生成')


# ============================================================
#  HTML 生成
# ============================================================
def prepare_image_data(images):
    """将 image 数据精简，准备嵌入 JSON"""
    out = []
    for idx, img in enumerate(images):
        objs = []
        classes_present = set()
        for obj in img['objects']:
            objs.append({'c': obj['cls'], 'b': obj['box']})
            classes_present.add(obj['cls'])
        out.append({
            'f': img['file'],
            't': img.get('thumb', ''),
            'w': img['width'],
            'h': img['height'],
            'o': objs,
            'n': len(objs),
            'cp': sorted(classes_present),
        })
    return out


def build_html_dynamic_parts(class_names, colors_hex):
    """生成 HTML 中动态的 CSS 和 HTML 片段（类别相关）"""
    # CSS: class-badge 颜色
    css_rules = []
    for cls in class_names:
        css_rules.append(f'.class-badge.{cls} {{ background: {colors_hex[cls]}; }}')
    css_badges = '\n'.join(css_rules)

    # Header 中的类别徽章
    header_badges = '\n'.join(
        f'      <span class="class-badge {cls}">{cls}</span>'
        for cls in class_names
    )

    # Gallery 筛选栏的 checkbox
    filter_items = '\n'.join([
        f'    <label><input type="checkbox" class="class-filter" value="{cls}" checked>'
        f'<span class="class-badge {cls}">{cls}</span>'
        f'<span class="fc-count" data-cls="{cls}"></span></label>'
        for cls in class_names
    ])

    return css_badges, header_badges, filter_items


def generate_html(images, stats, anomalies, out_dir, jpg_dir_rel,
                  class_names, colors_hex):
    """生成 index.html"""
    print('  生成 HTML...')
    os.makedirs(out_dir, exist_ok=True)

    img_data = prepare_image_data(images)

    payload = {
        'images': img_data,
        'stats': stats,
        'anomalies': anomalies,
        'class_names': class_names,
        'class_colors': colors_hex,
        'jpg_base': jpg_dir_rel.replace('\\', '/'),
    }
    json_str = json.dumps(payload, ensure_ascii=False)

    css_badges, header_badges, filter_items = build_html_dynamic_parts(
        class_names, colors_hex)

    # 替换模板中的动态占位符
    html = HTML_TEMPLATE
    html = html.replace('__CLASS_BADGES_CSS__', css_badges)
    html = html.replace('__CLASS_BADGES_HEADER__', header_badges)
    html = html.replace('__CLASS_FILTERS__', filter_items)
    html = html.replace('__JSON_DATA__', json_str)

    html_path = os.path.join(out_dir, 'index.html')
    with open(html_path, 'w', encoding='utf-8') as f:
        f.write(html)
    print(f'  生成完成: {html_path}')


# ============================================================
#  HTML 模板
# ============================================================
HTML_TEMPLATE = r'''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>SmartCar 标注数据查看器</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js">
</script>
<style>
* { margin: 0; padding: 0; box-sizing: border-box; }
body { font-family: 'Segoe UI', 'Microsoft YaHei', sans-serif; background: #f0f2f5; color: #333; }

/* Header */
.header {
  background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
  color: #fff; padding: 20px 32px; display: flex;
  justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 12px;
}
.header h1 { font-size: 22px; font-weight: 600; }
.summary-strip { display: flex; gap: 20px; font-size: 14px; opacity: 0.85; }
.summary-strip span { white-space: nowrap; }
.class-badge { display: inline-block; padding: 2px 10px; border-radius: 12px;
  font-size: 12px; font-weight: 600; color: #fff; margin: 0 2px; }
__CLASS_BADGES_CSS__

/* Tabs */
.tabs { display: flex; background: #fff; border-bottom: 2px solid #e0e0e0;
  padding: 0 32px; gap: 0; }
.tab-btn { padding: 12px 24px; border: none; background: none; cursor: pointer;
  font-size: 14px; color: #666; border-bottom: 3px solid transparent;
  margin-bottom: -2px; transition: all 0.2s; position: relative; }
.tab-btn:hover { color: #333; }
.tab-btn.active { color: #1a73e8; border-bottom-color: #1a73e8; font-weight: 600; }
.badge { display: inline-block; background: #e74c3c; color: #fff; font-size: 11px;
  padding: 1px 7px; border-radius: 10px; margin-left: 4px; vertical-align: top; }
.badge.clean { background: #27ae60; }

.tab-content { display: none; padding: 20px 32px; }
.tab-content.active { display: block; }

/* Filter bar */
.filter-bar { display: flex; flex-wrap: wrap; gap: 12px; align-items: center;
  margin-bottom: 16px; padding: 12px 16px; background: #fff; border-radius: 8px;
  box-shadow: 0 1px 3px rgba(0,0,0,0.08); }
.filter-bar label { display: flex; align-items: center; gap: 6px; font-size: 13px;
  cursor: pointer; user-select: none; }
.filter-bar input[type="checkbox"] { cursor: pointer; width: 16px; height: 16px; }
#search-box { padding: 6px 12px; border: 1px solid #ddd; border-radius: 6px;
  font-size: 13px; width: 180px; outline: none; }
#search-box:focus { border-color: #1a73e8; }
#result-count { font-size: 13px; color: #888; margin-left: auto; }

/* Gallery grid */
.gallery-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(220px, 1fr));
  gap: 12px; }
.gallery-card { background: #fff; border-radius: 8px; overflow: hidden;
  box-shadow: 0 1px 3px rgba(0,0,0,0.08); cursor: pointer; transition: transform 0.15s, box-shadow 0.15s; }
.gallery-card:hover { transform: translateY(-2px); box-shadow: 0 4px 12px rgba(0,0,0,0.15); }
.gallery-card img { width: 100%; height: auto; display: block; aspect-ratio: 4/3; object-fit: cover; }
.gallery-card .card-info { padding: 6px 10px; display: flex; justify-content: space-between;
  align-items: center; font-size: 12px; }
.gallery-card .card-fname { color: #555; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.gallery-card .card-count { color: #888; white-space: nowrap; }
.gallery-card .no-thumb { width: 100%; aspect-ratio: 4/3; background: #eee;
  display: flex; align-items: center; justify-content: center; color: #999; font-size: 13px; }

/* Pagination */
.pagination { display: flex; justify-content: center; align-items: center; gap: 8px;
  margin: 16px 0; }
.pagination button { padding: 6px 14px; border: 1px solid #ddd; background: #fff;
  border-radius: 6px; cursor: pointer; font-size: 13px; }
.pagination button:hover { background: #f5f5f5; }
.pagination button:disabled { opacity: 0.4; cursor: default; }
.pagination .page-info { font-size: 13px; color: #666; }
.pagination input { width: 50px; padding: 4px 8px; border: 1px solid #ddd;
  border-radius: 6px; text-align: center; font-size: 13px; }

/* Lightbox */
.lightbox { position: fixed; top: 0; left: 0; width: 100%; height: 100%;
  background: rgba(0,0,0,0.88); z-index: 999; display: flex; align-items: center;
  justify-content: center; gap: 24px; padding: 24px; }
.lightbox.hidden { display: none; }
.lightbox .lb-close { position: absolute; top: 16px; right: 24px; font-size: 36px;
  color: #fff; cursor: pointer; line-height: 1; }
.lightbox .lb-nav { position: absolute; top: 50%; transform: translateY(-50%);
  font-size: 48px; color: #fff; cursor: pointer; user-select: none;
  padding: 12px; opacity: 0.7; }
.lightbox .lb-nav:hover { opacity: 1; }
.lightbox .lb-prev { left: 16px; }
.lightbox .lb-next { right: 16px; }
.lightbox .lb-img { max-width: 70%; max-height: 90vh; border-radius: 4px;
  object-fit: contain; }
.lightbox .lb-info { background: #fff; border-radius: 8px; padding: 16px;
  max-width: 320px; max-height: 90vh; overflow-y: auto; font-size: 13px; }
.lightbox .lb-info h3 { margin-bottom: 8px; font-size: 15px; color: #1a1a2e; }
.lightbox .lb-obj { padding: 8px; margin-bottom: 6px; border-radius: 6px;
  border-left: 4px solid; background: #f9f9f9; }
.lightbox .lb-obj .obj-cls { font-weight: 600; }
.lightbox .lb-obj .obj-coords { color: #777; font-size: 12px; }
.lightbox .lb-obj .obj-area { color: #999; font-size: 11px; }
.lb-empty { color: #ccc; text-align: center; padding: 20px; }

/* Stats */
.charts-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(380px, 1fr));
  gap: 20px; }
.chart-card { background: #fff; border-radius: 8px; padding: 20px;
  box-shadow: 0 1px 3px rgba(0,0,0,0.08); }
.chart-card canvas { max-height: 320px; }
.stats-table { width: 100%; border-collapse: collapse; font-size: 13px; margin-top: 12px; }
.stats-table th, .stats-table td { padding: 6px 10px; border-bottom: 1px solid #eee;
  text-align: right; }
.stats-table th { text-align: left; color: #888; font-weight: 500; }
.stats-table td:first-child { text-align: left; }

/* Anomalies */
.anomaly-clean { background: #d4edda; color: #155724; padding: 16px 20px;
  border-radius: 8px; font-size: 14px; }
.anomaly-table { width: 100%; border-collapse: collapse; font-size: 13px; }
.anomaly-table th { text-align: left; padding: 8px 12px; background: #f5f5f5;
  border-bottom: 2px solid #ddd; }
.anomaly-table td { padding: 8px 12px; border-bottom: 1px solid #eee; }
.anomaly-table tr.sev-error { border-left: 3px solid #e74c3c; }
.anomaly-table tr.sev-warning { border-left: 3px solid #f39c12; }
.sev-tag { display: inline-block; padding: 1px 8px; border-radius: 10px;
  font-size: 11px; font-weight: 600; color: #fff; }
.sev-tag.error { background: #e74c3c; }
.sev-tag.warning { background: #f39c12; }

/* Responsive */
@media (max-width: 768px) {
  .header { padding: 14px 16px; }
  .header h1 { font-size: 18px; }
  .tabs, .tab-content { padding: 0 12px; }
  .tab-content { padding: 12px; }
  .gallery-grid { grid-template-columns: repeat(auto-fill, minmax(150px, 1fr)); }
  .charts-grid { grid-template-columns: 1fr; }
  .lightbox { flex-direction: column; }
  .lightbox .lb-img { max-width: 95%; max-height: 50vh; }
  .lightbox .lb-info { max-width: 95%; max-height: 35vh; }
}
</style>
</head>
<body>

<!-- Header -->
<div class="header">
  <h1>🔍 SmartCar 标注数据查看器</h1>
  <div class="summary-strip">
    <span>📷 <b id="h-images">-</b> images</span>
    <span>📦 <b id="h-boxes">-</b> boxes</span>
    <span>
__CLASS_BADGES_HEADER__
    </span>
  </div>
</div>

<!-- Tabs -->
<nav class="tabs" id="tab-nav">
  <button class="tab-btn active" data-tab="gallery">🖼 Gallery</button>
  <button class="tab-btn" data-tab="stats">📊 Statistics</button>
  <button class="tab-btn" data-tab="anomalies">
    ⚠ Anomalies <span class="badge" id="anomaly-badge" style="display:none">0</span>
  </button>
</nav>

<!-- Gallery Tab -->
<section id="tab-gallery" class="tab-content active">
  <div class="filter-bar">
__CLASS_FILTERS__
    <input type="text" id="search-box" placeholder="🔍 搜索文件名...">
    <span id="result-count"></span>
  </div>
  <div class="pagination" id="pagination-top"></div>
  <div class="gallery-grid" id="gallery"></div>
  <div class="pagination" id="pagination-bottom"></div>
</section>

<!-- Stats Tab -->
<section id="tab-stats" class="tab-content">
  <div class="charts-grid">
    <div class="chart-card">
      <canvas id="chart-class-dist"></canvas>
    </div>
    <div class="chart-card">
      <canvas id="chart-box-area"></canvas>
    </div>
    <div class="chart-card">
      <canvas id="chart-obj-per-img"></canvas>
    </div>
  </div>
  <div class="chart-card" style="margin-top: 20px;">
    <h3 style="margin-bottom: 12px; font-size: 15px;">框面积统计 (px²)</h3>
    <table class="stats-table" id="area-stats-table"></table>
  </div>
</section>

<!-- Anomalies Tab -->
<section id="tab-anomalies" class="tab-content">
  <div id="anomalies-content"></div>
</section>

<!-- Lightbox -->
<div id="lightbox" class="lightbox hidden">
  <span class="lb-close" id="lb-close">&times;</span>
  <span class="lb-nav lb-prev" id="lb-prev">◀</span>
  <img id="lb-img" class="lb-img" src="" alt="">
  <div class="lb-info" id="lb-info"></div>
  <span class="lb-nav lb-next" id="lb-next">▶</span>
</div>

<script id="anno-data" type="application/json">
__JSON_DATA__
</script>

<script>
(function() {
  var script = document.getElementById('anno-data');
  var DATA = JSON.parse(script.textContent);
  var ALL = DATA.images;
  var CLASS_NAMES = DATA.class_names;
  var CLASS_COLORS = DATA.class_colors;
  var JPG_BASE = DATA.jpg_base;

  var currentTab = 'gallery';
  var activeFilters = new Set(CLASS_NAMES);
  var searchText = '';
  var currentPage = 1;
  var PER_PAGE = 20;
  var filteredImages = [];
  var lbIndex = -1;

  function applyFilter() {
    filteredImages = ALL.filter(function(img) {
      if (activeFilters.size < CLASS_NAMES.length) {
        var has = false;
        for (var i = 0; i < img.cp.length; i++) {
          if (activeFilters.has(img.cp[i])) { has = true; break; }
        }
        if (!has) return false;
      }
      if (searchText) {
        if (img.f.toLowerCase().indexOf(searchText) === -1) return false;
      }
      return true;
    });
  }

  function updateFilterDisplay() {
    applyFilter();
    currentPage = Math.min(currentPage, Math.ceil(filteredImages.length / PER_PAGE) || 1);
    renderGallery();
    renderPagination();
    document.getElementById('result-count').textContent =
      '共 ' + filteredImages.length + ' 张图片';
  }

  function renderGallery() {
    var grid = document.getElementById('gallery');
    var start = (currentPage - 1) * PER_PAGE;
    var end = Math.min(start + PER_PAGE, filteredImages.length);
    var page = filteredImages.slice(start, end);
    var html = '';
    for (var i = 0; i < page.length; i++) {
      var img = page[i];
      var actualIdx = ALL.indexOf(img);
      html += '<div class="gallery-card" data-idx="' + actualIdx + '" data-fi="' +
        (start + i) + '" onclick="window._openLightbox(' + (start + i) + ')">';
      if (img.t) {
        html += '<img src="' + img.t + '" alt="' + img.f + '" loading="lazy">';
      } else {
        html += '<div class="no-thumb">无缩略图</div>';
      }
      html += '<div class="card-info">' +
        '<span class="card-fname" title="' + img.f + '">' + img.f + '</span>' +
        '<span class="card-count">' + img.n + ' obj</span>' +
        '</div></div>';
    }
    grid.innerHTML = html;
  }

  function renderPagination() {
    var totalPages = Math.ceil(filteredImages.length / PER_PAGE) || 1;
    var gen = function(currentPage) {
      var h = '';
      h += '<button onclick="window._goPage(1)" ' + (currentPage === 1 ? 'disabled' : '') + '>« 首页</button>';
      h += '<button onclick="window._goPage(' + (currentPage - 1) + ')" ' + (currentPage <= 1 ? 'disabled' : '') + '>‹ 上页</button>';
      h += '<span class="page-info">第 <b>' + currentPage + '</b> / ' + totalPages + ' 页</span>';
      h += '<button onclick="window._goPage(' + (currentPage + 1) + ')" ' + (currentPage >= totalPages ? 'disabled' : '') + '>下页 ›</button>';
      h += '<button onclick="window._goPage(' + totalPages + ')" ' + (currentPage === totalPages ? 'disabled' : '') + '>末页 »</button>';
      h += ' <input type="text" id="jump-page" value="' + currentPage + '" onkeydown="if(event.key===\'Enter\')window._goPage(parseInt(this.value))" title="输入页码回车跳转">';
      return h;
    };
    document.getElementById('pagination-top').innerHTML = gen(currentPage);
    document.getElementById('pagination-bottom').innerHTML = gen(currentPage);
  }

  window._goPage = function(p) {
    var total = Math.ceil(filteredImages.length / PER_PAGE) || 1;
    if (isNaN(p) || p < 1) p = 1;
    if (p > total) p = total;
    currentPage = p;
    renderGallery();
    renderPagination();
    document.getElementById('gallery').scrollIntoView({behavior: 'smooth'});
  };

  window._openLightbox = function(fi) {
    lbIndex = fi;
    document.getElementById('lightbox').classList.remove('hidden');
    document.body.style.overflow = 'hidden';
    renderLightbox();
  };

  function renderLightbox() {
    var img = filteredImages[lbIndex];
    if (!img) return;
    document.getElementById('lb-img').src = JPG_BASE + '/' + img.f;
    document.getElementById('lb-img').alt = img.f;
    var info = document.getElementById('lb-info');
    var h = '<h3>' + img.f + '</h3>';
    h += '<p style="color:#888;font-size:12px;margin-bottom:8px">' +
      img.w + '×' + img.h + ' | ' + img.n + ' objects</p>';
    if (img.o.length === 0) {
      h += '<div class="lb-empty">No annotations</div>';
    }
    for (var i = 0; i < img.o.length; i++) {
      var obj = img.o[i];
      var cls = obj.c;
      var box = obj.b;
      var w = box[2] - box[0];
      var h2 = box[3] - box[1];
      var area = w * h2;
      var color = CLASS_COLORS[cls] || '#999';
      h += '<div class="lb-obj" style="border-left-color:' + color + '">' +
        '<div class="obj-cls"><span class="class-badge ' + cls + '">' + cls + '</span></div>' +
        '<div class="obj-coords">📐 [' + box.join(', ') + '] &nbsp; ' + w + '×' + h2 + '</div>' +
        '<div class="obj-area">面积: ' + area + ' px²</div>' +
        '</div>';
    }
    info.innerHTML = h;
    document.getElementById('lb-prev').style.display = lbIndex > 0 ? '' : 'none';
    document.getElementById('lb-next').style.display =
      lbIndex < filteredImages.length - 1 ? '' : 'none';
  }

  function closeLightbox() {
    document.getElementById('lightbox').classList.add('hidden');
    document.body.style.overflow = '';
    lbIndex = -1;
  }

  function lightboxPrev() {
    if (lbIndex > 0) { lbIndex--; renderLightbox(); }
  }

  function lightboxNext() {
    if (lbIndex < filteredImages.length - 1) { lbIndex++; renderLightbox(); }
  }

  document.getElementById('lb-close').onclick = closeLightbox;
  document.getElementById('lb-prev').onclick = lightboxPrev;
  document.getElementById('lb-next').onclick = lightboxNext;
  document.getElementById('lightbox').addEventListener('click', function(e) {
    if (e.target === this) closeLightbox();
  });
  document.addEventListener('keydown', function(e) {
    if (document.getElementById('lightbox').classList.contains('hidden')) return;
    if (e.key === 'Escape') closeLightbox();
    if (e.key === 'ArrowLeft') lightboxPrev();
    if (e.key === 'ArrowRight') lightboxNext();
  });

  document.getElementById('tab-nav').addEventListener('click', function(e) {
    var btn = e.target.closest('.tab-btn');
    if (!btn) return;
    var tab = btn.dataset.tab;
    if (tab === currentTab) return;
    currentTab = tab;
    document.querySelectorAll('.tab-btn').forEach(function(b) { b.classList.remove('active'); });
    btn.classList.add('active');
    document.querySelectorAll('.tab-content').forEach(function(s) { s.classList.remove('active'); });
    document.getElementById('tab-' + tab).classList.add('active');
    if (tab === 'stats') renderStats();
  });

  document.querySelectorAll('.class-filter').forEach(function(cb) {
    cb.addEventListener('change', function() {
      if (this.checked) activeFilters.add(this.value);
      else activeFilters.delete(this.value);
      updateFilterDisplay();
    });
  });

  document.getElementById('search-box').addEventListener('input', function() {
    searchText = this.value.toLowerCase().trim();
    updateFilterDisplay();
  });

  document.getElementById('h-images').textContent = ALL.length;
  document.getElementById('h-boxes').textContent = DATA.stats.total_boxes;
  document.querySelectorAll('.fc-count').forEach(function(span) {
    var cls = span.dataset.cls;
    span.textContent = '(' + (DATA.stats.class_counts[cls] || 0) + ')';
  });

  // Anomalies
  (function() {
    var anomalies = DATA.anomalies;
    var badge = document.getElementById('anomaly-badge');
    var container = document.getElementById('anomalies-content');
    var errCount = anomalies.filter(function(a) { return a.severity === 'error'; }).length;
    if (anomalies.length === 0) {
      badge.style.display = 'none';
      container.innerHTML = '<div class="anomaly-clean">✅ 数据集干净，未发现任何异常。</div>';
    } else {
      badge.style.display = 'inline-block';
      badge.textContent = anomalies.length;
      if (errCount === 0) { badge.classList.add('clean'); }
      var h = '<table class="anomaly-table"><thead><tr>' +
        '<th width="30">#</th><th>文件</th><th width="80">严重级别</th>' +
        '<th>类型</th><th>详情</th></tr></thead><tbody>';
      for (var i = 0; i < anomalies.length; i++) {
        var a = anomalies[i];
        h += '<tr class="sev-' + a.severity + '">' +
          '<td>' + (i + 1) + '</td>' +
          '<td>' + a.file + '</td>' +
          '<td><span class="sev-tag ' + a.severity + '">' + a.severity + '</span></td>' +
          '<td>' + a.type + '</td>' +
          '<td style="font-size:12px;color:#777">' + a.detail + '</td>' +
          '</tr>';
      }
      h += '</tbody></table>';
      container.innerHTML = h;
    }
  })();

  // Stats
  var chartsRendered = false;

  function renderStats() {
    if (chartsRendered) return;
    chartsRendered = true;
    var stats = DATA.stats;
    var cc = stats.class_counts;
    var colors = CLASS_NAMES.map(function(c) { return CLASS_COLORS[c]; });

    new Chart(document.getElementById('chart-class-dist'), {
      type: 'doughnut',
      data: {
        labels: CLASS_NAMES,
        datasets: [{
          data: CLASS_NAMES.map(function(c) { return cc[c]; }),
          backgroundColor: colors,
          borderWidth: 2,
          borderColor: '#fff',
        }]
      },
      options: {
        responsive: true,
        plugins: {
          title: { display: true, text: '类别分布 (' + stats.total_boxes + ' 个框)', font: { size: 14 } },
          legend: { position: 'bottom' },
        },
      }
    });

    var ah = stats.area_histogram;
    var bins = stats.area_bins;
    new Chart(document.getElementById('chart-box-area'), {
      type: 'bar',
      data: {
        labels: bins,
        datasets: CLASS_NAMES.map(function(c, i) {
          return {
            label: c,
            data: ah[c],
            backgroundColor: colors[i],
            borderWidth: 0,
            borderRadius: 3,
          };
        }),
      },
      options: {
        responsive: true,
        plugins: {
          title: { display: true, text: '框面积分布 (px²)', font: { size: 14 } },
          legend: { position: 'bottom' },
        },
        scales: {
          x: { title: { display: true, text: '面积区间' } },
          y: { title: { display: true, text: '框数量' }, beginAtZero: true },
        },
      }
    });

    var opi = stats.obj_per_image;
    var opiKeys = ['0', '1', '2', '3', '4', '5+'];
    new Chart(document.getElementById('chart-obj-per-img'), {
      type: 'bar',
      data: {
        labels: opiKeys,
        datasets: [{
          label: '图片数',
          data: opiKeys.map(function(k) { return opi[k] || 0; }),
          backgroundColor: '#4363D8',
          borderWidth: 0,
          borderRadius: 3,
        }]
      },
      options: {
        responsive: true,
        plugins: {
          title: { display: true, text: '每图目标数分布', font: { size: 14 } },
          legend: { display: false },
        },
        scales: {
          x: { title: { display: true, text: '目标数量' } },
          y: { title: { display: true, text: '图片数' }, beginAtZero: true },
        },
      }
    });

    var at = stats.area_stats;
    var th = '<tr><th>类别</th><th>数量</th><th>最小</th><th>P25</th>' +
      '<th>中位数</th><th>P75</th><th>最大</th><th>均值</th></tr>';
    for (var c = 0; c < CLASS_NAMES.length; c++) {
      var cls = CLASS_NAMES[c];
      var s = at[cls];
      th += '<tr><td><span class="class-badge ' + cls + '">' + cls + '</span></td>' +
        '<td>' + s.total + '</td><td>' + s.min + '</td><td>' + s.p25 + '</td>' +
        '<td>' + s.median + '</td><td>' + s.p75 + '</td><td>' + s.max + '</td>' +
        '<td>' + s.mean + '</td></tr>';
    }
    document.getElementById('area-stats-table').innerHTML = th;
  }

  updateFilterDisplay();

  window.addEventListener('load', function() {
    if (typeof Chart === 'undefined') {
      var chartCards = document.querySelectorAll('.chart-card');
      for (var i = 0; i < chartCards.length; i++) {
        chartCards[i].innerHTML = '<p style="color:#999;text-align:center;padding:40px">'
          + '⚠ Chart.js CDN 加载失败，请连接互联网后刷新页面查看统计图表。</p>';
      }
    }
  });
})();
</script>

</body>
</html>'''


# ============================================================
#  Main
# ============================================================
def main():
    parser = argparse.ArgumentParser(description='标注数据查看器生成器')
    parser.add_argument('-image', default=JPG_DIR_DEFAULT,
                        help=f'图片目录路径 (默认: {JPG_DIR_DEFAULT})')
    parser.add_argument('-ann', default=ANN_DIR_DEFAULT,
                        help=f'标注目录路径 (默认: {ANN_DIR_DEFAULT})')
    parser.add_argument('-output', default=OUT_DIR_DEFAULT,
                        help=f'输出目录路径 (默认: {OUT_DIR_DEFAULT})')
    args = parser.parse_args()

    ann_dir = os.path.normpath(args.ann)
    jpg_dir = os.path.normpath(args.image)
    out_dir = os.path.normpath(args.output)

    print('=' * 50)
    print('  SmartCar 标注数据查看器生成器')
    print('=' * 50)
    print(f'  标注目录: {ann_dir}')
    print(f'  图片目录: {jpg_dir}')
    print(f'  输出目录: {out_dir}')

    # 检查目录
    if not os.path.isdir(ann_dir):
        print(f'错误: 标注目录不存在: {ann_dir}')
        sys.exit(1)
    if not os.path.isdir(jpg_dir):
        print(f'错误: 图片目录不存在: {jpg_dir}')
        sys.exit(1)

    # 加载类别名
    os.chdir(BASE_DIR)  # 确保 utils.py 的 yolo_cfg 能找到 config.cfg
    class_names = load_class_names()
    colors_rgb, colors_hex = get_class_colors(class_names)

    # 计算 image 目录到 output 的 相对路径（HTML 中 lightbox 用）
    jpg_dir_rel = os.path.relpath(jpg_dir, out_dir)

    thumb_dir = os.path.join(out_dir, 'thumbs')

    # 阶段 1: 解析 XML
    images, parse_errors = parse_all_xmls(ann_dir)
    if not images:
        print('错误: 未找到有效标注文件')
        sys.exit(1)

    # 将解析错误加入异常
    anomalies = [{
        'file': pe['file'], 'severity': 'error',
        'type': 'XML 解析失败', 'detail': pe['error'],
    } for pe in parse_errors]

    # 阶段 2+3: 异常检测
    anomalies += check_anomalies(images, ann_dir, jpg_dir, class_names)

    # 阶段 4: 统计
    stats = compute_stats(images, class_names)

    # 阶段 5: 字体 & 缩略图
    try:
        font = ImageFont.truetype('simhei.ttf', 12)
    except Exception:
        try:
            font = ImageFont.truetype('C:/Windows/Fonts/simhei.ttf', 12)
        except Exception:
            font = ImageFont.load_default()

    generate_thumbnails(images, jpg_dir, thumb_dir, font, colors_rgb)

    # 阶段 6: 生成 HTML
    generate_html(images, stats, anomalies, out_dir, jpg_dir_rel,
                  class_names, colors_hex)

    print('=' * 50)
    print('  完成! 浏览器打开:')
    print(f'  {os.path.join(out_dir, "index.html")}')
    print('=' * 50)


if __name__ == '__main__':
    main()
