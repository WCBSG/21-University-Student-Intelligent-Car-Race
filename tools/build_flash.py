#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
build_flash.py — 从 CODE/ 生成去注释、去 docstring、去空行的精简副本到 CODE/.flash/

用途: MCU flash/RAM 共用时，上传精简版节省空间。
- 去除 # 注释
- 去除 docstring（模块/类/函数）
- 压缩多余空行（连续 ≥3 → 1）
- 去除尾随空格
- 跳过例程、标定脚本等不上车的文件

用法 (在仓库根目录):
  python tools/build_flash.py
  python tools/build_flash.py --src CODE --dst CODE/.flash
"""

from __future__ import print_function

import argparse
import io
import os
import re
import shutil
import sys
import tokenize


# 不处理的目录和文件
SKIP_DIR_NAMES = {".flash", "__pycache__", ".git", u"例程", u"[例程]Rt1021例程"}
SKIP_FILE_NAMES = {
    "calibrate_tcs.py",        # 标定脚本，不上车
}


def strip_python(src_text):
    """
    去除 Python 源码中的注释、docstring、多余空行、尾随空格。
    分两步: ① tokenize 安全去注释  ② 行级去 docstring + 清洗 artifact
    """
    if isinstance(src_text, bytes):
        src_text = src_text.decode("utf-8")

    # ── 第 1 步: tokenize 去除 COMMENT ────────────────────
    readline = io.StringIO(src_text).readline
    kept_tokens = []
    prev_was_nl = False

    for tok in tokenize.generate_tokens(readline):
        ttype = tok.type
        if ttype == tokenize.COMMENT:
            continue
        if ttype == tokenize.ENCODING:
            continue
        if ttype == tokenize.ENDMARKER:
            continue
        if ttype == tokenize.NL:
            if prev_was_nl:
                continue
            prev_was_nl = True
        elif ttype == tokenize.NEWLINE:
            prev_was_nl = False
        else:
            prev_was_nl = False
        kept_tokens.append(tok)

    text = tokenize.untokenize(kept_tokens)

    # ── 第 2 步: 行级后处理 ────────────────────────────────
    lines = text.split('\n')
    result = []
    prev_blank = False
    in_triple = False
    triple_delim = None
    i = 0

    while i < len(lines):
        line = lines[i]

        # --- 多行 docstring / 多行字符串内 ---
        if in_triple:
            if triple_delim in line:
                in_triple = False
                triple_delim = None
            i += 1
            continue

        # --- 检测多行 docstring 开始 ---
        m = re.match(r'^(\s*)(\"\"\"|\'\'\')', line)
        if m:
            delim = m.group(2)
            # 同行闭合？(单行 triple-quoted docstring)
            rest = line[m.end():]
            if delim in rest and not rest.rstrip(delim).endswith(delim):
                # 单行: """xxx""" — 跳过
                i += 1
                continue
            elif delim not in rest:
                # 多行 docstring: """\n...\n""" — 进入 tripe 态，跳过
                in_triple = True
                triple_delim = delim
                i += 1
                continue
            # else: "x""" 或 """x"""y — 不是真 docstring，继续处理

        # --- 单行 docstring (单/双引号) ---
        stripped = line.strip()
        if _is_docstring_line(stripped):
            i += 1
            continue

        # --- 清理 ---
        line = line.rstrip()

        # 去除 tokenize artifact: 单独的 \ 行（续行残留），可能带缩进
        if line.strip() == '\\':
            i += 1
            continue

        # 压缩空行
        if not line:
            if not prev_blank and result:
                result.append('')
                prev_blank = True
            i += 1
            continue
        prev_blank = False

        result.append(line)
        i += 1

    # 去除尾部空行
    while result and not result[-1]:
        result.pop()

    return '\n'.join(result) + '\n'


def _is_docstring_line(stripped):
    # 判单行 docstring: 整行只有一段字符串字面量
    # 例: """doc""", '''doc''', "doc", 'doc'
    # 不会误杀: x="val", _log("msg"), return{"k":v}
    if not stripped:
        return False
    # triple-quoted 单行
    if (stripped.startswith('"""') and len(stripped) > 5
        and stripped.endswith('"""') and not stripped.startswith('""" ')):
        return True
    if (stripped.startswith("'''") and len(stripped) > 5
        and stripped.endswith("'''") and not stripped.startswith("''' ")):
        return True
    # single-quoted 单行 (只有 "xxx" 或 'xxx')
    if (stripped.startswith('"') and stripped.endswith('"')
        and stripped.count('"') == 2 and not stripped.endswith('\\"')):
        return True
    if (stripped.startswith("'") and stripped.endswith("'")
        and stripped.count("'") == 2 and not stripped.endswith("\\'")):
        return True
    return False


def should_skip_dir(name):
    if name in SKIP_DIR_NAMES:
        return True
    if u"例程" in name or "[例程]" in name:
        return True
    return False


def should_skip_file(name):
    return name in SKIP_FILE_NAMES


def process_tree(src_root, dst_root):
    src_root = os.path.abspath(src_root)
    dst_root = os.path.abspath(dst_root)

    if os.path.isdir(dst_root):
        shutil.rmtree(dst_root)
    os.makedirs(dst_root)

    n_py, n_skip, n_err = 0, 0, 0
    bytes_in, bytes_out = 0, 0

    for dirpath, dirnames, filenames in os.walk(src_root):
        dirnames[:] = [d for d in dirnames if not should_skip_dir(d)]
        rel = os.path.relpath(dirpath, src_root)
        if rel == ".":
            rel = ""

        abs_dir = os.path.abspath(dirpath)
        if abs_dir == dst_root or abs_dir.startswith(dst_root + os.sep):
            dirnames[:] = []
            continue

        out_dir = os.path.join(dst_root, rel) if rel else dst_root
        if not os.path.isdir(out_dir):
            os.makedirs(out_dir)

        for name in filenames:
            src_path = os.path.join(dirpath, name)
            dst_path = os.path.join(out_dir, name)

            if name.endswith(".py"):
                if should_skip_file(name):
                    n_skip += 1
                    print("[SKIP] %s" % src_path)
                    continue
                try:
                    with open(src_path, "r", encoding="utf-8") as f:
                        raw = f.read()
                    stripped = strip_python(raw)
                    with open(dst_path, "w", encoding="utf-8", newline="\n") as f:
                        f.write(stripped)
                    n_py += 1
                    bytes_in += len(raw.encode("utf-8"))
                    bytes_out += len(stripped.encode("utf-8"))
                except Exception as e:
                    n_err += 1
                    print("[ERR] %s: %s" % (src_path, e))
            # .json / .md / .txt 等不复制

    return n_py, n_skip, n_err, bytes_in, bytes_out


def main():
    ap = argparse.ArgumentParser(description="Strip CODE/ into CODE/.flash/")
    ap.add_argument("--src", default="CODE", help="source tree")
    ap.add_argument("--dst", default=os.path.join("CODE", ".flash"), help="output tree")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)
    src = args.src if os.path.isabs(args.src) else os.path.join(root, args.src)
    dst = args.dst if os.path.isabs(args.dst) else os.path.join(root, args.dst)

    if not os.path.isdir(src):
        print("source not found: %s" % src)
        return 1

    print("src: %s" % src)
    print("dst: %s" % dst)
    n_py, n_skip, n_err, bin_, bout = process_tree(src, dst)
    saved = bin_ - bout
    pct = (100.0 * saved / bin_) if bin_ else 0.0
    print("py: %d  skipped: %d  errors: %d" % (n_py, n_skip, n_err))
    print("bytes in: %d  out: %d  saved: %d (%.1f%%)" % (bin_, bout, saved, pct))
    return 0 if n_err == 0 else 2


if __name__ == "__main__":
    sys.exit(main())
