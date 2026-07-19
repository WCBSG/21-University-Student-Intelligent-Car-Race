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
- 若 config.json 中 "调试输出"=false，再剥离 log.py 与所有 info/log_setup 调用

用法 (在仓库根目录):
  python CODE/build_flash.py
  python CODE/build_flash.py --src CODE --dst CODE/.flash
"""

from __future__ import print_function

import argparse
import ast
import io
import json
import os
import re
import shutil
import sys
import tokenize


# 不处理的目录和文件
SKIP_DIR_NAMES = {".flash", "__pycache__", ".git", u"例程", u"[例程]Rt1021例程"}
# build_flash.py 自身不拷进 .flash (避免脚本自引用); 大体积/不必要文件也可加这里
SKIP_FILE_NAMES = {
    "build_flash.py",
    # 测试/调试脚本 — 不上车
    "imu_test_params.txt",
    "test_imu.py",
    "test_align.py",
    "test_leave.py",
    "test_hunt.py",
    "test_push.py",
    "test_backoff.py",
    "test_home.py",
    "log.txt",
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
    in_triple = False
    triple_delim = None
    i = 0

    while i < len(lines):
        line = lines[i]

        # --- 多行 docstring / 多行字符串内 ---
        if in_triple:
            if triple_delim in line:
                # 闭合行带尾随后缀（极少见, 如 `"""line""" + bar`）→ 单独成行会语法错误,
                # 安全做法: 整行视为 docstring 闭合, 后缀丢弃 (按需自行重写源)
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

        # 删除空行
        if not line:
            i += 1
            continue

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


# 日志剥离: config.json "调试输出"=false 时启用
# 默认白名单 + 运行时累加 from log import 引入的所有别名（含裸 setup 等）
_LOG_FUNCS = ("info", "log_setup")


def read_debug_output(json_path):
    """读取 config.json 的 "调试输出" 字段。返回 True/False/None (None=缺/无法解析)。"""
    if not os.path.isfile(json_path):
        return None
    try:
        with open(json_path, "r", encoding="utf-8") as f:
            raw = f.read()
        cleaned = strip_json_comments(raw)
        data = json.loads(cleaned)
        val = data.get("调试输出")
        if isinstance(val, bool):
            return val
    except Exception as e:
        print("[WARN] cannot read debug flag %s: %s" % (json_path, e))
    return None


def strip_log_lines(src_text):
    """Tokenize 级: 精确删除 info/log_setup 调用 token + log import 行，
    不重排源码格式（不调 ast.unparse）。空块补 pass。

    解析失败时原样返回 (交给 strip_python 再剥注释)。
    """
    if isinstance(src_text, bytes):
        src_text = src_text.decode("utf-8")
    try:
        tree = ast.parse(src_text)
    except (SyntaxError, ValueError):
        return src_text

    # ── 1) 收集 from log import 别名 ──
    log_aliases = set(_LOG_FUNCS)
    import_drops = []  # (sl, sc, el, ec) 1-based line, 0-based col
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module == "log":
            for alias in node.names:
                log_aliases.add(alias.asname or alias.name)
            import_drops.append((node.lineno, node.col_offset,
                                  node.end_lineno, node.end_col_offset))
        elif isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name == "log":
                    import_drops.append((alias.lineno, alias.col_offset,
                                          alias.end_lineno, alias.end_col_offset))

    # ── 2) tokenize 找出 info/log_setup 调用范围 ──
    readline = io.StringIO(src_text).readline
    try:
        tokens = list(tokenize.generate_tokens(readline))
    except (tokenize.TokenizeError, IndentationError):
        return src_text

    call_drops = []

    def _find_matching_paren(start_idx):
        depth = 1
        j = start_idx
        while j < len(tokens):
            t = tokens[j]
            if t.type == tokenize.OP:
                if t.string == '(':
                    depth += 1
                elif t.string == ')':
                    depth -= 1
                    if depth == 0:
                        return j
            j += 1
        return -1

    i = 0
    while i < len(tokens):
        tok = tokens[i]
        if (tok.type == tokenize.NAME and tok.string in log_aliases and
            i + 1 < len(tokens) and
            tokens[i + 1].type == tokenize.OP and tokens[i + 1].string == '('):
            j = _find_matching_paren(i + 2)
            if j > 0:
                call_drops.append((tok.start[0], tok.start[1],
                                    tokens[j].end[0], tokens[j].end[1]))
                i = j + 1
                continue
        elif (tok.type == tokenize.OP and tok.string == '.' and
              i + 2 < len(tokens) and
              tokens[i + 1].type == tokenize.NAME and tokens[i + 1].string in log_aliases and
              tokens[i + 2].type == tokenize.OP and tokens[i + 2].string == '('):
            j = _find_matching_paren(i + 3)
            if j > 0:
                if i > 0 and tokens[i - 1].type == tokenize.NAME and tokens[i - 1].string in ('log', 'l'):
                    start_sl, start_sc = tokens[i - 1].start
                else:
                    start_sl, start_sc = tok.start
                call_drops.append((start_sl, start_sc,
                                    tokens[j].end[0], tokens[j].end[1]))
                i = j + 1
                continue
        i += 1

    all_drops = import_drops + call_drops

    # ── 3) (sl, sc) → 字符索引 ──
    lines = src_text.splitlines(keepends=True)
    line_starts = [0]
    for ln in lines:
        line_starts.append(line_starts[-1] + len(ln))

    def _to_off(sl, sc):
        if sl - 1 >= len(line_starts):
            return len(src_text)
        return line_starts[sl - 1] + sc

    char_ranges = []
    for sl, sc, el, ec in all_drops:
        s = _to_off(sl, sc)
        e = _to_off(el, ec)
        if s < e:
            char_ranges.append((s, e))
    char_ranges.sort(key=lambda r: (r[0], r[1]))

    # ── 4) 应用删除 ──
    result = src_text
    for s, e in sorted(char_ranges, key=lambda r: -r[0]):
        result = result[:s] + result[e:]

    # ── 5) 合并连续空行 ──
    rlines = result.split('\n')
    merged = []
    prev_blank = False
    for ln in rlines:
        is_blank = (ln.strip() == '')
        if is_blank and prev_blank:
            continue
        merged.append(ln)
        prev_blank = is_blank
    result = '\n'.join(merged)

    # ── 6) 空块补 pass: 在 result 上扫描块头行 + 看下一行是否为空 ──
    # 块头行: 缩进 + (if|elif|else:|for|while|try:|except|finally:|with|def|class) + ... + :
    BLOCK_HEADER = re.compile(r'^(\s*)(if |elif |else:|for |while |try:|except\b|finally:|with |def |class )')

    new_lines = result.split('\n')
    # 找所有块头行
    block_headers = []  # (line_idx, indent)
    for idx, ln in enumerate(new_lines):
        m = BLOCK_HEADER.match(ln)
        if not m:
            continue
        indent = len(ln) - len(ln.lstrip())
        # 块头必须以 : 结尾 (注释前): 检查 # 之前的部分
        hash_pos = ln.find('#')
        check_part = ln[:hash_pos] if hash_pos >= 0 else ln
        if check_part.rstrip().endswith(':'):
            block_headers.append((idx, indent))

    # 对每个块头, 看下方第一行（缩进更大的）是否是空行/已 pass/不在新行
    # 如果该行是空 → 替换为 `indent + 2 个空格 + pass`
    inserts = []  # (line_idx, new_line_content)
    for idx, indent in block_headers:
        if idx + 1 >= len(new_lines):
            continue
        next_line = new_lines[idx + 1]
        next_indent = len(next_line) - len(next_line.lstrip())
        next_content = next_line.strip()
        # 块空条件: 下一行缩进 <= 块缩进（同级块头/更低）→ 块空
        if next_indent <= indent:
            pass_text = ' ' * (indent + 2) + 'pass'
            inserts.append((idx + 1, pass_text))
        elif next_indent > indent and next_content == '':
            pass_text = ' ' * (indent + 2) + 'pass'
            inserts.append((idx + 1, pass_text))
        # else: 下一行有内容 → 块不空

    # 应用替换（按行号倒序）
    for idx, new_content in sorted(inserts, key=lambda x: -x[0]):
        new_lines[idx] = new_content

    return '\n'.join(new_lines)


def strip_json_comments(text):
    """去掉 JSON 注释：独立行 "//…" / "__…" 整行删除；行内 ,"__…": value 移除。"""
    # 匹配: ,"__key": <string|array|simple_value>
    _INLINE = re.compile(
        r'\s*,\s*"__[^"]*"\s*:\s*'
        r'("(?:[^"\\]|\\.)*"|\[[^\]]*\]|true|false|null|-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)'
    )
    lines = text.split('\n')
    kept = []
    for line in lines:
        s = line.strip()
        # 整行注释（段标题 "//…" 或独立 __ 行）
        if s.startswith('"//') or s.startswith('"__'):
            continue
        # 移除行内 __ 注释
        line = _INLINE.sub('', line)
        if not line.strip():
            continue
        kept.append(line)
    text = '\n'.join(kept)
    # 修复尾随逗号
    text = re.sub(r',\s*\n\s*}', '\n}', text)
    text = re.sub(r',\s*\n\s*]', '\n]', text)
    return text


def strip_json_for_build(text, name, drop_debug=False):
    """strip_json_comments 后再加一层: 若 drop_debug 且为 config.json, 删除 "调试输出" 键。"""
    cleaned = strip_json_comments(text)
    if not drop_debug or name != "config.json":
        return cleaned
    try:
        data = json.loads(cleaned)
    except Exception:
        return cleaned
    if isinstance(data, dict) and "调试输出" in data:
        del data["调试输出"]
        try:
            return json.dumps(data, ensure_ascii=False, indent=2) + "\n"
        except Exception:
            return cleaned
    return cleaned


def process_tree(src_root, dst_root, strip_log=False):
    src_root = os.path.abspath(src_root)
    dst_root = os.path.abspath(dst_root)

    if os.path.isdir(dst_root):
        shutil.rmtree(dst_root)
    os.makedirs(dst_root)

    n_py, n_skip, n_err = 0, 0, 0
    bytes_in, bytes_out = 0, 0

    # 单层遍历: 仅处理 src_root 顶层文件（CODE/ 不放子目录, 例程目录跳过即可）
    for name in os.listdir(src_root):
        src_path = os.path.join(src_root, name)
        if os.path.isdir(src_path):
            if should_skip_dir(name):
                n_skip += 1
                print("[SKIP-DIR] %s" % src_path)
            else:
                n_skip += 1
                print("[WARN-SUBDIR] %s not processed (build_flash 单层构建)" % src_path)
            continue

        dst_path = os.path.join(dst_root, name)

        if name.endswith(".py"):
            if should_skip_file(name):
                n_skip += 1
                print("[SKIP] %s" % src_path)
                continue
            # 调试输出关闭时: log.py 不写；其它 .py 先做 AST 级 log 剥离
            if strip_log and name == "log.py":
                print("[DROP] %s (debug output disabled)" % src_path)
                continue
            try:
                with open(src_path, "r", encoding="utf-8") as f:
                    raw = f.read()
                if strip_log:
                    raw = strip_log_lines(raw)
                stripped = strip_python(raw)
                with open(dst_path, "w", encoding="utf-8", newline="\n") as f:
                    f.write(stripped)
                n_py += 1
                bytes_in += len(raw.encode("utf-8"))
                bytes_out += len(stripped.encode("utf-8"))
            except Exception as e:
                n_err += 1
                print("[ERR] %s: %s" % (src_path, e))
        else:
            if should_skip_file(name):
                n_skip += 1
                print("[SKIP] %s" % src_path)
            else:
                n_skip += 1
                print("[SKIP-OTHER] %s (仅处理 .py/.json)" % src_path)

    return n_py, n_skip, n_err, bytes_in, bytes_out


def main():
    ap = argparse.ArgumentParser(description="Strip CODE/ into CODE/.flash/")
    ap.add_argument("--src", default="CODE", help="source tree")
    ap.add_argument("--dst", default=os.path.join("CODE", ".flash"), help="output tree")
    ap.add_argument("--strip-log", action="store_true", help="strip log.py and all info/log_setup calls")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)
    src = args.src if os.path.isabs(args.src) else os.path.join(root, args.src)
    dst = args.dst if os.path.isabs(args.dst) else os.path.join(root, args.dst)

    if not os.path.isdir(src):
        print("source not found: %s" % src)
        return 1

    strip_log = args.strip_log
    print("src: %s" % src)
    print("dst: %s" % dst)
    print("strip_log=%s" % strip_log)
    n_py, n_skip, n_err, bin_, bout = process_tree(src, dst, strip_log=strip_log)
    saved = bin_ - bout
    pct = (100.0 * saved / bin_) if bin_ else 0.0
    print("py: %d  skipped: %d  errors: %d" % (n_py, n_skip, n_err))
    print("bytes in: %d  out: %d  saved: %d (%.1f%%)" % (bin_, bout, saved, pct))
    return 0 if n_err == 0 else 2


if __name__ == "__main__":
    sys.exit(main())
