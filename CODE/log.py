"""
log.py — 统一日志接口
  info(tag, msg)   → [TAG] msg
  setup(enabled)   → 开关文件写入
"""
import os

_print = print           # 保存原始 print（在任何 override 之前）
_enabled = False
_buf = ''
_path = '/flash/log.txt'
MAX_KB = 6144  # 6MB


def setup(enabled):
  global _enabled
  _enabled = bool(enabled)


def _flush():
  global _buf
  if not _buf:
    return
  try:
    with open(_path, 'a') as f:
      f.write(_buf)
  except Exception:
    pass
  _buf = ''
  # 超限滚动：重命名旧文件，新建日志
  try:
    st = os.stat(_path)
    if st[6] > MAX_KB * 1024:
      try:
        os.rename(_path, _path + '.old')
      except Exception:
        os.remove(_path)
  except Exception:
    pass


def _write(line):
  global _buf
  if not _enabled:
    return
  _buf += line
  if '\n' in _buf:
    _flush()


def info(tag, msg):
  """带标签日志: [TAG] msg"""
  line = "[%s] %s" % (tag, msg)
  _print(line)
  _write(line + '\n')

