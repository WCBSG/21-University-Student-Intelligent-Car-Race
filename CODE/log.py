"""
log.py — 统一日志接口（仅写文件）
  info(tag, msg) → [t+sss.ss] [TAG] msg → /flash/log.txt
"""
import os
from time import ticks_ms

_buf = ''
_path = '/flash/log.txt'
MAX_KB = 6144  # 6MB


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
  _buf += line
  if '\n' in _buf:
    _flush()


def info(tag, msg):
  """带标签日志: [t+sss.ss] [TAG] msg → 文件"""
  line = "[%6.2f] [%s] %s" % (ticks_ms() / 1000, tag, msg)
  _write(line + '\n')
