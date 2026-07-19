import os
from time import ticks_ms
_buf = []
_buf_bytes = 0
_path = '/flash/log.txt'
MAX_KB = 6144
_FLUSH_BYTES = 512
_FLUSH_LINES = 8
def _flush():
  global _buf, _buf_bytes
  if not _buf:
    return
  data = ''.join(_buf)
  _buf = []
  _buf_bytes = 0
  try:
    with open(_path, 'a') as f:
      f.write(data)
  except Exception:
    pass
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
  global _buf_bytes
  _buf.append(line)
  _buf_bytes += len(line)
  if _buf_bytes >= _FLUSH_BYTES or len(_buf) >= _FLUSH_LINES:
    _flush()
def flush():
  _flush()
def info(tag, msg):
  line = "[%6.2f] [%s] %s" % (ticks_ms() / 1000, tag, msg)
  _write(line + '\n')
