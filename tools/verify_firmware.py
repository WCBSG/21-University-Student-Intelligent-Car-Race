#!/usr/bin/env python3
"""PC-side syntax and source/flash safety checks."""

import ast
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CODE = ROOT / "CODE"
FLASH = CODE / ".flash"
RESULT = Path(__file__).with_suffix(".result.txt")

MODULES = (
  "main.py",
  "match.py",
  "motion.py",
  "camera.py",
  "config.py",
  "tcs3472.py",
  "imu.py",
  "log.py",
)

REQUIRED_MATCH_TEXT = (
  "rot = self._cfg.yaw_actuation_sign * self._hdg_pid.update(err, dt)",
  "PUSH timeout %dms — NOT scored",
  'self._fault("HOME timeout — gate not confirmed")',
  'self._sub = "CLEAR"',
  'self.phase = "ORBIT"',
  'self.phase = "FINAL_APPROACH"',
  "def navigation_snapshot(self, sensors=None):",
  "self._approach_deadline",
)

STALE_FILES = (
  "runner.py",
  "fsm.py",
  "ctrl.py",
  "Motor.py",
  "bitbang_i2c.py",
)


def main():
  errors = []

  for root in (CODE, FLASH):
    for name in MODULES:
      path = root / name
      try:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
      except Exception as exc:
        errors.append("%s: %s" % (path.relative_to(ROOT), exc))

  for name in STALE_FILES:
    for root in (CODE, FLASH):
      path = root / name
      if path.exists():
        errors.append("%s: stale file remains after merge" %
                      path.relative_to(ROOT))

  for path in (CODE / "config.json", FLASH / "config.json"):
    try:
      data = json.loads(path.read_text(encoding="utf-8"))
      if int(data.get("清线时间", -1)) < 0:
        errors.append("%s: invalid 清线时间" % path.relative_to(ROOT))
      if not bool(data.get("严格目标", False)):
        errors.append("%s: 严格目标 must be enabled" % path.relative_to(ROOT))
      if int(data.get("目标个数", -1)) != 1:
        errors.append("%s: 目标个数 must be 1 for single-car acceptance" %
                      path.relative_to(ROOT))
      if float(data.get("航向执行极性", 0.0)) != -1.0:
        errors.append("%s: 航向执行极性 must match measured -1" %
                      path.relative_to(ROOT))
      if int(data.get("绕物总超时", 0)) <= 0:
        errors.append("%s: invalid 绕物总超时" % path.relative_to(ROOT))
    except Exception as exc:
      errors.append("%s: %s" % (path.relative_to(ROOT), exc))

  for path in (CODE / "match.py", FLASH / "match.py"):
    try:
      text = path.read_text(encoding="utf-8")
      for required in REQUIRED_MATCH_TEXT:
        if required not in text:
          errors.append("%s: missing %r" % (path.relative_to(ROOT), required))
      if "PRE_PUSH" in text:
        errors.append("%s: stale PRE_PUSH path remains" % path.relative_to(ROOT))
    except Exception as exc:
      errors.append("%s: %s" % (path.relative_to(ROOT), exc))

  key_param = ROOT / "KeyParam.md"
  if not key_param.exists():
    key_param = ROOT / "docs" / "KeyParam.md"
  try:
    text = key_param.read_text(encoding="utf-8")
    for required in ("`场心航向 = 0.0°`", "y2 = 78.4", "C ≈ 5475"):
      if required not in text:
        errors.append("%s: missing %r" % (
          key_param.relative_to(ROOT), required))
  except Exception as exc:
    errors.append("%s: %s" % (key_param.relative_to(ROOT), exc))

  lines = ["FAIL" if errors else "PASS"]
  lines.extend(errors)
  result = "\n".join(lines) + "\n"
  RESULT.write_text(result, encoding="utf-8")
  print(result, end="")
  return 1 if errors else 0


if __name__ == "__main__":
  raise SystemExit(main())
