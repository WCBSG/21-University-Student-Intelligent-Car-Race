from CameraReceiver import CameraReceiver, recv_frame as _recv_frame
from time import sleep_ms

c = CameraReceiver()
print("Init OK, waiting for data...")
sleep_ms(3000)  # OpenART 10fps → 应该积了 ~30 帧
print("uart any:", c._uart.any())

# 手动试收一帧
cmd, payload = _recv_frame(c._uart)
if cmd is not None:
    plen = len(payload) if payload else 0
    print("recv: cmd=0x%02X len=%d" % (cmd, plen))
else:
    print("recv: None")

# 连续收10帧
for i in range(10):
    sleep_ms(100)
    cmd, payload = _recv_frame(c._uart)
    if cmd is not None:
        plen = len(payload) if payload else 0
        print("  f%d: cmd=0x%02X len=%d" % (i, cmd, plen))
