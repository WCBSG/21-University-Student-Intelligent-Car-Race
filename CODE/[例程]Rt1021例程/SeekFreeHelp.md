# SeekFree 模块帮助文档

本文档汇总了 SeekFree 嵌入式平台下各模块的引脚、初始化参数及 API 说明。

---

## 1. KEY_HANDLER

**引脚**：  
- KEY1 = C8  
- KEY2 = C9  
- KEY3 = C14  
- KEY4 = C15  

### 初始化
```python
object = KEY_HANDLER(period)   # period: 扫描周期(ms)，不能为0
```

### 方法
| 方法 | 参数 | 说明 |
|------|------|------|
| `help()` | 无 | 模块帮助 |
| `info()` | 无 | 模块参数信息 |
| `capture()` | 无 | 触发一次扫描 |
| `get()` | 无 | 获取当前按键状态 |
| `read()` | 无 | 触发一次扫描并获取状态 |
| `clear()` | 无 或 `index` | 清除所有或指定按键状态（index: 1~4） |
| `get_period()` | 无 | 获取当前扫描周期(ms) |

---

## 2. MOTOR_CONTROLLER

### 初始化
```python
MOTOR_CONTROLLER(channel, freq=13000, duty=0, invert=False)
```
- `channel`：可选值  
  `MOTOR_CONTROLLER.PWM_C30_DIR_C31`、`PWM_C28_DIR_C29`、`PWM_D4_DIR_D5`、`PWM_D6_DIR_D7`、  
  `PWM_C30_PWM_C31`、`PWM_C28_PWM_C29`、`PWM_D4_PWM_D5`、`PWM_D6_PWM_D7`
- `freq`：PWM 频率，范围 [1, 100000] Hz，默认 13000
- `duty`：占空比，范围 [-10000, +10000]，默认 0
- `invert`：是否反转，True/False，默认 False

### 方法
| 方法 | 说明 |
|------|------|
| `help()` | 模块帮助 |
| `info()` | 参数信息 |
| `freq()` | 无参返回当前频率；有参设置新频率并重新初始化 |
| `duty()` | 无参返回当前占空比；有参设置新占空比 |

---

## 3. BLDC_CONTROLLER

### 初始化
```python
BLDC_CONTROLLER(channel, freq=50, highlevel_us=0)
```
- `channel`：`BLDC_CONTROLLER.PWM_C25` 或 `PWM_C27`
- `freq`：PWM 频率，范围 [50, 300] Hz，默认 50
- `highlevel_us`：高电平时间，范围 [1000, 2000] µs，默认 0

### 方法
| 方法 | 说明 |
|------|------|
| `help()` | 模块帮助 |
| `info()` | 参数信息 |
| `freq()` | 无参返回当前频率；有参设置新频率并重新初始化 |
| `highlevel_us()` | 无参返回当前高电平时间；有参设置新值 |

---

## 4. IMU660RX

**支持传感器**：IMU660RA / IMU660RB / IMU660RC  
**默认量程**：加速度 ±8g，陀螺仪 ±2000dps  
**默认输出频率**：  
- 加速度：RA-50Hz / RB-52Hz / RC-60Hz  
- 陀螺仪：RA-200Hz / RB-208Hz / RC-480Hz  

**引脚**：SPI-Index=1 (LPSPI2)  
- IMU660RX_SPC = C10  
- IMU660RX_SDI = C12  
- IMU660RX_SDO = C13  
- IMU660RX_CS = C11  
- IMU660RX_INT2 = D19  

### 初始化
```python
IMU660RX(capture_div=1, imu_type=IMU660RX.TYPE_AUTO, quar_rate=IMU660RX.RATE_DISABLE)
```
- `capture_div`：多少次触发后刷新缓冲区，默认 1
- `imu_type`：`TYPE_AUTO` / `TYPE_RA` / `TYPE_RB` / `TYPE_RC`
- `quar_rate`：四元数输出频率，可选 `RATE_15HZ` / `30HZ` / `60HZ` / `120HZ` / `240HZ` / `480HZ` / `DISABLE`  
  **注意**：若 `quar_rate` 非 `DISABLE`，INT2 必须接 D19，且不能使用 `Ticker.capture_list()` 处理。

### 方法
| 方法 | 返回值 | 说明 |
|------|--------|------|
| `help()` | - | 模块帮助 |
| `info()` | - | 参数信息 |
| `capture()` | - | 触发一次采集 |
| `get()` | `(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)` | 获取加速度和陀螺仪数据 |
| `get_euler()` | `(roll, pitch, yaw)` | 获取欧拉角（仅在非 DISABLE 模式更新） |
| `get_quarternion()` | `(x, y, z, w)` | 获取四元数（仅在非 DISABLE 模式更新） |
| `read()` | 同 `get()` | 触发一次采集并返回数据 |
| `get_capture_div()` | int | 获取当前 capture_div |

---

## 5. IMU963RX

**支持传感器**：IMU963RA  
**默认量程**：加速度 ±8g，陀螺仪 ±2000dps，磁力计 8G  
**输出频率**：加速度 52Hz，陀螺仪 208Hz，磁力计 52Hz  

**引脚**：同 IMU660RX（SPI1，`C10,C12,C13,C11,D19`）

### 初始化
```python
IMU963RX(capture_div=1, imu_type=IMU963RX.TYPE_AUTO, quar_rate=IMU963RX.RATE_DISABLE)
```
- `imu_type`：`TYPE_AUTO` / `TYPE_RA`
- `quar_rate`：仅 `DISABLE`（当前版本不支持四元数）

### 方法
| 方法 | 返回值 | 说明 |
|------|--------|------|
| `help()` | - | 模块帮助 |
| `info()` | - | 参数信息 |
| `capture()` | - | 触发一次采集 |
| `get()` | `(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z)` | 获取所有数据 |
| `read()` | 同上 | 触发一次采集并返回数据 |
| `get_capture_div()` | int | 获取 capture_div |

---

## 6. WIRELESS_UART

**引脚**：UART-Index=2 (LPUART3)  
- WIRELESS_UART_TX = C7  
- WIRELESS_UART_RX = C6  
- WIRELESS_UART_RTS = C5  

### 初始化
```python
WIRELESS_UART(baudrate=460800)
```
- `baudrate`：范围 [4800, 2000000]，默认 460800

### 方法
| 方法 | 说明 |
|------|------|
| `help()` | 模块帮助 |
| `info()` | 参数信息 |
| `send_str(string)` | 发送字符串 |
| `send_oscilloscope(data1, ..., data8)` | 发送最多 8 个浮点数据到上位机示波器 |
| `send_ccd_image(index, color)` | 发送 CCD 图像，`index` 为 `CCD1_BUFFER_INDEX` ~ `CCD4_BUFFER_INDEX`，`color` 为 RGB565 |
| `data_analysis()` | 解析上位机数据 |
| `get_data(index)` | 获取解析后的数据，index [0,7] |
| `receive_bytearray(array, length)` | 接收指定长度字节数组，返回实际长度 |
| `send_bytearray(array, length)` | 发送字节数组 |

---

## 7. WIFI_SPI

**引脚**：SPI-Index=3 (LPSPI4)  
- WIFI_SPI_SCK = B18  
- WIFI_SPI_MOSI = B20  
- WIFI_SPI_MISO = B21  
- WIFI_SPI_CS = B19  
- WIFI_SPI_INT = B3  
- WIFI_SPI_RST = B4  

### 初始化
```python
WIFI_SPI(wifi_ssid, pass_word, connect_type, target_ip, port)
```
- `wifi_ssid`：热点名称（字符串）
- `pass_word`：密码（字符串）
- `connect_type`：`WIFI_SPI.TCP_CONNECT` 或 `UDP_CONNECT`
- `target_ip`：目标 IP（字符串）
- `port`：端口号（字符串）

### 方法
| 方法 | 说明 |
|------|------|
| `help()` | 模块帮助 |
| `info()` | 参数信息 |
| `send_str(string)` | 发送字符串 |
| `send_oscilloscope(data1, ..., data8)` | 发送最多 8 个浮点数据到上位机示波器 |
| `send_ccd_image(index, color)` | 发送 CCD 图像 |
| `data_analysis()` | 解析上位机数据 |
| `get_data(index)` | 获取解析后的数据，index [0,7] |
| `receive_bytearray(array, length)` | 接收指定长度字节数组，返回实际长度 |
| `send_bytearray(array, length)` | 发送字节数组 |

---

## 8. DL1X

**支持传感器**：DL1A / DL1B  
**最大输出频率**：DL1A 1.2M@30Hz，DL1B 1.4M@100Hz  

**引脚**：  
- DL1x_SCL = D0  
- DL1x_SDA = D1  
- DL1x_XS = D2  
- DL1x_INT = D3  

### 初始化
```python
DL1X(capture_div=1)
```

### 方法
| 方法 | 说明 |
|------|------|
| `help()` | 模块帮助 |
| `info()` | 参数信息 |
| `capture()` | 触发一次采集 |
| `get()` | 获取当前缓冲区数据 |
| `read()` | 触发采集并返回数据 |
| `get_capture_div()` | 获取 capture_div |

---

## 9. TSL1401

**支持传感器**：TSL1401  
**引脚**：  
- TSL1401_CLK = B13  
- TSL1401_SI = B16  
- TSL1401_AO1 = B22  
- TSL1401_AO2 = B23  
- TSL1401_AO3 = B24  
- TSL1401_AO4 = B25  

### 初始化
```python
TSL1401(capture_div=1)
```

### 方法
| 方法 | 说明 |
|------|------|
| `help()` | 模块帮助 |
| `info()` | 参数信息 |
| `capture()` | 触发一次采集 |
| `get(index)` | 获取指定索引（0~3）的 CCD 数据 |
| `read(index)` | 触发采集并返回指定索引数据 |
| `get_capture_div()` | 获取 capture_div |
| `set_resolution(res)` | 设置分辨率：`RES_8BIT` / `RES_10BIT` / `RES_12BIT` |

---
