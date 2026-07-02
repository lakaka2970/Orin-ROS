# Orin CAN 解析库使用说明

> 模块: `orin_can_parser.py` | 接口: `can0` (SocketCAN) | 字节序: Motorola (MSB)

---

## 1. 前置条件

```bash
# 配置并启动 can0（如尚未配置）
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# 安装 python-can
pip install python-can
```

---

## 2. 支持的报文

| 报文ID | 报文名称 | 发送节点 | 周期 | 主要物理信号 |
|--------|----------|----------|------|-------------|
| 0x0E0 | VddmChas1Fr05 | BBM | 20ms | 纵向车速 |
| 0x080 | TcmChas1Fr08 | TCM | 30ms | 档位 |
| 0x1B0 | VddmChas1Fr14 | VDDM | 30ms | 横摆角速度、侧倾角速度 |
| 0x0A0 | VddmChas1Fr03 | VDDM | 15ms | 纵向/横向/垂向加速度 |
| 0x04E | PscmChas1Fr07 | PSCM1 | 10ms | 转向角/转向角速度/方向盘扭矩 |

---

## 3. API 一览

| 函数 | 输入 | 输出 | 说明 |
|------|------|------|------|
| `parse_message(msg_id, data)` | `int`, `bytes` | `dict` | 解析报文全部信号，返回物理值 |
| `check_validity(msg_id, parsed)` | `int`, `dict` | `dict` | 基于 UB/QF 标注信号有效性 |
| `get_gear_str(gear_value)` | `int` | `str` | 档位原始值 → 字符串 |
| `get_counter(msg_id, parsed)` | `int`, `dict` | `int\|None` | 获取活体计数器 |
| `extract_motorola_signal(data, start_bit, bit_length, is_signed)` | 各参数 | `int` | 底层：提取 Motorola 字节序信号原始值 |
| `parse_signal(data, signal)` | `bytes`, `SignalConfig` | `float` | 底层：解析单个信号物理值 |

### `parse_message` 返回值格式

```python
{
    '信号名称': {
        'value': 物理值 (float),
        'raw':   原始整数值 (int),
        'unit':  单位 (str),
    },
    ...
}
```

### `check_validity` 返回值格式

```python
{
    '信号名称': True/False,
    ...
}
```

---

## 4. 完整调用示例（msg 模式）

```python
import can
from orin_can_parser import parse_message, check_validity, get_gear_str, get_counter

# 打开 can0
bus = can.interface.Bus(channel='can0', interface='socketcan')
print("CAN监听已启动，等待报文...")

try:
    while True:
        msg = bus.recv(timeout=1.0)
        if msg is None:
            continue

        parsed = parse_message(msg.arbitration_id, msg.data)
        if not parsed:
            continue

        validity = check_validity(msg.arbitration_id, parsed)
        counter = get_counter(msg.arbitration_id, parsed)
        mid = msg.arbitration_id

        if mid == 0x0E0:
            speed_kmh = parsed['VehSpdLgtA']['value'] * 3.6
            valid = validity.get('VehSpdLgtA', False)
            print(f"[0x0E0] 车速: {speed_kmh:.1f} km/h, valid={valid}, cnt={counter}")

        elif mid == 0x080:
            gear_raw = int(parsed['TrsmActrPosn2TrsmActrPosn']['value'])
            gear_str = get_gear_str(gear_raw)
            valid = validity.get('TrsmActrPosn2TrsmActrPosn', False)
            print(f"[0x080] 档位: {gear_str}, valid={valid}, cnt={counter}")

        elif mid == 0x1B0:
            yaw = parsed['AgDataRawSafeYawRate']['value']
            roll = parsed['AgDataRawSafeRollRate']['value']
            valid = validity.get('AgDataRawSafeYawRate', False)
            print(f"[0x1B0] Yaw: {yaw:.4f} rad/s, Roll: {roll:.4f} rad/s, valid={valid}")

        elif mid == 0x0A0:
            a_lgt = parsed['ADataRawSafeALgt']['value']
            a_lat = parsed['ADataRawSafeALat']['value']
            a_vert = parsed['ADataRawSafeAVert']['value']
            print(f"[0x0A0] Acc: lgt={a_lgt:.3f}, lat={a_lat:.3f}, vert={a_vert:.3f} m/s^2")

        elif mid == 0x04E:
            steer_ag = parsed['PinionSteerAgGroupPinionSteerAg1']['value']
            steer_spd = parsed['PinionSteerAgGroupPinionSteerAgSpd1']['value']
            steer_tq = parsed['PinionSteerAgGroupSteerWhlTq']['value']
            print(f"[0x04E] 转向角: {steer_ag:.4f} rad, 角速度: {steer_spd:.4f} rad/s, 扭矩: {steer_tq:.3f} Nm")

except KeyboardInterrupt:
    print("退出监听")
finally:
    bus.shutdown()
```

---

## 5. 快速离线验证（无需 CAN 总线）

```python
from orin_can_parser import parse_message, check_validity

# 直接构造 8 字节数据，调用解析
data = bytes([0x07, 0xD0, 0x00, 0x00, 0x10, 0x00, 0x00, 0x30])
result = parse_message(0x0E0, data)

print(result['VehSpdLgtA']['value'], 'm/s')
print(check_validity(0x0E0, result))
```

---

## 6. 有效性判断规则

| 报文 | 信号 | 有效条件 |
|------|------|---------|
| 0x0E0 | VehSpdLgtA | `UB == 1 && QF == 3` |
| 0x080 | TrsmActrPosn2TrsmActrPosn | `UB == 1` |
| 0x1B0 | AgDataRawSafeYawRate / RollRate | `UB == 1 && QF == 3` |
| 0x0A0 | ADataRawSafeALgt / ALat / AVert | `UB == 1 && 各自QF == 3` |
| 0x04E | PinionSteerAg / Spd / Tq | `UB == 1 && 各自QF == 3` |

---

## 7. 档位枚举

| 值 | 含义 |
|----|------|
| 0 | P (Park) 驻车 |
| 1 | R (Reverse) 倒车 |
| 2 | N (Neutral) 空挡 |
| 3 | D (Drive) 前进 |
| 4 | S (Sport) 运动 |
| 5 | L (Low) 低速 |
| 6 | M (Manual) 手动 |
| 7 | Invalid 无效 |

---

## 8. 物理值公式速查

| 信号 | 公式 | 单位 |
|------|------|------|
| 车速 | `raw × 0.00391` | m/s |
| 横摆/侧倾角速度 | `raw × 0.000244140625` | rad/s |
| 加速度 (三轴) | `raw × 0.0085` | m/s² |
| 转向机角度 | `raw × 0.0009765625` | rad |
| 转向角速度 | `raw × 0.0078125` | rad/s |
| 方向盘扭矩 | `raw × 0.00390625` | Nm |
