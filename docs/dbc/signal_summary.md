# 底盘CAN信号汇总表

> 字节序: Motorola (MSB) | 来源DBC: SDB22404_KX11_A2_ADCU2_VCU2_PCM_ChassisCAN1Cfg_230423.dbc

---

## 报文总览

| 报文ID | 报文名称 | 发送节点 | DLC | 周期 | 主要物理信号 |
|--------|----------|----------|-----|------|-------------|
| 0x0E0 | VddmChas1Fr05 | BBM | 8 | 20ms | 纵向车速 |
| 0x080 | TcmChas1Fr08 | TCM | 8 | 30ms | 档位 |
| 0x1B0 | VddmChas1Fr14 | VDDM | 8 | 30ms | 横摆角速度、侧倾角速度 |
| 0x0A0 | VddmChas1Fr03 | VDDM | 8 | 15ms | 纵向/横向/垂向加速度 |
| 0x04E | PscmChas1Fr07 | PSCM1 | 8 | 10ms | 转向角/转向角速度/方向盘扭矩 |

---

## 0x0E0 — VddmChas1Fr05（车速）

| 信号名称 | 起始位 | 位长 | 类型 | Factor | Offset | 单位 | 最小值 | 最大值 | 说明 |
|----------|--------|------|------|--------|--------|------|--------|--------|------|
| VehSpdLgtA | 38 | 15 | unsigned | 0.00391 | 0 | m/s | 0 | 125.0 | 纵向车速 |
| VehSpdLgtQf | 59 | 2 | unsigned | 1 | 0 | — | 0 | 3 | 质量因子 |
| VehSpdLgt_UB | 39 | 1 | unsigned | 1 | 0 | — | 0 | 1 | 有效标志 |
| VehSpdLgtChks | 55 | 8 | unsigned | 1 | 0 | — | 0 | 255 | CRC校验 |
| VehSpdLgtCntr | 63 | 4 | unsigned | 1 | 0 | — | 0 | 15 | 活体计数器 |

**物理值公式**: `speed = raw × 0.00391`  
**有效性判断**: `UB == 1 && QF == 3`

---

## 0x080 — TcmChas1Fr08（档位）

| 信号名称 | 起始位 | 位长 | 类型 | Factor | Offset | 单位 | 最小值 | 最大值 | 说明 |
|----------|--------|------|------|--------|--------|------|--------|--------|------|
| TrsmActrPosn2TrsmActrPosn | 6 | 3 | unsigned | 1 | 0 | — | 0 | 7 | 档位位置 |
| TrsmActrPosn2_UB | 7 | 1 | unsigned | 1 | 0 | — | 0 | 1 | 有效标志 |
| TrsmActrPosn2TrsmActrPosnChks | 15 | 8 | unsigned | 1 | 0 | — | 0 | 255 | CRC校验 |
| TrsmActrPosn2TrsmActrPosnCntr | 3 | 4 | unsigned | 1 | 0 | — | 0 | 15 | 活体计数器 |

**物理值公式**: `gear = raw`  
**有效性判断**: `UB == 1`

**档位枚举**:

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

## 0x1B0 — VddmChas1Fr14（IMU数据）

| 信号名称 | 起始位 | 位长 | 类型 | Factor | Offset | 单位 | 最小值 | 最大值 | 说明 |
|----------|--------|------|------|--------|--------|------|--------|--------|------|
| AgDataRawSafeYawRate | 23 | 16 | signed | 0.000244140625 | 0 | rad/s | -6.0 | 6.0 | 横摆角速度 |
| AgDataRawSafeYawRateQf | 41 | 2 | unsigned | 1 | 0 | — | 0 | 3 | 横摆QF |
| AgDataRawSafeRollRate | 7 | 16 | signed | 0.000244140625 | 0 | rad/s | -6.0 | 6.0 | 侧倾角速度 |
| AgDataRawSafe_UB | 49 | 1 | unsigned | 1 | 0 | — | 0 | 1 | 有效标志 |
| AgDataRawSafeChks | 39 | 8 | unsigned | 1 | 0 | — | 0 | 255 | CRC校验 |
| AgDataRawSafeCntr | 47 | 4 | unsigned | 1 | 0 | — | 0 | 15 | 活体计数器 |

**物理值公式**: `yaw_rate = raw × 0.000244140625`  
**有效性判断**: `UB == 1 && QF == 3`

---

## 0x0A0 — VddmChas1Fr03（加速度）

| 信号名称 | 起始位 | 位长 | 类型 | Factor | Offset | 单位 | 最小值 | 最大值 | 说明 |
|----------|--------|------|------|--------|--------|------|--------|--------|------|
| ADataRawSafeALgt | 38 | 15 | signed | 0.0085 | 0 | m/s^2 | -139.0 | 139.0 | 纵向加速度 |
| ADataRawSafeALgt1Qf | 9 | 2 | unsigned | 1 | 0 | — | 0 | 3 | 纵向QF |
| ADataRawSafeALat | 23 | 15 | signed | 0.0085 | 0 | m/s^2 | -139.0 | 139.0 | 横向加速度 |
| ADataRawSafeALat1Qf | 11 | 2 | unsigned | 1 | 0 | — | 0 | 3 | 横向QF |
| ADataRawSafeAVert | 55 | 15 | signed | 0.0085 | 0 | m/s^2 | -139.0 | 139.0 | 垂向加速度 |
| ADataRawSafeAVertQf | 24 | 2 | unsigned | 1 | 0 | — | 0 | 3 | 垂向QF |
| ADataRawSafe_UB | 56 | 1 | unsigned | 1 | 0 | — | 0 | 1 | 有效标志 |
| ADataRawSafeChks | 7 | 8 | unsigned | 1 | 0 | — | 0 | 255 | CRC校验 |
| ADataRawSafeCntr | 15 | 4 | unsigned | 1 | 0 | — | 0 | 15 | 活体计数器 |

**物理值公式**: `acc = raw × 0.0085`  
**有效性判断**: `UB == 1 && 对应QF == 3`（每个加速度信号独立判断）

---

## 0x04E — PscmChas1Fr07（转向数据）

| 信号名称 | 起始位 | 位长 | 类型 | Factor | Offset | 单位 | 最小值 | 最大值 | 说明 |
|----------|--------|------|------|--------|--------|------|--------|--------|------|
| PinionSteerAgGroupPinionSteerAg1 | 46 | 15 | signed | 0.0009765625 | 0 | rad | -14.5 | 14.5 | 转向机角度 |
| PinionSteerAgGroupPinionSteerAg1Qf | 15 | 2 | unsigned | 1 | 0 | — | 0 | 3 | 角度QF |
| PinionSteerAgGroupPinionSteerAgSpd1 | 13 | 14 | signed | 0.0078125 | 0 | rad/s | -50.0 | 50.0 | 转向角速度 |
| PinionSteerAgGroupPinionSteerAgSpd1Qf | 31 | 2 | unsigned | 1 | 0 | — | 0 | 3 | 角速度QF |
| PinionSteerAgGroupSteerWhlTq | 29 | 14 | signed | 0.00390625 | 0 | Nm | -30.0 | 30.0 | 方向盘扭矩 |
| PinionSteerAgGroupSteerWhlTqQf | 59 | 2 | unsigned | 1 | 0 | — | 0 | 3 | 扭矩QF |
| PinionSteerAgGroup_UB | 57 | 1 | unsigned | 1 | 0 | — | 0 | 1 | 有效标志 |
| PinionSteerAgGroupChks | 7 | 8 | unsigned | 1 | 0 | — | 0 | 255 | CRC校验 |
| PinionSteerAgGroupCntr | 63 | 4 | unsigned | 1 | 0 | — | 0 | 15 | 活体计数器 |

**物理值公式**:
- 转向角: `angle = raw × 0.0009765625`
- 角速度: `speed = raw × 0.0078125`
- 扭矩: `torque = raw × 0.00390625`

**有效性判断**: `UB == 1 && 对应QF == 3`（每个信号独立判断）

---

## 统计汇总

| 类别 | 数量 |
|------|------|
| 报文总数 | 5 |
| 信号总数 | 33 |
| 主要物理信号 | 10 |
| E2E保护信号组 | 5 (均含CRC+Counter) |

### 主要物理信号一览

| 序号 | 信号名称 | 报文ID | 报文名称 | 物理含义 | 单位 |
|------|----------|--------|----------|----------|------|
| 1 | VehSpdLgtA | 0x0E0 | VddmChas1Fr05 | 纵向车速 | m/s |
| 2 | TrsmActrPosn2TrsmActrPosn | 0x080 | TcmChas1Fr08 | 档位位置 | — |
| 3 | AgDataRawSafeYawRate | 0x1B0 | VddmChas1Fr14 | 横摆角速度 | rad/s |
| 4 | AgDataRawSafeRollRate | 0x1B0 | VddmChas1Fr14 | 侧倾角速度 | rad/s |
| 5 | ADataRawSafeALgt | 0x0A0 | VddmChas1Fr03 | 纵向加速度 | m/s^2 |
| 6 | ADataRawSafeALat | 0x0A0 | VddmChas1Fr03 | 横向加速度 | m/s^2 |
| 7 | ADataRawSafeAVert | 0x0A0 | VddmChas1Fr03 | 垂向加速度 | m/s^2 |
| 8 | PinionSteerAgGroupPinionSteerAg1 | 0x04E | PscmChas1Fr07 | 转向机角度 | rad |
| 9 | PinionSteerAgGroupPinionSteerAgSpd1 | 0x04E | PscmChas1Fr07 | 转向角速度 | rad/s |
| 10 | PinionSteerAgGroupSteerWhlTq | 0x04E | PscmChas1Fr07 | 方向盘扭矩 | Nm |
