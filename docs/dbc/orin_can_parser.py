#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
底盘CAN信号解析库 (Orin can0)
支持报文: 0x0E0(车速), 0x080(档位), 0x1B0(IMU), 0x0A0(加速度), 0x04E(转向)
字节序: Motorola (MSB)
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class SignalConfig:
    """信号配置"""
    name: str
    start_bit: int
    bit_length: int
    is_signed: bool
    factor: float
    offset: float
    unit: str
    min_val: float
    max_val: float


@dataclass
class MessageConfig:
    """报文配置"""
    msg_id: int
    name: str
    dlc: int
    signals: dict  # signal_name -> SignalConfig


# ──────────────────────────────────────────────
# 信号定义
# ──────────────────────────────────────────────

SIGNALS = {
    # 0x0E0 - VddmChas1Fr05 - 车速
    'VehSpdLgtA': SignalConfig('VehSpdLgtA', 38, 15, False, 0.00391, 0.0, 'm/s', 0.0, 125.0),
    'VehSpdLgtQf': SignalConfig('VehSpdLgtQf', 59, 2, False, 1, 0.0, '', 0, 3),
    'VehSpdLgt_UB': SignalConfig('VehSpdLgt_UB', 39, 1, False, 1, 0.0, '', 0, 1),
    'VehSpdLgtChks': SignalConfig('VehSpdLgtChks', 55, 8, False, 1, 0.0, '', 0, 255),
    'VehSpdLgtCntr': SignalConfig('VehSpdLgtCntr', 63, 4, False, 1, 0.0, '', 0, 15),

    # 0x080 - TcmChas1Fr08 - 档位
    'TrsmActrPosn2TrsmActrPosn': SignalConfig('TrsmActrPosn2TrsmActrPosn', 6, 3, False, 1, 0.0, '', 0, 7),
    'TrsmActrPosn2_UB': SignalConfig('TrsmActrPosn2_UB', 7, 1, False, 1, 0.0, '', 0, 1),
    'TrsmActrPosn2TrsmActrPosnChks': SignalConfig('TrsmActrPosn2TrsmActrPosnChks', 15, 8, False, 1, 0.0, '', 0, 255),
    'TrsmActrPosn2TrsmActrPosnCntr': SignalConfig('TrsmActrPosn2TrsmActrPosnCntr', 3, 4, False, 1, 0.0, '', 0, 15),

    # 0x1B0 - VddmChas1Fr14 - IMU (横摆/侧倾)
    'AgDataRawSafeYawRate': SignalConfig('AgDataRawSafeYawRate', 23, 16, True, 0.000244140625, 0.0, 'rad/s', -6.0, 6.0),
    'AgDataRawSafeYawRateQf': SignalConfig('AgDataRawSafeYawRateQf', 41, 2, False, 1, 0.0, '', 0, 3),
    'AgDataRawSafeRollRate': SignalConfig('AgDataRawSafeRollRate', 7, 16, True, 0.000244140625, 0.0, 'rad/s', -6.0, 6.0),
    'AgDataRawSafe_UB': SignalConfig('AgDataRawSafe_UB', 49, 1, False, 1, 0.0, '', 0, 1),
    'AgDataRawSafeChks': SignalConfig('AgDataRawSafeChks', 39, 8, False, 1, 0.0, '', 0, 255),
    'AgDataRawSafeCntr': SignalConfig('AgDataRawSafeCntr', 47, 4, False, 1, 0.0, '', 0, 15),

    # 0x0A0 - VddmChas1Fr03 - 加速度
    'ADataRawSafeALgt': SignalConfig('ADataRawSafeALgt', 38, 15, True, 0.0085, 0.0, 'm/s^2', -139.0, 139.0),
    'ADataRawSafeALgt1Qf': SignalConfig('ADataRawSafeALgt1Qf', 9, 2, False, 1, 0.0, '', 0, 3),
    'ADataRawSafeALat': SignalConfig('ADataRawSafeALat', 23, 15, True, 0.0085, 0.0, 'm/s^2', -139.0, 139.0),
    'ADataRawSafeALat1Qf': SignalConfig('ADataRawSafeALat1Qf', 11, 2, False, 1, 0.0, '', 0, 3),
    'ADataRawSafeAVert': SignalConfig('ADataRawSafeAVert', 55, 15, True, 0.0085, 0.0, 'm/s^2', -139.0, 139.0),
    'ADataRawSafeAVertQf': SignalConfig('ADataRawSafeAVertQf', 24, 2, False, 1, 0.0, '', 0, 3),
    'ADataRawSafe_UB': SignalConfig('ADataRawSafe_UB', 56, 1, False, 1, 0.0, '', 0, 1),
    'ADataRawSafeChks': SignalConfig('ADataRawSafeChks', 7, 8, False, 1, 0.0, '', 0, 255),
    'ADataRawSafeCntr': SignalConfig('ADataRawSafeCntr', 15, 4, False, 1, 0.0, '', 0, 15),

    # 0x04E - PscmChas1Fr07 - 转向
    'PinionSteerAgGroupPinionSteerAg1': SignalConfig('PinionSteerAgGroupPinionSteerAg1', 46, 15, True, 0.0009765625, 0.0, 'rad', -14.5, 14.5),
    'PinionSteerAgGroupPinionSteerAg1Qf': SignalConfig('PinionSteerAgGroupPinionSteerAg1Qf', 15, 2, False, 1, 0.0, '', 0, 3),
    'PinionSteerAgGroupPinionSteerAgSpd1': SignalConfig('PinionSteerAgGroupPinionSteerAgSpd1', 13, 14, True, 0.0078125, 0.0, 'rad/s', -50.0, 50.0),
    'PinionSteerAgGroupPinionSteerAgSpd1Qf': SignalConfig('PinionSteerAgGroupPinionSteerAgSpd1Qf', 31, 2, False, 1, 0.0, '', 0, 3),
    'PinionSteerAgGroupSteerWhlTq': SignalConfig('PinionSteerAgGroupSteerWhlTq', 29, 14, True, 0.00390625, 0.0, 'Nm', -30.0, 30.0),
    'PinionSteerAgGroupSteerWhlTqQf': SignalConfig('PinionSteerAgGroupSteerWhlTqQf', 59, 2, False, 1, 0.0, '', 0, 3),
    'PinionSteerAgGroup_UB': SignalConfig('PinionSteerAgGroup_UB', 57, 1, False, 1, 0.0, '', 0, 1),
    'PinionSteerAgGroupChks': SignalConfig('PinionSteerAgGroupChks', 7, 8, False, 1, 0.0, '', 0, 255),
    'PinionSteerAgGroupCntr': SignalConfig('PinionSteerAgGroupCntr', 63, 4, False, 1, 0.0, '', 0, 15),
}

# ──────────────────────────────────────────────
# 报文定义
# ──────────────────────────────────────────────

MESSAGES = {
    0x0E0: MessageConfig(0x0E0, 'VddmChas1Fr05', 8, {
        'VehSpdLgtA': SIGNALS['VehSpdLgtA'],
        'VehSpdLgtQf': SIGNALS['VehSpdLgtQf'],
        'VehSpdLgt_UB': SIGNALS['VehSpdLgt_UB'],
        'VehSpdLgtChks': SIGNALS['VehSpdLgtChks'],
        'VehSpdLgtCntr': SIGNALS['VehSpdLgtCntr'],
    }),
    0x080: MessageConfig(0x080, 'TcmChas1Fr08', 8, {
        'TrsmActrPosn2TrsmActrPosn': SIGNALS['TrsmActrPosn2TrsmActrPosn'],
        'TrsmActrPosn2_UB': SIGNALS['TrsmActrPosn2_UB'],
        'TrsmActrPosn2TrsmActrPosnChks': SIGNALS['TrsmActrPosn2TrsmActrPosnChks'],
        'TrsmActrPosn2TrsmActrPosnCntr': SIGNALS['TrsmActrPosn2TrsmActrPosnCntr'],
    }),
    0x1B0: MessageConfig(0x1B0, 'VddmChas1Fr14', 8, {
        'AgDataRawSafeYawRate': SIGNALS['AgDataRawSafeYawRate'],
        'AgDataRawSafeYawRateQf': SIGNALS['AgDataRawSafeYawRateQf'],
        'AgDataRawSafeRollRate': SIGNALS['AgDataRawSafeRollRate'],
        'AgDataRawSafe_UB': SIGNALS['AgDataRawSafe_UB'],
        'AgDataRawSafeChks': SIGNALS['AgDataRawSafeChks'],
        'AgDataRawSafeCntr': SIGNALS['AgDataRawSafeCntr'],
    }),
    0x0A0: MessageConfig(0x0A0, 'VddmChas1Fr03', 8, {
        'ADataRawSafeALgt': SIGNALS['ADataRawSafeALgt'],
        'ADataRawSafeALgt1Qf': SIGNALS['ADataRawSafeALgt1Qf'],
        'ADataRawSafeALat': SIGNALS['ADataRawSafeALat'],
        'ADataRawSafeALat1Qf': SIGNALS['ADataRawSafeALat1Qf'],
        'ADataRawSafeAVert': SIGNALS['ADataRawSafeAVert'],
        'ADataRawSafeAVertQf': SIGNALS['ADataRawSafeAVertQf'],
        'ADataRawSafe_UB': SIGNALS['ADataRawSafe_UB'],
        'ADataRawSafeChks': SIGNALS['ADataRawSafeChks'],
        'ADataRawSafeCntr': SIGNALS['ADataRawSafeCntr'],
    }),
    0x04E: MessageConfig(0x04E, 'PscmChas1Fr07', 8, {
        'PinionSteerAgGroupPinionSteerAg1': SIGNALS['PinionSteerAgGroupPinionSteerAg1'],
        'PinionSteerAgGroupPinionSteerAg1Qf': SIGNALS['PinionSteerAgGroupPinionSteerAg1Qf'],
        'PinionSteerAgGroupPinionSteerAgSpd1': SIGNALS['PinionSteerAgGroupPinionSteerAgSpd1'],
        'PinionSteerAgGroupPinionSteerAgSpd1Qf': SIGNALS['PinionSteerAgGroupPinionSteerAgSpd1Qf'],
        'PinionSteerAgGroupSteerWhlTq': SIGNALS['PinionSteerAgGroupSteerWhlTq'],
        'PinionSteerAgGroupSteerWhlTqQf': SIGNALS['PinionSteerAgGroupSteerWhlTqQf'],
        'PinionSteerAgGroup_UB': SIGNALS['PinionSteerAgGroup_UB'],
        'PinionSteerAgGroupChks': SIGNALS['PinionSteerAgGroupChks'],
        'PinionSteerAgGroupCntr': SIGNALS['PinionSteerAgGroupCntr'],
    }),
}

# 档位枚举
GEAR_POSITIONS = {
    0: 'P (Park)',
    1: 'R (Reverse)',
    2: 'N (Neutral)',
    3: 'D (Drive)',
    4: 'S (Sport)',
    5: 'L (Low)',
    6: 'M (Manual)',
    7: 'Invalid',
}


# ──────────────────────────────────────────────
# 解析函数
# ──────────────────────────────────────────────

def extract_motorola_signal(data: bytes, start_bit: int, bit_length: int, is_signed: bool) -> int:
    """
    从Motorola字节序(MSB)数据中提取信号原始值

    Args:
        data: 原始数据字节 (8 bytes)
        start_bit: 起始位 (MSB位置)
        bit_length: 位长度
        is_signed: 是否有符号

    Returns:
        提取的原始整数值
    """
    result = 0
    for i in range(bit_length):
        bit_pos = start_bit - i
        byte_idx = bit_pos // 8
        bit_idx = bit_pos % 8
        if byte_idx >= len(data):
            break
        if data[byte_idx] & (1 << bit_idx):
            result |= (1 << (bit_length - 1 - i))

    # 有符号数补码转换
    if is_signed and (result & (1 << (bit_length - 1))):
        result -= (1 << bit_length)
    return result


def parse_signal(data: bytes, signal: SignalConfig) -> float:
    """解析单个信号, 返回物理值 = raw * factor + offset"""
    raw_value = extract_motorola_signal(data, signal.start_bit, signal.bit_length, signal.is_signed)
    return raw_value * signal.factor + signal.offset


def parse_message(msg_id: int, data: bytes) -> dict:
    """
    解析CAN报文, 返回所有信号的物理值

    Args:
        msg_id: 报文ID (如 0x0E0)
        data: 数据字节 (bytes, 长度=DLC)

    Returns:
        dict: { signal_name: {'value': float, 'raw': int, 'unit': str} }
        若msg_id不在已知列表中, 返回空dict
    """
    if msg_id not in MESSAGES:
        return {}

    msg_config = MESSAGES[msg_id]
    result = {}
    for signal_name, sig in msg_config.signals.items():
        raw = extract_motorola_signal(data, sig.start_bit, sig.bit_length, sig.is_signed)
        result[signal_name] = {
            'value': raw * sig.factor + sig.offset,
            'raw': raw,
            'unit': sig.unit,
        }
    return result


def check_validity(msg_id: int, parsed: dict) -> dict:
    """
    对已解析的报文做有效性标注 (UB/QF检查)

    Args:
        msg_id: 报文ID
        parsed: parse_message() 的返回值

    Returns:
        dict: { signal_name: bool } 标注每个主要物理信号是否有效
    """
    validity = {}

    def _val(name):
        return parsed.get(name, {}).get('value', 0)

    if msg_id == 0x0E0:
        validity['VehSpdLgtA'] = (_val('VehSpdLgt_UB') == 1) and (_val('VehSpdLgtQf') == 3)

    elif msg_id == 0x080:
        validity['TrsmActrPosn2TrsmActrPosn'] = (_val('TrsmActrPosn2_UB') == 1)

    elif msg_id == 0x1B0:
        validity['AgDataRawSafeYawRate'] = (_val('AgDataRawSafe_UB') == 1) and (_val('AgDataRawSafeYawRateQf') == 3)
        validity['AgDataRawSafeRollRate'] = (_val('AgDataRawSafe_UB') == 1) and (_val('AgDataRawSafeYawRateQf') == 3)

    elif msg_id == 0x0A0:
        ub = _val('ADataRawSafe_UB') == 1
        validity['ADataRawSafeALgt'] = ub and (_val('ADataRawSafeALgt1Qf') == 3)
        validity['ADataRawSafeALat'] = ub and (_val('ADataRawSafeALat1Qf') == 3)
        validity['ADataRawSafeAVert'] = ub and (_val('ADataRawSafeAVertQf') == 3)

    elif msg_id == 0x04E:
        ub = _val('PinionSteerAgGroup_UB') == 1
        validity['PinionSteerAgGroupPinionSteerAg1'] = ub and (_val('PinionSteerAgGroupPinionSteerAg1Qf') == 3)
        validity['PinionSteerAgGroupPinionSteerAgSpd1'] = ub and (_val('PinionSteerAgGroupPinionSteerAgSpd1Qf') == 3)
        validity['PinionSteerAgGroupSteerWhlTq'] = ub and (_val('PinionSteerAgGroupSteerWhlTqQf') == 3)

    return validity


def get_gear_str(gear_value: int) -> str:
    """档位原始值 -> 字符串"""
    return GEAR_POSITIONS.get(int(gear_value), 'Unknown')


def get_counter(msg_id: int, parsed: dict) -> Optional[int]:
    """获取报文的活体计数器值"""
    counter_map = {
        0x0E0: 'VehSpdLgtCntr',
        0x080: 'TrsmActrPosn2TrsmActrPosnCntr',
        0x1B0: 'AgDataRawSafeCntr',
        0x0A0: 'ADataRawSafeCntr',
        0x04E: 'PinionSteerAgGroupCntr',
    }
    name = counter_map.get(msg_id)
    if name and name in parsed:
        return int(parsed[name]['value'])
    return None
