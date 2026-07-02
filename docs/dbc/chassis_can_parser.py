#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
底盘CAN信号解析脚本
支持报文: 0xE0 (车速), 0x80 (档位), 0x1B0 (YawRate)
字节序: Motorola (MSB)
"""

import struct
from dataclasses import dataclass
from typing import Optional, Tuple


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


# 信号定义
SIGNALS = {
    # 0xE0 - VddmChas1Fr05 - 车速
    'VehSpdLgtA': SignalConfig(
        name='VehSpdLgtA',
        start_bit=38,
        bit_length=15,
        is_signed=False,
        factor=0.00391,
        offset=0.0,
        unit='m/s',
        min_val=0.0,
        max_val=125.0027
    ),
    'VehSpdLgtQf': SignalConfig(
        name='VehSpdLgtQf',
        start_bit=59,
        bit_length=2,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=3
    ),
    'VehSpdLgt_UB': SignalConfig(
        name='VehSpdLgt_UB',
        start_bit=39,
        bit_length=1,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=1
    ),
    'VehSpdLgtChks': SignalConfig(
        name='VehSpdLgtChks',
        start_bit=55,
        bit_length=8,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=255
    ),
    'VehSpdLgtCntr': SignalConfig(
        name='VehSpdLgtCntr',
        start_bit=63,
        bit_length=4,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=15
    ),

    # 0x80 - TcmChas1Fr08 - 档位
    'TrsmActrPosn2TrsmActrPosn': SignalConfig(
        name='TrsmActrPosn2TrsmActrPosn',
        start_bit=6,
        bit_length=3,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=7
    ),
    'TrsmActrPosn2_UB': SignalConfig(
        name='TrsmActrPosn2_UB',
        start_bit=7,
        bit_length=1,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=1
    ),
    'TrsmActrPosn2TrsmActrPosnChks': SignalConfig(
        name='TrsmActrPosn2TrsmActrPosnChks',
        start_bit=15,
        bit_length=8,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=255
    ),
    'TrsmActrPosn2TrsmActrPosnCntr': SignalConfig(
        name='TrsmActrPosn2TrsmActrPosnCntr',
        start_bit=3,
        bit_length=4,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=15
    ),

    # 0x1B0 - VddmChas1Fr14 - YawRate
    'AgDataRawSafeYawRate': SignalConfig(
        name='AgDataRawSafeYawRate',
        start_bit=23,
        bit_length=16,
        is_signed=True,
        factor=0.000244140625,
        offset=0.0,
        unit='rad/s',
        min_val=-6.0,
        max_val=6.0
    ),
    'AgDataRawSafeYawRateQf': SignalConfig(
        name='AgDataRawSafeYawRateQf',
        start_bit=41,
        bit_length=2,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=3
    ),
    'AgDataRawSafeRollRate': SignalConfig(
        name='AgDataRawSafeRollRate',
        start_bit=7,
        bit_length=16,
        is_signed=True,
        factor=0.000244140625,
        offset=0.0,
        unit='rad/s',
        min_val=-6.0,
        max_val=6.0
    ),
    'AgDataRawSafe_UB': SignalConfig(
        name='AgDataRawSafe_UB',
        start_bit=49,
        bit_length=1,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=1
    ),
    'AgDataRawSafeChks': SignalConfig(
        name='AgDataRawSafeChks',
        start_bit=39,
        bit_length=8,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=255
    ),
    'AgDataRawSafeCntr': SignalConfig(
        name='AgDataRawSafeCntr',
        start_bit=47,
        bit_length=4,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=15
    ),

    # 0xA0 - VddmChas1Fr03 - 加速度
    'ADataRawSafeALgt': SignalConfig(
        name='ADataRawSafeALgt',
        start_bit=38,
        bit_length=15,
        is_signed=True,
        factor=0.0085,
        offset=0.0,
        unit='m/s^2',
        min_val=-138.992,
        max_val=139.0005
    ),
    'ADataRawSafeALgt1Qf': SignalConfig(
        name='ADataRawSafeALgt1Qf',
        start_bit=9,
        bit_length=2,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=3
    ),
    'ADataRawSafeALat': SignalConfig(
        name='ADataRawSafeALat',
        start_bit=23,
        bit_length=15,
        is_signed=True,
        factor=0.0085,
        offset=0.0,
        unit='m/s^2',
        min_val=-138.992,
        max_val=139.0005
    ),
    'ADataRawSafeALat1Qf': SignalConfig(
        name='ADataRawSafeALat1Qf',
        start_bit=11,
        bit_length=2,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=3
    ),
    'ADataRawSafeAVert': SignalConfig(
        name='ADataRawSafeAVert',
        start_bit=55,
        bit_length=15,
        is_signed=True,
        factor=0.0085,
        offset=0.0,
        unit='m/s^2',
        min_val=-138.992,
        max_val=139.0005
    ),
    'ADataRawSafeAVertQf': SignalConfig(
        name='ADataRawSafeAVertQf',
        start_bit=24,
        bit_length=2,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=3
    ),
    'ADataRawSafe_UB': SignalConfig(
        name='ADataRawSafe_UB',
        start_bit=56,
        bit_length=1,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=1
    ),
    'ADataRawSafeChks': SignalConfig(
        name='ADataRawSafeChks',
        start_bit=7,
        bit_length=8,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=255
    ),
    'ADataRawSafeCntr': SignalConfig(
        name='ADataRawSafeCntr',
        start_bit=15,
        bit_length=4,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=15
    ),

    # 0x4E - PscmChas1Fr07 - 转向角
    'PinionSteerAgGroupPinionSteerAg1': SignalConfig(
        name='PinionSteerAgGroupPinionSteerAg1',
        start_bit=46,
        bit_length=15,
        is_signed=True,
        factor=0.0009765625,
        offset=0.0,
        unit='rad',
        min_val=-14.5,
        max_val=14.5
    ),
    'PinionSteerAgGroupPinionSteerAg1Qf': SignalConfig(
        name='PinionSteerAgGroupPinionSteerAg1Qf',
        start_bit=15,
        bit_length=2,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=3
    ),
    'PinionSteerAgGroupPinionSteerAgSpd1': SignalConfig(
        name='PinionSteerAgGroupPinionSteerAgSpd1',
        start_bit=13,
        bit_length=14,
        is_signed=True,
        factor=0.0078125,
        offset=0.0,
        unit='rad/s',
        min_val=-50.0,
        max_val=50.0
    ),
    'PinionSteerAgGroupPinionSteerAgSpd1Qf': SignalConfig(
        name='PinionSteerAgGroupPinionSteerAgSpd1Qf',
        start_bit=31,
        bit_length=2,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=3
    ),
    'PinionSteerAgGroupSteerWhlTq': SignalConfig(
        name='PinionSteerAgGroupSteerWhlTq',
        start_bit=29,
        bit_length=14,
        is_signed=True,
        factor=0.00390625,
        offset=0.0,
        unit='Nm',
        min_val=-30.0,
        max_val=30.0
    ),
    'PinionSteerAgGroupSteerWhlTqQf': SignalConfig(
        name='PinionSteerAgGroupSteerWhlTqQf',
        start_bit=59,
        bit_length=2,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=3
    ),
    'PinionSteerAgGroup_UB': SignalConfig(
        name='PinionSteerAgGroup_UB',
        start_bit=57,
        bit_length=1,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=1
    ),
    'PinionSteerAgGroupChks': SignalConfig(
        name='PinionSteerAgGroupChks',
        start_bit=7,
        bit_length=8,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=255
    ),
    'PinionSteerAgGroupCntr': SignalConfig(
        name='PinionSteerAgGroupCntr',
        start_bit=63,
        bit_length=4,
        is_signed=False,
        factor=1,
        offset=0.0,
        unit='',
        min_val=0,
        max_val=15
    ),
}

# 报文定义
MESSAGES = {
    0xE0: MessageConfig(
        msg_id=0xE0,
        name='VddmChas1Fr05',
        dlc=8,
        signals={
            'VehSpdLgtA': SIGNALS['VehSpdLgtA'],
            'VehSpdLgtQf': SIGNALS['VehSpdLgtQf'],
            'VehSpdLgt_UB': SIGNALS['VehSpdLgt_UB'],
            'VehSpdLgtChks': SIGNALS['VehSpdLgtChks'],
            'VehSpdLgtCntr': SIGNALS['VehSpdLgtCntr'],
        }
    ),
    0x80: MessageConfig(
        msg_id=0x80,
        name='TcmChas1Fr08',
        dlc=8,
        signals={
            'TrsmActrPosn2TrsmActrPosn': SIGNALS['TrsmActrPosn2TrsmActrPosn'],
            'TrsmActrPosn2_UB': SIGNALS['TrsmActrPosn2_UB'],
            'TrsmActrPosn2TrsmActrPosnChks': SIGNALS['TrsmActrPosn2TrsmActrPosnChks'],
            'TrsmActrPosn2TrsmActrPosnCntr': SIGNALS['TrsmActrPosn2TrsmActrPosnCntr'],
        }
    ),
    0x1B0: MessageConfig(
        msg_id=0x1B0,
        name='VddmChas1Fr14',
        dlc=8,
        signals={
            'AgDataRawSafeYawRate': SIGNALS['AgDataRawSafeYawRate'],
            'AgDataRawSafeYawRateQf': SIGNALS['AgDataRawSafeYawRateQf'],
            'AgDataRawSafeRollRate': SIGNALS['AgDataRawSafeRollRate'],
            'AgDataRawSafe_UB': SIGNALS['AgDataRawSafe_UB'],
            'AgDataRawSafeChks': SIGNALS['AgDataRawSafeChks'],
            'AgDataRawSafeCntr': SIGNALS['AgDataRawSafeCntr'],
        }
    ),
    0xA0: MessageConfig(
        msg_id=0xA0,
        name='VddmChas1Fr03',
        dlc=8,
        signals={
            'ADataRawSafeALgt': SIGNALS['ADataRawSafeALgt'],
            'ADataRawSafeALgt1Qf': SIGNALS['ADataRawSafeALgt1Qf'],
            'ADataRawSafeALat': SIGNALS['ADataRawSafeALat'],
            'ADataRawSafeALat1Qf': SIGNALS['ADataRawSafeALat1Qf'],
            'ADataRawSafeAVert': SIGNALS['ADataRawSafeAVert'],
            'ADataRawSafeAVertQf': SIGNALS['ADataRawSafeAVertQf'],
            'ADataRawSafe_UB': SIGNALS['ADataRawSafe_UB'],
            'ADataRawSafeChks': SIGNALS['ADataRawSafeChks'],
            'ADataRawSafeCntr': SIGNALS['ADataRawSafeCntr'],
        }
    ),
    0x4E: MessageConfig(
        msg_id=0x4E,
        name='PscmChas1Fr07',
        dlc=8,
        signals={
            'PinionSteerAgGroupPinionSteerAg1': SIGNALS['PinionSteerAgGroupPinionSteerAg1'],
            'PinionSteerAgGroupPinionSteerAg1Qf': SIGNALS['PinionSteerAgGroupPinionSteerAg1Qf'],
            'PinionSteerAgGroupPinionSteerAgSpd1': SIGNALS['PinionSteerAgGroupPinionSteerAgSpd1'],
            'PinionSteerAgGroupPinionSteerAgSpd1Qf': SIGNALS['PinionSteerAgGroupPinionSteerAgSpd1Qf'],
            'PinionSteerAgGroupSteerWhlTq': SIGNALS['PinionSteerAgGroupSteerWhlTq'],
            'PinionSteerAgGroupSteerWhlTqQf': SIGNALS['PinionSteerAgGroupSteerWhlTqQf'],
            'PinionSteerAgGroup_UB': SIGNALS['PinionSteerAgGroup_UB'],
            'PinionSteerAgGroupChks': SIGNALS['PinionSteerAgGroupChks'],
            'PinionSteerAgGroupCntr': SIGNALS['PinionSteerAgGroupCntr'],
        }
    ),
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


def extract_motorola_signal(data: bytes, start_bit: int, bit_length: int, is_signed: bool) -> int:
    """
    从Motorola字节序数据中提取信号

    Args:
        data: 原始数据字节
        start_bit: 起始位(MSB位置)
        bit_length: 位长度
        is_signed: 是否有符号

    Returns:
        提取的原始值
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

    # 处理有符号数
    if is_signed and (result & (1 << (bit_length - 1))):
        result -= (1 << bit_length)

    return result


def parse_signal(data: bytes, signal: SignalConfig) -> float:
    """
    解析信号值

    Args:
        data: 原始数据字节
        signal: 信号配置

    Returns:
        物理值
    """
    raw_value = extract_motorola_signal(data, signal.start_bit, signal.bit_length, signal.is_signed)
    physical_value = raw_value * signal.factor + signal.offset
    return physical_value


def parse_message(msg_id: int, data: bytes) -> dict:
    """
    解析CAN报文

    Args:
        msg_id: 报文ID
        data: 数据字节

    Returns:
        解析后的信号值字典
    """
    if msg_id not in MESSAGES:
        return {}

    msg_config = MESSAGES[msg_id]
    result = {}

    for signal_name, signal_config in msg_config.signals.items():
        value = parse_signal(data, signal_config)
        result[signal_name] = {
            'value': value,
            'unit': signal_config.unit,
            'raw': extract_motorola_signal(data, signal_config.start_bit,
                                          signal_config.bit_length, signal_config.is_signed)
        }

    return result


def format_vehicle_speed(speed_ms: float) -> Tuple[float, str]:
    """格式化车速"""
    speed_kmh = speed_ms * 3.6
    return speed_kmh, 'km/h'


def format_gear_position(gear_value: int) -> str:
    """格式化档位"""
    return GEAR_POSITIONS.get(int(gear_value), 'Unknown')


def format_yaw_rate(yaw_rate: float) -> Tuple[float, str]:
    """格式化横摆角速度"""
    yaw_rate_degs = yaw_rate * 57.2957795  # rad to deg/s
    return yaw_rate_degs, 'deg/s'


class ChassisCANParser:
    """底盘CAN解析器"""

    def __init__(self):
        self.last_counter = {}
        self.e2e_errors = {}

    def parse(self, msg_id: int, data: bytes) -> dict:
        """解析报文"""
        result = parse_message(msg_id, data)

        # 添加格式化值
        if msg_id == 0xE0 and 'VehSpdLgtA' in result:
            speed_ms = result['VehSpdLgtA']['value']
            speed_kmh, unit = format_vehicle_speed(speed_ms)
            result['VehSpdLgtA']['kmh'] = speed_kmh
            result['VehSpdLgtA']['kmh_unit'] = unit

            # 有效性检查
            ub = result.get('VehSpdLgt_UB', {}).get('value', 0)
            qf = result.get('VehSpdLgtQf', {}).get('value', 0)
            result['VehSpdLgtA']['valid'] = (ub == 1) and (qf == 3)

        elif msg_id == 0x80 and 'TrsmActrPosn2TrsmActrPosn' in result:
            gear_val = int(result['TrsmActrPosn2TrsmActrPosn']['value'])
            result['TrsmActrPosn2TrsmActrPosn']['gear_str'] = format_gear_position(gear_val)

            # 有效性检查
            ub = result.get('TrsmActrPosn2_UB', {}).get('value', 0)
            result['TrsmActrPosn2TrsmActrPosn']['valid'] = (ub == 1)

        elif msg_id == 0x1B0 and 'AgDataRawSafeYawRate' in result:
            yaw_rate = result['AgDataRawSafeYawRate']['value']
            yaw_rate_degs, unit = format_yaw_rate(yaw_rate)
            result['AgDataRawSafeYawRate']['deg_s'] = yaw_rate_degs
            result['AgDataRawSafeYawRate']['deg_s_unit'] = unit

            # 有效性检查
            ub = result.get('AgDataRawSafe_UB', {}).get('value', 0)
            qf = result.get('AgDataRawSafeYawRateQf', {}).get('value', 0)
            result['AgDataRawSafeYawRate']['valid'] = (ub == 1) and (qf == 3)

        elif msg_id == 0xA0:
            # 纵向加速度
            if 'ADataRawSafeALgt' in result:
                a_lgt = result['ADataRawSafeALgt']['value']
                result['ADataRawSafeALgt']['g'] = a_lgt / 9.81
                ub = result.get('ADataRawSafe_UB', {}).get('value', 0)
                qf = result.get('ADataRawSafeALgt1Qf', {}).get('value', 0)
                result['ADataRawSafeALgt']['valid'] = (ub == 1) and (qf == 3)

            # 横向加速度
            if 'ADataRawSafeALat' in result:
                a_lat = result['ADataRawSafeALat']['value']
                result['ADataRawSafeALat']['g'] = a_lat / 9.81
                ub = result.get('ADataRawSafe_UB', {}).get('value', 0)
                qf = result.get('ADataRawSafeALat1Qf', {}).get('value', 0)
                result['ADataRawSafeALat']['valid'] = (ub == 1) and (qf == 3)

            # 垂向加速度
            if 'ADataRawSafeAVert' in result:
                a_vert = result['ADataRawSafeAVert']['value']
                result['ADataRawSafeAVert']['g'] = a_vert / 9.81
                ub = result.get('ADataRawSafe_UB', {}).get('value', 0)
                qf = result.get('ADataRawSafeAVertQf', {}).get('value', 0)
                result['ADataRawSafeAVert']['valid'] = (ub == 1) and (qf == 3)

        elif msg_id == 0x4E:
            # 转向机角度
            if 'PinionSteerAgGroupPinionSteerAg1' in result:
                steer_ag = result['PinionSteerAgGroupPinionSteerAg1']['value']
                result['PinionSteerAgGroupPinionSteerAg1']['deg'] = steer_ag * 57.2957795
                ub = result.get('PinionSteerAgGroup_UB', {}).get('value', 0)
                qf = result.get('PinionSteerAgGroupPinionSteerAg1Qf', {}).get('value', 0)
                result['PinionSteerAgGroupPinionSteerAg1']['valid'] = (ub == 1) and (qf == 3)

            # 转向角速度
            if 'PinionSteerAgGroupPinionSteerAgSpd1' in result:
                steer_spd = result['PinionSteerAgGroupPinionSteerAgSpd1']['value']
                result['PinionSteerAgGroupPinionSteerAgSpd1']['deg_s'] = steer_spd * 57.2957795
                qf = result.get('PinionSteerAgGroupPinionSteerAgSpd1Qf', {}).get('value', 0)
                result['PinionSteerAgGroupPinionSteerAgSpd1']['valid'] = (qf == 3)

            # 方向盘扭矩
            if 'PinionSteerAgGroupSteerWhlTq' in result:
                steer_tq = result['PinionSteerAgGroupSteerWhlTq']['value']
                qf = result.get('PinionSteerAgGroupSteerWhlTqQf', {}).get('value', 0)
                result['PinionSteerAgGroupSteerWhlTq']['valid'] = (qf == 3)

        return result

    def print_parsed_data(self, msg_id: int, data: bytes):
        """打印解析结果"""
        result = self.parse(msg_id, data)

        if msg_id == 0xE0:
            speed = result.get('VehSpdLgtA', {})
            print(f"\n[0xE0] 车速 (VddmChas1Fr05)")
            print(f"  车速: {speed.get('value', 0):.3f} m/s ({speed.get('kmh', 0):.1f} km/h)")
            print(f"  有效性: {'有效' if speed.get('valid') else '无效'}")
            print(f"  QF: {result.get('VehSpdLgtQf', {}).get('value', 0)}")
            print(f"  Counter: {result.get('VehSpdLgtCntr', {}).get('value', 0)}")

        elif msg_id == 0x80:
            gear = result.get('TrsmActrPosn2TrsmActrPosn', {})
            print(f"\n[0x80] 档位 (TcmChas1Fr08)")
            print(f"  档位: {gear.get('gear_str', 'Unknown')}")
            print(f"  原始值: {int(gear.get('value', 0))}")
            print(f"  有效性: {'有效' if gear.get('valid') else '无效'}")
            print(f"  Counter: {result.get('TrsmActrPosn2TrsmActrPosnCntr', {}).get('value', 0)}")

        elif msg_id == 0x1B0:
            yaw = result.get('AgDataRawSafeYawRate', {})
            roll = result.get('AgDataRawSafeRollRate', {})
            print(f"\n[0x1B0] IMU数据 (VddmChas1Fr14)")
            print(f"  Yaw Rate: {yaw.get('value', 0):.6f} rad/s ({yaw.get('deg_s', 0):.2f} deg/s)")
            print(f"  Roll Rate: {roll.get('value', 0):.6f} rad/s")
            print(f"  有效性: {'有效' if yaw.get('valid') else '无效'}")
            print(f"  QF: {result.get('AgDataRawSafeYawRateQf', {}).get('value', 0)}")
            print(f"  Counter: {result.get('AgDataRawSafeCntr', {}).get('value', 0)}")

        elif msg_id == 0xA0:
            a_lgt = result.get('ADataRawSafeALgt', {})
            a_lat = result.get('ADataRawSafeALat', {})
            a_vert = result.get('ADataRawSafeAVert', {})
            print(f"\n[0xA0] 加速度 (VddmChas1Fr03)")
            print(f"  纵向加速度: {a_lgt.get('value', 0):.4f} m/s^2 ({a_lgt.get('g', 0):.4f} g)")
            print(f"    有效性: {'有效' if a_lgt.get('valid') else '无效'}")
            print(f"    QF: {result.get('ADataRawSafeALgt1Qf', {}).get('value', 0)}")
            print(f"  横向加速度: {a_lat.get('value', 0):.4f} m/s^2 ({a_lat.get('g', 0):.4f} g)")
            print(f"    有效性: {'有效' if a_lat.get('valid') else '无效'}")
            print(f"    QF: {result.get('ADataRawSafeALat1Qf', {}).get('value', 0)}")
            print(f"  垂向加速度: {a_vert.get('value', 0):.4f} m/s^2 ({a_vert.get('g', 0):.4f} g)")
            print(f"    有效性: {'有效' if a_vert.get('valid') else '无效'}")
            print(f"    QF: {result.get('ADataRawSafeAVertQf', {}).get('value', 0)}")
            print(f"  UB: {result.get('ADataRawSafe_UB', {}).get('value', 0)}")
            print(f"  Counter: {result.get('ADataRawSafeCntr', {}).get('value', 0)}")

        elif msg_id == 0x4E:
            steer_ag = result.get('PinionSteerAgGroupPinionSteerAg1', {})
            steer_spd = result.get('PinionSteerAgGroupPinionSteerAgSpd1', {})
            steer_tq = result.get('PinionSteerAgGroupSteerWhlTq', {})
            print(f"\n[0x4E] 转向数据 (PscmChas1Fr07)")
            print(f"  转向机角度: {steer_ag.get('value', 0):.6f} rad ({steer_ag.get('deg', 0):.2f} deg)")
            print(f"    有效性: {'有效' if steer_ag.get('valid') else '无效'}")
            print(f"    QF: {result.get('PinionSteerAgGroupPinionSteerAg1Qf', {}).get('value', 0)}")
            print(f"  转向角速度: {steer_spd.get('value', 0):.6f} rad/s ({steer_spd.get('deg_s', 0):.2f} deg/s)")
            print(f"    有效性: {'有效' if steer_spd.get('valid') else '无效'}")
            print(f"    QF: {result.get('PinionSteerAgGroupPinionSteerAgSpd1Qf', {}).get('value', 0)}")
            print(f"  方向盘扭矩: {steer_tq.get('value', 0):.4f} Nm")
            print(f"    有效性: {'有效' if steer_tq.get('valid') else '无效'}")
            print(f"    QF: {result.get('PinionSteerAgGroupSteerWhlTqQf', {}).get('value', 0)}")
            print(f"  UB: {result.get('PinionSteerAgGroup_UB', {}).get('value', 0)}")
            print(f"  Counter: {result.get('PinionSteerAgGroupCntr', {}).get('value', 0)}")


def test_parser():
    """测试解析器"""
    parser = ChassisCANParser()

    print("=" * 60)
    print("底盘CAN信号解析器测试")
    print("=" * 60)

    # 测试0xE0 - 车速
    # 构造测试数据: VehSpdLgtA = 1000 (raw) -> 3.91 m/s
    # start_bit=38, bit_length=15, Motorola
    data_0xe0 = bytearray(8)
    # 1000 = 0x3E8, 15位: 000001111101000
    # bit38 -> byte4 bit6, bit24 -> byte3 bit0
    data_0xe0[4] = 0x07  # bits 38-32
    data_0xe0[3] = 0xD0  # bits 31-24
    data_0xe0[4] |= 0x10  # UB=1 (bit39)
    data_0xe0[7] = 0x30  # QF=3 (bits 59-58)
    parser.print_parsed_data(0xE0, bytes(data_0xe0))

    # 测试0x80 - 档位
    # 构造测试数据: TrsmActrPosn2TrsmActrPosn = 3 (D档)
    data_0x80 = bytearray(8)
    data_0x80[0] = 0x73  # bits 7-0: UB=0, gear=3(011), counter=3
    data_0x80[0] |= 0x80  # UB=1 (bit7)
    parser.print_parsed_data(0x80, bytes(data_0x80))

    # 测试0x1B0 - YawRate
    # 构造测试数据: AgDataRawSafeYawRate = 1000 (raw) -> 0.244 rad/s
    data_0x1b0 = bytearray(8)
    # 1000 = 0x03E8, 16位有符号
    data_0x1b0[2] = 0x03  # bits 23-16
    data_0x1b0[1] = 0xE8  # bits 15-8
    data_0x1b0[6] = 0x20  # UB=1 (bit49)
    data_0x1b0[5] = 0x03  # QF=3 (bits 41-40)
    parser.print_parsed_data(0x1B0, bytes(data_0x1b0))

    # 测试0xA0 - 加速度
    # 构造测试数据: ADataRawSafeALgt = 500 (raw) -> 4.25 m/s²
    # ADataRawSafeALat = -300 (raw) -> -2.55 m/s²
    data_0xa0 = bytearray(8)
    # ADataRawSafeALgt: start_bit=38, bit_length=15, signed
    # 500 = 0x01F4, 15位: 000 0001 1111 0100
    # bit38 -> byte4 bit6, bit24 -> byte3 bit0
    data_0xa0[4] = 0x01  # bits 38-32: 00000001
    data_0xa0[3] = 0xF4  # bits 31-24: 11110100
    # ADataRawSafeALat: start_bit=23, bit_length=15, signed
    # -300 补码 = 0x1ED4 (15位), 实际: 1 1110 1101 0100 -> 需要仔细计算
    # -300的15位补码: 2^15 - 300 = 32768 - 300 = 32468 = 0x7ED4
    # 但15位有符号, -300 = 0x7ED4 (取15位)
    # bit23 -> byte2 bit7, bit9 -> byte1 bit1
    data_0xa0[2] = 0xFE  # bits 23-16: 11111110
    data_0xa0[1] = 0xD4  # bits 15-8: 11010100
    # ADataRawSafe_UB = 1 (bit56)
    data_0xa0[7] = 0x01  # bit56
    # ADataRawSafeALgt1Qf = 3 (bits 9-8)
    data_0xa0[1] |= 0x03  # bits 9-8: 11
    # ADataRawSafeALat1Qf = 3 (bits 11-10)
    data_0xa0[1] |= 0x0C  # bits 11-10: 11
    parser.print_parsed_data(0xA0, bytes(data_0xa0))

    # 测试0x4E - 转向角
    # 构造测试数据: PinionSteerAgGroupPinionSteerAg1 = 1000 (raw) -> 0.977 rad
    data_0x4e = bytearray(8)
    # PinionSteerAgGroupPinionSteerAg1: start_bit=46, bit_length=15, signed
    # 1000 = 0x03E8, 15位: 000 0011 1110 1000
    # bit46 -> byte5 bit6, bit32 -> byte4 bit0
    data_0x4e[5] = 0x07  # bits 46-40: 00000111
    data_0x4e[4] = 0xD0  # bits 39-32: 11010000
    # PinionSteerAgGroup_UB = 1 (bit57)
    data_0x4e[7] = 0x02  # bit57
    # PinionSteerAgGroupPinionSteerAg1Qf = 3 (bits 15-14)
    data_0x4e[1] = 0x03  # bits 15-14: 11
    parser.print_parsed_data(0x4E, bytes(data_0x4e))

    print("\n" + "=" * 60)


if __name__ == '__main__':
    test_parser()
