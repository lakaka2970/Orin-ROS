"""
数据输入输出模块：读取二进制雷达数据文件
"""
import numpy as np


def read_synthetic_bin(
    filepath: str,
    num_rx: int,
    samples_per_chirp: int,
    num_chirp: int,
    is_complex: bool = False
) -> np.ndarray:
    """
    读取由 data_generatev3.py 生成的二进制文件

    Args:
        filepath: 文件路径
        num_rx: 接收天线数
        samples_per_chirp: 每个Chirp采样点数
        num_chirp: Chirp数量
        is_complex: 是否为IQ交替存储的复数格式（False表示实部+虚部分开存储？按原代码逻辑：偶数索引为虚部，奇数索引为实部）

    Returns:
        raw_data: 形状 (num_chirp, num_rx, samples_per_chirp)，数据类型 complex64（若is_complex）或 float32
    """
    data_int16 = np.fromfile(filepath, dtype=np.int16)

    if is_complex:
        if len(data_int16) % 2 != 0:
            raise ValueError("数据长度不是偶数，无法解析为复数")
        real_part = data_int16[1::2]      # 原代码：实部在奇数索引
        imag_part = data_int16[0::2]      # 虚部在偶数索引
        data = (real_part + 1j * imag_part).astype(np.complex64)
    else:
        data = data_int16.astype(np.float32)

    expected_len = samples_per_chirp * num_chirp * num_rx
    if len(data) != expected_len:
        raise ValueError(f"数据长度不匹配：期望 {expected_len}，实际 {len(data)}")

    # 重塑为 (samples_per_chirp, num_chirp, num_rx) 并转置为 (num_chirp, num_rx, samples_per_chirp)
    reshaped = data.reshape(samples_per_chirp, num_chirp, num_rx)
    raw = np.transpose(reshaped, (1, 2, 0))
    return raw