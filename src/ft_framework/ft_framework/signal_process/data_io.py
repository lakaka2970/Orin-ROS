"""
数据输入输出模块：读取级联CTRX原始二进制雷达数据文件
参考MATLAB解析逻辑：
  - 每片CTRX: 4 RF通道, I/Q交替, reshape(4, Ns, 2, Nc) Fortran序
  - I分量 → ch1~4, Q分量 → ch5~8, 两片CTRX → 16通道
  - 最终 permute → (Nc, Ns, 16) → transpose → (Nc, 16, Ns)
"""
import os
import numpy as np


def readRawBinCasc(
    datadir: str,
    frameNr: int,
    nSamples: int,
    nRamps: int,
    nChannels: int = 16
) -> np.ndarray:
    """
    读取级联CTRX原始二进制数据 (int16, I/Q通道交替)。

    每片CTRX固定4个RF通道, I分量与Q分量各占一半,
    两片CTRX拼出16个虚拟通道。

    Args:
        datadir:   存放 ctrx*_raw.bin 文件的目录路径
        frameNr:   要读取的帧序号（从0开始）
        nSamples:  每个Chirp的采样点数 (Ns)
        nRamps:    一帧内的Chirp数 (Nc)
        nChannels: 总通道数（默认16，须为8的整数倍）

    Returns:
        Raw: 形状 (nRamps, nChannels, nSamples)，数据类型 float64
    """
    if nChannels % 8 != 0:
        raise ValueError(f"nChannels 必须为8的整数倍，当前值: {nChannels}")

    Ns = nSamples
    Nc = nRamps
    numCtrx = nChannels // 8                       # 每片CTRX贡献8个虚拟通道

    # 每片CTRX一帧的 int16 数量 = 4ch × Ns × 2(I/Q) × Nc
    frameBinsPerCtrx = 4 * Ns * 2 * Nc
    frameBytesPerCtrx = frameBinsPerCtrx * 2       # int16 = 2 bytes

    # --- 读第一片 CTRX0 ---
    fname = os.path.join(datadir, "ctrx0_raw.bin")
    fstat = os.stat(fname)
    numFrames = fstat.st_size // frameBytesPerCtrx
    if frameNr >= numFrames:
        raise IndexError(f"frameNr={frameNr} 超出文件总帧数 {numFrames}")

    with open(fname, "rb") as f:
        f.seek(frameBytesPerCtrx * frameNr)
        raw_bytes = f.read(frameBytesPerCtrx)
    data0 = np.frombuffer(raw_bytes, dtype=np.int16)

    # MATLAB: reshape(data0, [4, Ns, 2, Nc])  列优先 (Fortran order)
    data0_tmp = data0.reshape((4, Ns, 2, Nc), order='F')     # (4, Ns, 2, Nc)
    I0 = data0_tmp[:, :, 0, :].copy()                         # (4, Ns, Nc) — I分量 → ch0~3
    Q0 = data0_tmp[:, :, 1, :].copy()                         # (4, Ns, Nc) — Q分量 → ch4~7
    Raw = np.concatenate([I0, Q0], axis=0)                    # (8, Ns, Nc)

    # --- 读后续 CTRX1~N ---
    for fid in range(1, numCtrx):
        fname = os.path.join(datadir, f"ctrx{fid}_raw.bin")
        with open(fname, "rb") as f:
            f.seek(frameBytesPerCtrx * frameNr)
            raw_bytes = f.read(frameBytesPerCtrx)
        tmp = np.frombuffer(raw_bytes, dtype=np.int16)
        tmp = tmp.reshape((4, Ns, 2, Nc), order='F')         # (4, Ns, 2, Nc)
        Ii = tmp[:, :, 0, :].copy()                           # (4, Ns, Nc)
        Qi = tmp[:, :, 1, :].copy()                           # (4, Ns, Nc)
        tmp_ch = np.concatenate([Ii, Qi], axis=0)             # (8, Ns, Nc)
        Raw = np.concatenate([Raw, tmp_ch], axis=0)           # (nChannels, Ns, Nc)

    # permute: (nChannels, Ns, Nc) → (Nc, Ns, nChannels)
    #      再 → (Nc, nChannels, Ns)  匹配预处理预期的 (n_chirps, n_rx, n_samples)
    ADCdata = np.transpose(Raw, (2, 1, 0))                    # (Nc, Ns, nChannels)
    Raw = np.transpose(ADCdata, (0, 2, 1))                    # (Nc, nChannels, Ns)
    Raw = Raw.astype(np.float64, copy=False)

    return Raw
