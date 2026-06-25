import numpy as np
from scipy import signal
from scipy.fftpack import fft
from scipy import io as sio
import os

###############################################################################
def readRawMat(fname):
    mat_contents = sio.loadmat(fname)
    Raw = mat_contents['ADC_R']
    Raw = np.float64(Raw)
    Raw = np.transpose(Raw, (0, 2, 1))
    return Raw

###############################################################################
def readRawBin(datadir, fid, Ns1, Nrx, Ns2):
    fname = datadir + f"/timedata_{fid:04d}.bin"
    Raw = np.fromfile(fname, np.int16)
    Raw = Raw.reshape((Ns1, Nrx, Ns2))
    Raw = np.float64(Raw)

    return Raw

###############################################################################
#读取 V4L2 原始捕获文件 (如 ctrx0_raw.bin)
#每帧格式: nRows 行 × nSamples 列 uint16 (RG12, 高12位有效)
#行排列: RX0_chirp0..RX0_chirpN, RX1_chirp0..RX1_chirpN, ...
#输出: (nSamples, nRx, nChirpsPerRx) 的 float64 数组
def readRawBinV4L2(filepath, frameNr, nSamples, nRows, nRx):
    nChirpsPerRx = nRows // nRx
    frameSize = nRows * nSamples * 2  # uint16 = 2 bytes

    f = open(filepath, "rb")
    f.seek(frameSize * frameNr)
    data = f.read(frameSize)
    f.close()

    if len(data) < frameSize:
        raise EOFError(f"Frame {frameNr}: 期望 {frameSize} 字节, 实际 {len(data)} 字节")

    Raw = np.frombuffer(data, np.uint16)
    Raw = Raw.reshape((nRows, nSamples))

    # RG12: 清除低4位填充位 (保留高12位有效数据)
    Raw = np.bitwise_and(Raw, np.uint16(0xFFF0))
    Raw = Raw.astype(np.int16, copy=False)

    # 重塑为 (nRx, nChirpsPerRx, nSamples)
    Raw = Raw.reshape((nRx, nChirpsPerRx, nSamples))
    # 转置为 (nSamples, nRx, nChirpsPerRx) 符合后续处理约定
    Raw = np.transpose(Raw, (2, 0, 1))

    Raw = np.float64(Raw)
    return Raw

###############################################################################
#多帧拼接: 读取多个连续帧并沿 chirp 维度拼接
def readRawBinV4L2Multi(filepath, startFrame, numFrames, nSamples, nRows, nRx):
    frames = []
    for i in range(numFrames):
        frame = readRawBinV4L2(filepath, startFrame + i, nSamples, nRows, nRx)
        frames.append(frame)
    return np.concatenate(frames, axis=2)  # (nSamples, nRx, totalChirps)

###############################################################################
#数据读取 拼接 去除硬件填充位
def readRawBinCasc(datadir, frameNr, nSamples, nRamps, nChannels):
    
    Nbins = nSamples*nRamps*4#一片 CTRX 固定 4 个通道
    frameSize = Nbins*2
    
    fname = "ctrx0.bin"
    fstat = os.stat(fname)
    numFrames = fstat.st_size / frameSize
    assert frameNr < numFrames

    fid = 0
    fname = datadir + f"/ctrx{fid}_bin.raw"
    f = open(fname, "rb")
    f.seek(frameSize*frameNr)
    data = f.read(frameSize)
    f.close()
    Raw = np.frombuffer(data, np.uint16)#二进制字节流 → 转成 uint16 数组
    Raw = Raw.reshape((nRamps, nSamples, 4))

    numCtrx = int(round(nChannels / 4))
    for fid in range(1,numCtrx):
        fname = datadir + f"/ctrx{fid}_bin.raw"
        f = open(fname, "rb")
        f.seek(frameSize*frameNr)
        data = f.read(frameSize)
        f.close()
        tmp = np.frombuffer(data, np.uint16)
        tmp = Raw.reshape((nRamps, nSamples, 4)) ## 应该为 tmp = tmp.reshape((nRamps, nSamples, 4))？？
        Raw = np.concat((Raw,tmp), 2)

    Raw = np.transpose(Raw, (1, 2, 0))#(nRamps, nSamples, nChannels) -> (nSamples, nChannels, nRamps)

    # remove padded bits:
    Raw = np.bitwise_and(Raw, np.uint16(0xFFF0))#清4bit 填充位
    Raw = Raw.astype(np.int16, copy=False)

    Raw = np.float64(Raw)

    return Raw

###############################################################################
def rdFft(Raw):
    Ns1, Nrx, Ns2 = Raw.shape

    Wd1 = signal.windows.chebwin(Ns1, 80)#生成切比雪夫窗 窗衰减80dB
    Wd2 = signal.windows.chebwin(Ns2, 80)

    Nf1 = Ns1
    Nf2 = Ns2

    Nrang = int(Nf1/2)
    Ndopp = Nf2

    # Range FFT
    Ff1 = np.complex64(np.zeros((Nrang,Nrx,Ndopp)))#创建空数组存距离 FFT 结果
    for chirp in range(0, Ns2):
        for rx in range(0, Nrx):
            Rw = Raw[:,rx,chirp] * Wd1
            tmp = fft(Rw, Nf1)
            Ff1[:,rx,chirp] = tmp[0:Nrang]

    # Doppler FFT
    RD = np.complex64(np.zeros((Ndopp,Nrang,Nrx)))
    for rx in range(0, Nrx):
        for rg in range(0, Nrang):
            Rw = Ff1[rg,rx,:] * Wd2
            tmp = fft(Rw, Nf2)
            RD[:,rg,rx] = tmp

    return RD#RD(doppler,range,rx)

###############################################################################
def nci(RD):
    Ndopp,Nrang,Nrx = RD.shape
    # NCI
    Plin = np.abs(RD)**2#abs^2
    NCI = Plin[:,:,0]
    for rx in range(1, Nrx):
        NCI = NCI + Plin[:,:,rx]

    return NCI

###############################################################################
def localMax(NCI):#十字
    Ndopp, Nrang = NCI.shape
    LMAP = np.zeros((Ndopp, Nrang), bool)

    NCIt = np.concatenate((NCI[(Ndopp-1):Ndopp,:],NCI,NCI[0:1,:]))# 对 多普勒维度做循环填充/边缘扩展
    NCIt = np.concatenate((NCIt[:,1:2],NCIt,NCIt[:,(Nrang-2):(Nrang-1)]),axis=1)

    for r in range(0,Nrang):
        for d in range(0,Ndopp):
            cut = NCIt[d+1,r+1]
            dmax = (cut > NCIt[d+2,r+1]) & (cut > NCIt[d+0,r+1])
            rmax = (cut > NCIt[d+1,r+2]) & (cut > NCIt[d+1,r+0])
            LMAP[d,r] = dmax & rmax

    return LMAP

###############################################################################
def thresholding(NCI, beta_dB):#50%
    Ndopp, Nrang = NCI.shape

    S = np.zeros((Ndopp,Nrang))
    for r in range(0,Nrang):
        tmp = np.sort(NCI[:,r])
        S[:,r] = tmp[int(Ndopp/2)]

    threshold = S * (10**(beta_dB/10))
    TMAP = (NCI > threshold)

    return TMAP

###############################################################################
def matching(TMAP, txCode):#tx都检测到的才能通过
    Ndopp, Nrange = TMAP.shape
    shift = np.int32(txCode*Ndopp)

    Ntx = txCode.size
    MMAP = np.ones((Ndopp,Nrange), bool)
    for tx in range(0, Ntx):
        MMAP = MMAP & np.roll(TMAP, -shift[tx], axis=0)

    return MMAP

###############################################################################
def getPeaks(DMAP):
    Ndopp, Nrange = DMAP.shape
    PEAKS = []
    
    for r in range(1, Nrange):
        for d in range (0, Ndopp):
            if DMAP[d,r]:
                tmp = (r,d)
                PEAKS.append(tmp)
            
    return PEAKS

###############################################################################
def getPeaksWithEnergy(DMAP, NCI):
    """返回峰值列表，每个元素为 (range_bin, doppler_bin, energy_dB)"""
    Ndopp, Nrange = DMAP.shape
    NCI_dB = 10 * np.log10(NCI + 1e-10)
    PEAKS = []
    for r in range(1, Nrange):
        for d in range(0, Ndopp):
            if DMAP[d, r]:
                PEAKS.append((r, d, NCI_dB[d, r]))
    return PEAKS

###############################################################################
def estimateAzimuth(RD, peaks, wavelength=0.0039, d_rx=0.00195):
    """
    简单相位比较法估计方位角 (适用于4RX均匀线阵)

    Args:
        RD: Range-Doppler cube, shape (nDopp, nRange, nRx)
        peaks: list of (range_bin, doppler_bin, ...)
        wavelength: 雷达波长 [m], 默认 77GHz
        d_rx: 相邻RX天线间距 [m], 默认 lambda/2

    Returns:
        list of (range_bin, doppler_bin, azimuth_deg, energy...)
    """
    Ndopp, Nrang, Nrx = RD.shape
    results = []

    for peak in peaks:
        r_bin = peak[0]
        d_bin = peak[1]

        if Nrx < 2:
            continue

        # 提取峰值处的RX通道数据
        rx_data = RD[d_bin, r_bin, :]  # (nRx,)

        # 相邻RX对相位差 (共 Nrx-1 对)
        phase_diffs = []
        for rx in range(Nrx - 1):
            cross = rx_data[rx] * np.conj(rx_data[rx + 1])
            phase_diff = np.angle(cross)
            phase_diffs.append(phase_diff)

        # 平均相位差
        avg_phase = np.mean(phase_diffs)

        # 方位角: theta = arcsin(phase * lambda / (2*pi*d))
        sin_theta = avg_phase * wavelength / (2 * np.pi * d_rx)
        sin_theta = np.clip(sin_theta, -1.0, 1.0)
        azimuth_rad = np.arcsin(sin_theta)
        azimuth_deg = np.rad2deg(azimuth_rad)

        results.append((*peak, azimuth_deg, azimuth_rad))

    return results

###############################################################################
def mimoVector(RD, rdIdx, txCode, vIdx):
    #1: collect according to idx and tx code
    #2: arrange according to vIdx

    return mVec

###############################################################################
def axis(NCI):
    Ndopp, Nrang = NCI.shape

    y = np.linspace(0, Ndopp-1, Ndopp)
    x = np.linspace(0, Nrang-1, Nrang)
    x,y = np.meshgrid(x,y)

    return x,y

###############################################################################
def axisPlotly(NCI):
    Ndopp, Nrang = NCI.shape

    y = np.linspace(0, Ndopp-1, Ndopp)
    x = np.linspace(0, Nrang-1, Nrang)
    x,y = np.meshgrid(x,y)

    return x,y
