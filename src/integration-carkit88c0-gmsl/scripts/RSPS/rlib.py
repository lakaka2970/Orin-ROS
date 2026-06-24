import numpy as np
from scipy import signal
from scipy import fft
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
def readRawBinCasc(datadir, frameNr, nSamples, nRamps, nChannels):
    
    Nbins = nSamples*nRamps*4
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
    Raw = np.frombuffer(data, np.uint16)
    Raw = Raw.reshape((nRamps, nSamples, 4))

    numCtrx = int(round(nChannels / 4))
    for fid in range(1,numCtrx):
        fname = datadir + f"/ctrx{fid}_bin.raw"
        f = open(fname, "rb")
        f.seek(frameSize*frameNr)
        data = f.read(frameSize)
        f.close()
        tmp = np.frombuffer(data, np.uint16)
        tmp = Raw.reshape((nRamps, nSamples, 4))
        Raw = np.concat((Raw,tmp), 2)

    Raw = np.transpose(Raw, (1, 2, 0))

    # remove padded bits:
    Raw = np.bitwise_and(Raw, np.uint16(0xFFF0))
    Raw = Raw.astype(np.int16, copy=False)

    Raw = np.float64(Raw)

    return Raw

###############################################################################
def rdFft(Raw):
    Ns1, Nrx, Ns2 = Raw.shape

    Wd1 = signal.windows.chebwin(Ns1, 80)
    Wd2 = signal.windows.chebwin(Ns2, 80)

    Nf1 = Ns1
    Nf2 = Ns2

    Nrang = int(Nf1/2)
    Ndopp = Nf2

    # Range FFT
    Ff1 = np.complex64(np.zeros((Nrang,Nrx,Ndopp)))
    for chirp in range(0, Ns2):
        for rx in range(0, Nrx):
            Rw = Raw[:,rx,chirp] * Wd1
            tmp = fft.fft(Rw, Nf1)
            Ff1[:,rx,chirp] = tmp[0:Nrang]

    # Doppler FFT
    RD = np.complex64(np.zeros((Ndopp,Nrang,Nrx)))
    for rx in range(0, Nrx):
        for rg in range(0, Nrang):
            Rw = Ff1[rg,rx,:] * Wd2
            tmp = fft.fft(Rw, Nf2)
            RD[:,rg,rx] = tmp

    return RD

###############################################################################
def nci(RD):
    Ndopp,Nrang,Nrx = RD.shape
    # NCI
    Plin = np.abs(RD)**2
    NCI = Plin[:,:,0]
    for rx in range(1, Nrx):
        NCI = NCI + Plin[:,:,rx]

    return NCI

###############################################################################
def localMax(NCI):
    Ndopp, Nrang = NCI.shape
    LMAP = np.zeros((Ndopp, Nrang), bool)

    NCIt = np.concatenate((NCI[(Ndopp-1):Ndopp,:],NCI,NCI[0:1,:]))
    NCIt = np.concatenate((NCIt[:,1:2],NCIt,NCIt[:,(Nrang-2):(Nrang-1)]),axis=1)

    for r in range(0,Nrang):
        for d in range(0,Ndopp):
            cut = NCIt[d+1,r+1]
            dmax = (cut > NCIt[d+2,r+1]) & (cut > NCIt[d+0,r+1])
            rmax = (cut > NCIt[d+1,r+2]) & (cut > NCIt[d+1,r+0])
            LMAP[d,r] = dmax & rmax

    return LMAP

###############################################################################
def thresholding(NCI, beta_dB):
    Ndopp, Nrang = NCI.shape

    S = np.zeros((Ndopp,Nrang))
    for r in range(0,Nrang):
        tmp = np.sort(NCI[:,r])
        S[:,r] = tmp[int(Ndopp/2)]

    threshold = S * (10**(beta_dB/10))
    TMAP = (NCI > threshold)

    return TMAP

###############################################################################
def matching(TMAP, txCode):
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
