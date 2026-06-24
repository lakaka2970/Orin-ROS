import rlib
import numpy as np
import plotly.graph_objects as go
import plotly.io as pio
from matplotlib import pyplot as plt

#plt.rcParams['figure.dpi'] = 600

pio.renderers.default = "browser"

frameNr = 0
nSamples = 1024
nRamps = 1024
nRx = 8
Raw = rlib.readRawBinCasc(".", frameNr, nSamples, nRamps, nRx)

txCode = np.array((0, 13, 10, +7, +2, +3, +4, +5))/16
txCode = np.mod(-txCode,1)
shift = txCode*1024

RD    = rlib.rdFft(Raw)
NCI   = rlib.nci(RD)
LMAP  = rlib.localMax(NCI)
TMAP  = rlib.thresholding(NCI,9)
MMAP  = rlib.matching(TMAP, txCode)
DMAP  = LMAP & MMAP
PEAKS = rlib.getPeaks(DMAP)
#mVec  = rlib.mimoVector(RD, rdIdx, txCode, vIdx);

J = 10*np.log10(NCI)

###############################################################################
#Z = DMAP
Z = J
Z = np.transpose(Z)

if True:
    x,y = rlib.axis(Z)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(x, y, Z, cmap='jet')
    ax.view_init(-90,0,0)
    ax.set_proj_type('ortho')
    plt.savefig("graph.png")
    plt.show()

if False:
    x,y = rlib.axisPlotly(Z)
    fig = go.Figure(data=[go.Surface(z=Z, x=x, y=y)])
    fig.update_layout(title='Mt Bruno Elevation', autosize=False,
                      width=600, height=600,
                      scene_camera=dict(eye=dict(x=0.,y=0.,z=-10.)),
                      margin=dict(l=65, r=50, b=65, t=90))
    #fig.layout.scene.camera.projection.type = "orthographic"
    fig.update_scenes(camera_projection_type='orthographic')
    fig.show()
