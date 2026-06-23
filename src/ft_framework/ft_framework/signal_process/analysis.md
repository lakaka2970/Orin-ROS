jtop:
sudo apt update && sudo apt install python3-pip && sudo pip3 install jetson-stats
sudo systemctl restart jtop.service
sudo jtop



line_profiler
pip install line_profiler



pytorch

# 1. 安装必要的系统依赖<br/>sudo apt-get update<br/>sudo apt-get install -y libopenblas-base libopenmpi-dev libomp-dev<br/><br/># 2. 下载并安装 NVIDIA 官方专为 JetPack 5 编译的 PyTorch (以 v2.1.0 为例)<br/>wget https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl<br/>pip3 install torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl<br/>


# 1. 安装必要的系统依赖<br/>sudo apt-get update<br/>sudo apt-get install -y libopenblas-base libopenmpi-dev libomp-dev<br/><br/># 2. 下载并安装 NVIDIA 官方专为 JetPack 6 编译的 PyTorch (以 v2.3.0 为例)<br/>wget https://developer.download.nvidia.com/compute/redist/jp/v60/pytorch/torch-2.3.0a0+40ec155e-cp310-cp310-linux_aarch64.whl<br/>pip3 install torch-2.3.0a0+40ec155e-cp310-cp310-linux_aarch64.whl<br/>

import torch<br/>print("PyTorch 版本:", torch.__version__)<br/>print("CUDA 是否可用:", torch.cuda.is_available())<br/>


 # 1. 升级虚拟环境里的 pip<br/>pip install --upgrade pip<br/><br/># 2. 根据你的系统版本，选择一行安装（不需要加 sudo）<br/># 如果你的 Orin 是 JetPack 5：<br/>wget https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl<br/>  pip install torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl<br/>



 camera:
 1.sudo apt update
 sudo apt install v4l-utils
 v4l2-ctl --list-devices
 v4l2-ctl -d /dev/video1 --list-formats-ext


 opencv:

 1.python3 -c "import cv2; print(cv2._file_)"
 2.source /path/to/your/env/bin/activate
 cd /path/to/your/env/lib/python3.x/site-packages/
 ln -s /usr/lib/python3.8/dist-packages/cv2 ./cv2