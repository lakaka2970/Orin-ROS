#!/bin/bash

# ------------------------------------------------------------------------------
# This debug script polls CSI-2 radar data according to the v4l2-ctl parameters
# and streams the data to a raw binary file.
# Adapt the device, width, height, pixel format and output file as needed.
# It takes ownership of the video device file descriptor.
#
# The padding applied by NVIDIA for RAW-12 appends 4 redundant bits after
# 12 data bits to round the 12 bits up to 16. Check the NVIDIA technical
# reference document for more information on padding.
#
# 设备: 优先使用 udev 符号链接 /dev/radar_ctrx{0,1},
#       回退到 /dev/video2/video3 (tegra-video 雷达设备)。
#       USB 摄像头占用 /dev/video0/video1 时不受影响。
# ------------------------------------------------------------------------------

# Adapt parameters as needed
OUTPUT_FOLDER="output"
OUTPUT_FILE="${OUTPUT_FOLDER}/ctrx0_raw.bin"
OUTPUT_FILE_1="${OUTPUT_FOLDER}/ctrx1_raw.bin"
WIDTH="8192"
HEIGHT="1024"
# WIDTH="4096"
# HEIGHT="2048"

PIXEL_FORMAT="RG12"

# 优先使用 udev 持久化符号链接, 回退到数字设备号
if [ -e "/dev/radar_ctrx0" ]; then
    DEVICE="/dev/radar_ctrx0"
elif [ -e "/dev/video2" ]; then
    DEVICE="/dev/video2"
else
    DEVICE="/dev/video0"
fi

if [ -e "/dev/radar_ctrx1" ]; then
    DEVICE_1="/dev/radar_ctrx1"
elif [ -e "/dev/video3" ]; then
    DEVICE_1="/dev/video3"
else
    DEVICE_1="/dev/video1"
fi

rm -rf "$OUTPUT_FOLDER"
mkdir -p "$OUTPUT_FOLDER"

# Number of frames to capture per device
STREAM_COUNT=20

# 超时时间: 每帧约 65ms @15Hz, 20帧 ≈ 1.3s, 留足余量用 30s
CAPTURE_TIMEOUT=30

# ------------------------------------------------------------------------------
# 采集函数: device → output_file
# 使用 timeout 防止硬件未推流时 v4l2-ctl 永久阻塞 DQBUF
# ------------------------------------------------------------------------------
capture_device() {
    local dev="$1"
    local out="$2"
    local label="$3"

    echo "[INFO] Polling radar data from $dev -> $out"

    if [ ! -e "$dev" ]; then
        echo "[ERROR] 设备不存在: $dev"
        return 1
    fi

    # 验证设备是否为 tegra-video 雷达 (仅取 Driver Info 段的驱动名)
    local driver_name
    driver_name=$(v4l2-ctl -d "$dev" --info 2>/dev/null | grep -m1 "Driver name" | awk '{print $NF}')
    if [ "$driver_name" != "tegra-video" ]; then
        echo "[WARN] $dev 驱动为 '$driver_name' 而非 'tegra-video', 可能不是雷达设备"
    fi

    timeout $CAPTURE_TIMEOUT v4l2-ctl -d "$dev" \
        --set-fmt-video=width=$WIDTH,height=$HEIGHT,pixelformat=$PIXEL_FORMAT \
        --set-ctrl bypass_mode=0 \
        --stream-mmap \
        --stream-count=$STREAM_COUNT \
        --stream-to="$out"

    local exit_code=$?

    case $exit_code in
        0)
            local fsize=$(stat -c%s "$out" 2>/dev/null || echo 0)
            echo "[OK]   $label: $fsize bytes ($STREAM_COUNT frames)"
            ;;
        124)
            echo "[ERROR] $label: 采集超时 (${CAPTURE_TIMEOUT}s), 雷达硬件可能未推流"
            echo "       请确认已执行: sudo python3 init_jetson.py"
            return 1
            ;;
        *)
            echo "[ERROR] $label: 采集失败 (exit=$exit_code)"
            return 1
            ;;
    esac
}

# ------------------------------------------------------------------------------
# 主流程
# ------------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=============================================="
echo "  雷达原始数据采集"
echo "  设备:   $DEVICE + $DEVICE_1"
echo "  分辨率: ${WIDTH}x${HEIGHT} $PIXEL_FORMAT"
echo "  帧数:   $STREAM_COUNT / 设备"
echo "  输出:   $SCRIPT_DIR/$OUTPUT_FOLDER/"
echo "=============================================="

FAILED=0

capture_device "$DEVICE" "$OUTPUT_FILE" "ctrx0"
[ $? -ne 0 ] && FAILED=1

echo ""
capture_device "$DEVICE_1" "$OUTPUT_FILE_1" "ctrx1"
[ $? -ne 0 ] && FAILED=1

echo ""
if [ $FAILED -eq 0 ]; then
    echo "[DONE] 采集完成"
    echo "  ctrx0: $SCRIPT_DIR/$OUTPUT_FILE"
    echo "  ctrx1: $SCRIPT_DIR/$OUTPUT_FILE_1"
    exit 0
else
    echo "[DONE] 采集部分失败，请检查上述错误信息"
    exit 1
fi
