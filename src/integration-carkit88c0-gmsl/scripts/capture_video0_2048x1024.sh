#!/bin/bash

# ------------------------------------------------------------------------------
# This debug script polls CSI-2 radar data according the the v4l2-ctl parameters
# and streams the data to a raw binary file.
# Adapt the device, width, height, pixel format and output file as needed.
# It takes ownership of the /dev/video0 file descriptor.
#
# The padding applied by NVIDIA for RAW-12 appends 4 redundant bits after
# 12 data bits to round the 12 bits up to 16. Check the NVIDIA technical 
# reference document for more information on padding.
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
DEVICE="/dev/video0"
DEVICE_1="/dev/video1"

rm -rf "$OUTPUT_FOLDER"
mkdir -p "$OUTPUT_FOLDER"

# Number of frames to capture per device
STREAM_COUNT=20

# Double-check that the dev-node matches your intent
echo "[INFO] Polling radar data from $DEVICE -> $OUTPUT_FILE"
v4l2-ctl -d $DEVICE \
    --set-fmt-video=width=$WIDTH,height=$HEIGHT,pixelformat=$PIXEL_FORMAT \
    --set-ctrl bypass_mode=0 \
    --stream-mmap \
    --stream-count=$STREAM_COUNT \
    --stream-to=$OUTPUT_FILE

if [ $? -ne 0 ]; then
    echo "[ERROR] Failed to capture from $DEVICE"
fi

echo "[INFO] Polling radar data from $DEVICE_1 -> $OUTPUT_FILE_1"
v4l2-ctl -d $DEVICE_1 \
    --set-fmt-video=width=$WIDTH,height=$HEIGHT,pixelformat=$PIXEL_FORMAT \
    --set-ctrl bypass_mode=0 \
    --stream-mmap \
    --stream-count=$STREAM_COUNT \
    --stream-to=$OUTPUT_FILE_1

if [ $? -ne 0 ]; then
    echo "[ERROR] Failed to capture from $DEVICE_1"
fi
