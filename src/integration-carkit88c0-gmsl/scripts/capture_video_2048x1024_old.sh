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
WIDTH="8192"
HEIGHT="1024"
# WIDTH="4096"
# HEIGHT="2048"

PIXEL_FORMAT="RG12"
DEVICE="/dev/video0"

rm -rf "$OUTPUT_FOLDER"
mkdir -p "$OUTPUT_FOLDER"

# Double-check that the dev-node matches your intent
echo "[INFO] Polling radar data"
v4l2-ctl -d $DEVICE \
    --set-fmt-video=width=$WIDTH,height=$HEIGHT,pixelformat=$PIXEL_FORMAT \
    --set-ctrl bypass_mode=0 \
    --stream-mmap \
    --stream-to=$OUTPUT_FILE
    
EXIT_VALUE=$?
if [ $EXIT_VALUE -ne 0 ]; then
    echo "[ERROR] v4l2-ctl returned a non-zero exit status"
    exit $EXIT_VALUE
fi
