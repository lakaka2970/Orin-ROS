# Troubleshooting & Debugging

This document provides troubleshooting guidance for common issues with the Jetson + Infineon radar integration.

## Common Error Scenarios

### Hardware Communication Errors

**I2C Communication Failures:**
```bash
# Error in dmesg: "i2c transfer failed"
# Diagnosis commands:
sudo i2cdetect -l                    # List available I2C buses
sudo i2cdetect -y 2                  # Scan bus 2 (adjust as needed)
ls -la /dev/i2c-*                    # Check I2C device permissions

# Solutions:
# - Add user to i2c group: sudo usermod -a -G i2c $USER
# - Check physical connections
# - Verify correct I2C bus in device tree
```

**SPI Communication Problems:**
```bash
# Error: Cannot access SPI device
# Diagnosis:
ls -la /dev/spi*
sudo dmesg | grep spi
lsmod | grep spidev

# Solutions:
sudo modprobe spidev                 # Load SPI user-space driver
```

**GPIO Issues:**
```bash
# Error: GPIO operation failed
# Diagnosis:
sudo dmesg | grep gpio
cat /sys/kernel/debug/gpio           # Show GPIO usage
ls -la /sys/class/gpio/
```

### Video Capture Issues

**Problem:** V4L2 capture fails or returns no data
- **Symptoms:**
  - `v4l2-ctl --list-devices` shows no radar devices
  - Capture command fails with "No such device"
  - Data captured but file size is wrong/zero

**Diagnosis:**
```bash
# Check if V4L2 devices exist
ls /dev/video*

# Get device information
v4l2-ctl --list-devices

# Check device capabilities
v4l2-ctl -d /dev/video0 --all

# Look for V4L2 errors in kernel log
sudo dmesg | grep -i v4l2

# List supported formats
v4l2-ctl -d /dev/video0 --list-formats-ext
```

**Solutions:**
- Ensure radar driver loaded before checking V4L2
- Verify correct video device number
- Check device permissions: `ls -la /dev/video*`

## Debugging Tools

### Enable Kernel Tracing

**Purpose:** Get detailed driver execution information for debugging complex issues, timing analysis, and understanding driver behavior.

**Using the tracing script**

The `scripts/tracing.sh` script provides an easy way to enable the kernel tracing:

```bash
# Enable tracing and print the tracing output continuously
sudo ./scripts/tracing.sh

# Start the capture process
# e.g. v4l2-ctl -d /dev/video0 --set-fmt-video=width=$WIDTH,height=$HEIGHT,pixelformat=RG12 --stream-mmap --stream-count=$FRAMES --stream-to="$OUTPUT_FILE"

# Run your radar application that's causing issues

# Watch the tracing output
# See below how to decode some of the outputs.

# To quit the output printing
# Ctrl+C and then press `q` to exit the trace viewer
```

**What the tracing script does:**
- Enables ftrace events
- Sets the buffer size to a value large enough for the trace
- Provides output of driver function calls and events
- Automatically handles trace buffer management

### Understanding Trace Output

**Typical trace output format:**
```
# tracer: nop
#
# entries-in-buffer/entries-written: 0/0   #P:12
#
#                                _-----=> irqs-off
#                               / _----=> need-resched
#                              | / _---=> hardirq/softirq
#                              || / _--=> preempt-depth
#                              ||| /     delay
#           TASK-PID     CPU#  ||||   TIMESTAMP  FUNCTION
#              | |         |   ||||      |         |
 vi-output, Rada-67069   [011] .... 10484.952129: tegra_channel_capture_setup: vnc_id 0 W 2048 H 512 fmt c4
```

**How to decode the output:**

1. **Process Information:**
   - `kworker/4:2-123`: Process name and PID that triggered the event
   - `[001]`: CPU core number where event occurred

2. **Timing Information:**
   - `12345.123456`: Timestamp in seconds since boot

3. **Function Calls:**
   - `tegra_channel_capture_setup`
   - `rtcpu_vinotify_event`

**Successful frame reception:**
```
     kworker/4:2-123     [004] .... 10607.706896: rtcpu_vinotify_event: tstamp:332364093722 cch:0 vi:0 tag:FS channel:0x00 frame:0 vi_tstamp:10635650762144 data:0x0000000000000010
     kworker/4:2-123     [004] .... 10607.706899: rtcpu_vinotify_event: tstamp:332364093862 cch:0 vi:0 tag:ATOMP_FS channel:0x00 frame:0 vi_tstamp:10635650762208 data:0x0000000800000000
     kworker/4:2-123     [004] .... 10607.706900: rtcpu_vinotify_event: tstamp:332364094046 cch:0 vi:0 tag:CHANSEL_PXL_SOF channel:0x23 frame:0 vi_tstamp:10635650785216 data:0x0000000000000001
     kworker/4:2-123     [004] .... 10607.706903: rtcpu_vinotify_event: tstamp:332364094180 cch:0 vi:0 tag:VIFALC_ACTIONLST channel:0x23 frame:0 vi_tstamp:10635650804288 data:0x0000000008020003
     kworker/4:2-123     [004] .... 10607.706907: rtcpu_vinotify_event: tstamp:332364534376 cch:0 vi:0 tag:VIFALC_TDSTATE channel:0x23 frame:0 vi_tstamp:10635651091392 data:0x359d550010000000
     kworker/4:2-123     [004] .... 10607.706909: rtcpu_vinotify_event: tstamp:332364534513 cch:0 vi:0 tag:VIFALC_TDSTATE channel:0x23 frame:0 vi_tstamp:10635651134720 data:0x0000000031000004
     kworker/4:2-123     [004] .... 10607.706911: rtcpu_vinotify_event: tstamp:332364534684 cch:0 vi:0 tag:CHANSEL_PXL_EOF channel:0x23 frame:0 vi_tstamp:10635660999968 data:0x0000000001ff0002
     kworker/4:2-123     [004] .... 10607.706913: rtcpu_vinotify_event: tstamp:332364534816 cch:0 vi:0 tag:FE channel:0x00 frame:0 vi_tstamp:10635661000608 data:0x0000000000000020
     kworker/4:2-123     [004] .... 10607.706915: rtcpu_vinotify_event: tstamp:332364534973 cch:0 vi:0 tag:ATOMP_FE channel:0x00 frame:0 vi_tstamp:10635661000704 data:0x0000000800000000
     kworker/4:2-123     [004] .... 10607.706915: rtcpu_vinotify_event: tstamp:332364535117 cch:0 vi:0 tag:ATOMP_FRAME_DONE channel:0x23 frame:0 vi_tstamp:10635661000928 data:0x0000000000000000
```

**Key words and root cause of the problems:**
  - `tag:CHANSEL_FAULT`: Received data format does not match the configured format via v4l2-ctl.
  - `tag:CHANSEL_SHORT_FRAME`: Less lines (ramps) received than expected.