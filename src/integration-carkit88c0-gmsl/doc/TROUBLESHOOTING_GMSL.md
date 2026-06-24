## When Reporting Issues

Include this information in bug reports:

**System Info:**
```bash
cat /etc/nv_tegra_release    # Jetson BSP version
uname -a                     # Kernel version
lscpu                        # CPU info
```

**Hardware State:**
```bash
sudo i2cdetect -l
ls -la /dev/spi* /dev/video* /dev/i2c-*
sudo cat /sys/kernel/debug/gpio
```

**Error Logs:**
```bash
# Last 30 lines of relevant kernel messages
sudo dmesg | grep -i radar | tail -30
sudo journalctl -k -b | grep -i radar | tail -30
```

**Reproduction Steps:**
- Exact commands that fail
- Expected vs actual behavior
- Any error messages displayed