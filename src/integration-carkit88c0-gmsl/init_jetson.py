#!/usr/bin/env python3 
# 完成硬件驱动的加载和配置
# 1.引脚复用（Pinmux）配置，适配 GMSL 外设的硬件引脚；
# 2.加载 / 验证 spidev 串口通信模块；
# 3.安装 / 卸载 max929x GMSL 驱动模块；
# 4.安装 / 卸载 radar 雷达驱动模块；
import os
import subprocess
import sys

def abspath(rel):# 将相对路径转为脚本所在目录的绝对路径
    dirname = os.path.dirname(__file__)
    return os.path.join(dirname, rel)

def run_command(cmd, check=True):# 定义系统命令执行函数：封装subprocess
    try:
        result = subprocess.run(cmd, check=check, capture_output=True, text=True)
        return result.returncode == 0
    except subprocess.CalledProcessError:
        return False

def is_module_loaded(module_name):# 定义内核模块检查函数：判断指定内核模块是否已加载
    try:
        result = subprocess.run(["lsmod"], capture_output=True, text=True, check=True)
        return module_name in result.stdout
    except subprocess.CalledProcessError:
        return False

def main():
    print("Starting CARKIT88C0 GMSL initialization...")
    # 第一步：执行Pinmux引脚复用配置（硬件引脚功能适配）
    # Execute pinmux configuration
    print("Configuring pinmux...")
    pinmux_script = abspath("artifacts/pinmux_config_gmsl_max9296.sh")
    if os.path.exists(pinmux_script):
        if run_command(["sudo", "bash", pinmux_script]):# 以sudo权限执行bash脚本
            print("✓ Pinmux configured")
        else:
            print("✗ Pinmux configuration failed")
            return 1
    else:
        print(f"✗ Pinmux script not found: {pinmux_script}")
        return 1
    # 第二步：检查/加载spidev模块（SPI串口通信模块，GMSL/雷达与主板通信的基础）
    # Insert spidev module
    print("Checking spidev module...")
    builtin_path = "/sys/module/spidev"
    if is_module_loaded("spidev"):
        print("✓ spidev already loaded")
    # Check if spidev is built-in
    elif os.path.exists(builtin_path):
        print("✓ spidev is built-in")
    else:
        if run_command(["sudo", "modprobe", "spidev"]):
            print("✓ spidev loaded")
        else:
            print("✗ Failed to load spidev")
            return 1
    # 第三步：安装max929x驱动（GMSL串行器/解串器芯片驱动，CARKIT88C0的核心GMSL外设驱动）
    # Insert max929x driver for 88c0
    print("Installing max929x driver...")
    max929x_dir = abspath("src/modules/max929x")
    max929x_builtin_path = "/sys/bus/i2c/drivers/max929x"
    load_max929x = True
    if os.path.exists(max929x_builtin_path):
        if run_command(["sudo", "rmmod", "max929x"], check=False):
            print("✓ max929x is already loaded -> unloading")
        else:
            load_max929x = False
            print("✓ max929x is built-in (cannot be unloaded)")
    if load_max929x:
        if os.path.exists(max929x_dir):
            if run_command(["make", "-C", max929x_dir, "install"]):
                print("✓ max929x driver installed")
            else:
                print("✗ Failed to install max929x driver")
                return 1
        else:
            print(f"✗ max929x directory not found: {max929x_dir}")
            return 1
    # 第四步：安装radar雷达模块（CARKIT88C0的雷达外设驱动）
    #Insert radar module for 88c0
    print("Installing radar module...")
    radar_dir = abspath("src/modules/radar")
    radar_builtin_path = "/sys/bus/i2c/drivers/Radar"
    load_radar = True
    if os.path.exists(radar_builtin_path):
        if run_command(["sudo", "rmmod", "radar"], check=False):
            print("✓ radar is already loaded -> unloading")
        else:
            load_radar = False
            print("✓ radar is built-in (cannot be unloaded)")
    if load_radar:
        if os.path.exists(radar_dir):
            if run_command(["make", "-C", radar_dir, "install"]):
                print("✓ radar module installed")
            else:
                print("✗ Failed to install radar module")
                return 1
        else:
            print(f"✗ radar directory not found: {radar_dir}")
            return 1
    
    print("CARKIT88C0 GMSL initialization completed successfully!")
    return 0

if __name__ == "__main__":
    sys.exit(main())
