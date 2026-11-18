#!/usr/bin/env python3
"""Initialize the connected Nintendo-labelled P3/XInput compatible controller."""
import argparse
import shutil
import subprocess
import ctypes
import ctypes.util
import os
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--install", action="store_true", help="安装仅匹配该兼容手柄的 USB 插入初始化规则")
args = parser.parse_args()
if os.geteuid() != 0:
    raise SystemExit("需要管理员 USB / udev 权限，请使用 sudo python3 执行此文件")

if args.install:
    destination = Path("/usr/local/lib/nav/p3_gamepad.py")
    rule = Path("/etc/udev/rules.d/99-nav-p3-gamepad.rules")
    source = Path(__file__).resolve()
    rule_text = ('ACTION=="add", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", '
                 'ATTR{idVendor}=="045e", ATTR{idProduct}=="028e", '
                 'ATTR{manufacturer}=="Nintendo Co., Ltd.", ATTR{product}=="XBOX 360 For Windows", '
                 'RUN+="/usr/bin/python3 /usr/local/lib/nav/p3_gamepad.py"\n')
    if destination.exists() and destination.read_bytes() != source.read_bytes():
        raise SystemExit("目标初始化脚本已存在且内容不同，请先备份再更新："+str(destination))
    if rule.exists() and rule.read_text() != rule_text:
        raise SystemExit("同名 udev 规则已存在且内容不同，未覆盖："+str(rule))
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination != source:
        shutil.copyfile(source, destination)
    os.chown(destination, 0, 0)
    destination.chmod(0o755)
    rule.write_text(rule_text)
    os.chown(rule, 0, 0)
    rule.chmod(0o644)
    subprocess.run(["udevadm", "control", "--reload-rules"], check=True)
    print("已安装 P3 兼容手柄自动初始化规则；仅在该型号 USB 插入时运行，不改变当前手柄或底盘状态。")
    raise SystemExit(0)

matches = []
for device in Path("/sys/bus/usb/devices").iterdir():
    if not (device/"idVendor").is_file():
        continue
    if ((device/"idVendor").read_text().strip() == "045e" and
            (device/"idProduct").read_text().strip() == "028e" and
            (device/"manufacturer").read_text().strip() == "Nintendo Co., Ltd."):
        matches.append(device)
if len(matches) != 1:
    raise SystemExit(f"期望恰好一只已确认的 P3 兼容手柄，实际找到 {len(matches)} 只；没有发送请求")

usb = ctypes.CDLL(ctypes.util.find_library("usb-1.0"))
usb.libusb_init.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
usb.libusb_init.restype = ctypes.c_int
usb.libusb_exit.argtypes = [ctypes.c_void_p]
usb.libusb_open_device_with_vid_pid.argtypes = [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint16]
usb.libusb_open_device_with_vid_pid.restype = ctypes.c_void_p
usb.libusb_close.argtypes = [ctypes.c_void_p]
usb.libusb_control_transfer.argtypes = [ctypes.c_void_p, ctypes.c_uint8, ctypes.c_uint8,
    ctypes.c_uint16, ctypes.c_uint16, ctypes.POINTER(ctypes.c_ubyte), ctypes.c_uint16, ctypes.c_uint]
usb.libusb_control_transfer.restype = ctypes.c_int
usb.libusb_error_name.argtypes = [ctypes.c_int]
usb.libusb_error_name.restype = ctypes.c_char_p
context = ctypes.c_void_p()
result = usb.libusb_init(ctypes.byref(context))
if result != 0:
    raise SystemExit("libusb 初始化失败："+usb.libusb_error_name(result).decode())
try:
    handle = usb.libusb_open_device_with_vid_pid(context, 0x045e, 0x028e)
    if not handle:
        raise SystemExit("无法打开已确认的 P3 兼容手柄；没有发送初始化请求")
    try:
        data = (ctypes.c_ubyte*20)()
        result = usb.libusb_control_transfer(handle, 0xc1, 0x01, 0x0100, 0, data, 20, 1000)
        if result < 0:
            raise SystemExit("手柄兼容初始化失败："+usb.libusb_error_name(result).decode())
        if result != 20:
            raise SystemExit(f"手柄返回长度异常：{result}，预期 20")
        print("P3 兼容手柄初始化请求成功，返回 20 字节。请按叉号键并推动摇杆验证。")
    finally:
        usb.libusb_close(handle)
finally:
    usb.libusb_exit(context)
