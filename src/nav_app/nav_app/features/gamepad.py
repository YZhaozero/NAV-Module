"""Joystick discovery output, guided mapping and per-model profiles."""
import json
import math
import re
from pathlib import Path


def parse_devices(output):
    devices = []
    for line in output.splitlines():
        if not line.strip() or line.startswith("ID :") or set(line.strip()) == {"-"}:
            continue
        match = re.fullmatch(r"\s*(\d+)\s*:\s*([0-9a-fA-F]{32})\s*:\s*(?:true|false)\s*:\s*(?:true|false)\s*:\s*(.+)", line)
        if match is None:
            raise ValueError("无法解析手柄枚举结果："+line)
        number, guid, name = match.groups()
        devices.append(dict(id=int(number), guid=guid.lower(), name=name.strip()))
    return devices


def validate_profile(profile):
    if profile["chassis"] not in ("ackermann", "omni", "differential"):
        raise ValueError("手柄标定中的车型无效")
    axes, signs, centers = profile["axes"], profile["signs"], profile["centers"]
    if len(axes) != 3 or len(signs) != 3 or not centers:
        raise ValueError("手柄轴标定不完整")
    required = range(3) if profile["chassis"] == "omni" else (0, 2)
    if any(type(axes[i]) is not int or not 0 <= axes[i] < len(centers) or signs[i] not in (-1, 1) for i in required):
        raise ValueError("手柄轴编号或方向无效")
    if len({axes[i] for i in required}) != len(tuple(required)):
        raise ValueError("不同动作不能使用同一个轴")
    if any(not math.isfinite(v) or abs(v) > 1 for v in centers):
        raise ValueError("手柄中位无效")
    if type(profile["deadman"]) is not int or profile["deadman"] < 0:
        raise ValueError("保持键标定无效")
    if not 0 <= profile["linear"] <= .5 or not 0 <= profile["angular"] <= 1:
        raise ValueError("手柄速度设置无效")


def load_profiles(path):
    if not Path(path).exists():
        return {}
    profiles = json.loads(Path(path).read_text())
    if not isinstance(profiles, dict):
        raise ValueError("手柄配置应为设备标识到标定参数的映射")
    for guid, profile in profiles.items():
        if not re.fullmatch("[0-9a-f]{32}", guid):
            raise ValueError("手柄设备标识无效")
        validate_profile(profile)
    return profiles


def save_profile(path, guid, profile):
    validate_profile(profile)
    profiles = load_profiles(path)
    profiles[guid] = profile
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(profiles, ensure_ascii=False, indent=2)+"\n")
    temporary.replace(path)


class Calibration:
    def __init__(self, chassis):
        self.chassis = chassis
        self.stage = "center"
        self.centers = ()
        self.axes = [-1, -1, -1]
        self.signs = [1, 1, 1]
        self.deadman = None
        self.actions = [0, 2, 1] if chassis == "omni" else [0, 2]
        self.action = 0
        self.candidate = None
        self.candidate_at = 0.
        self.message = "放开所有按钮，摇杆回中，点击“记录中位”"

    def check(self, sample, now):
        if sample is None or not 0 <= now-sample[2] <= .3:
            raise ValueError("未收到新鲜手柄数据，请检查连接")
        axes, buttons, _ = sample
        if not axes or not buttons or any(not math.isfinite(v) or abs(v) > 1 for v in axes):
            raise ValueError("手柄输入无效")
        if any(v not in (0, 1) for v in buttons):
            raise ValueError("手柄按钮输入无效")
        if self.centers and len(axes) != len(self.centers):
            raise ValueError("手柄轴数量变化，请重新标定")
        return axes, buttons

    def center(self, sample, now):
        axes, buttons = self.check(sample, now)
        if any(buttons):
            raise ValueError("请先松开所有按钮")
        self.centers = tuple(axes)
        self.stage = "button"
        self.message = "按一下作为“允许移动”的保持键，只按一个键"

    def next_axis(self):
        self.stage = "axis"
        title = {0:"前后移动：将选定摇杆向前推到底", 2:"转向：将选定摇杆向左推到底", 1:"侧移：将另一个摇杆向左推到底"}
        self.message = title[self.actions[self.action]]+"，停住片刻"
        self.candidate = None

    def update(self, sample, now):
        axes, buttons = self.check(sample, now)
        if self.stage == "button":
            pressed = [i for i, value in enumerate(buttons) if value]
            if len(pressed) == 1:
                self.deadman = pressed[0]
                self.stage = "release_button"
                self.message = f"已识别保持键 {self.deadman}，请松开"
            elif len(pressed) > 1:
                self.message = "请只按一个保持键"
        elif self.stage == "release_button" and not any(buttons):
            self.next_axis()
        elif self.stage == "axis":
            moved = sorted(((abs(value-self.centers[i]), i, value-self.centers[i]) for i, value in enumerate(axes)), reverse=True)
            magnitude, index, delta = moved[0]
            if magnitude < .6 or (len(moved) > 1 and moved[1][0] > magnitude*.5) or any(buttons):
                self.candidate = None
                return
            if index in self.axes:
                self.message = "这个轴已用于其他动作，请换另一个方向或摇杆"
                self.candidate = None
                return
            candidate = (index, 1 if delta > 0 else -1)
            if candidate != self.candidate:
                self.candidate, self.candidate_at = candidate, now
            elif now-self.candidate_at >= .2:
                action = self.actions[self.action]
                self.axes[action], self.signs[action] = candidate
                self.stage = "return"
                self.message = "已识别轴及方向，请将摇杆回中并松开按钮"
        elif self.stage == "return":
            if not any(buttons) and all(abs(axes[i]-self.centers[i]) < .12 for i in self.axes if i >= 0):
                self.action += 1
                if self.action == len(self.actions):
                    self.stage, self.message = "done", "标定完成。确认车型和速度后，点击“保存标定”"
                else:
                    self.next_axis()

    def profile(self, context):
        if self.stage != "done":
            raise ValueError("请先完成所有标定步骤")
        return dict(context, axes=self.axes.copy(), signs=self.signs.copy(), centers=list(self.centers), deadman=self.deadman)
