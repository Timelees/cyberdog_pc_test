#!/usr/bin/env python3

import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
SIM_ROOT = SCRIPT_DIR.parent
WORKSPACE_SRC = SIM_ROOT.parent
LCM_ROOT = WORKSPACE_SRC / "lcm"

MESSAGE_TYPE_DIR = (
    SIM_ROOT / "src" / "cyberdog_locomotion" /
    "common" / "lcm_type" / "lcm"
)
LCM_BUILD_PYTHON_DIR = (
    LCM_ROOT / "build" / "lib" / "python3" / "dist-packages"
)
LCM_SOURCE_PYTHON_DIR = LCM_ROOT / "lcm-python"

sys.path.insert(0, str(MESSAGE_TYPE_DIR))
if LCM_BUILD_PYTHON_DIR.is_dir():
    sys.path.insert(0, str(LCM_BUILD_PYTHON_DIR))
elif LCM_SOURCE_PYTHON_DIR.is_dir():
    sys.path.insert(0, str(LCM_SOURCE_PYTHON_DIR))

try:
    import lcm
except ModuleNotFoundError as exc:
    raise SystemExit(
        f"找不到 Python LCM 绑定。检查 {LCM_BUILD_PYTHON_DIR}，"
        "或先构建 LCM Python 绑定。"
    ) from exc

from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt


LCM_URL = "udpm://239.255.76.67:7671?ttl=255"
CHANNEL = "robot_control_cmd"
RESPONSE_LCM_URL = "udpm://239.255.76.67:7670?ttl=255"
RESPONSE_CHANNEL = "robot_control_response"
MODE_RECOVERY_STAND = 12
MODE_MOTION = 62
# gait_id=88: cyberdog2_user88_FR_lift.toml, the FR ball-kick sequence.
GAIT_FR_LIFT = 88
KICK_COMMAND_SECONDS = 3.2
PUBLISH_INTERVAL_SECONDS = 0.1


def next_life_count(value):
    # life_count 对应 int8_t
    value += 1
    if value > 127:
        value = -128
    return value


def set_command(msg, mode, gait_id, life_count):
    msg.mode = mode
    msg.gait_id = gait_id
    msg.contact = 0x0F
    msg.duration = 0
    msg.value = 0
    msg.life_count = life_count


def publish_for(lc, msg, mode, gait_id, life_count, seconds):
    set_command(msg, mode, gait_id, life_count)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        lc.publish(CHANNEL, msg.encode())
        msg.life_count = next_life_count(msg.life_count)
        time.sleep(PUBLISH_INTERVAL_SECONDS)
    return msg.life_count


def publish_until_interrupted(lc, msg, mode, gait_id, life_count):
    set_command(msg, mode, gait_id, life_count)
    while True:
        lc.publish(CHANNEL, msg.encode())
        msg.life_count = next_life_count(msg.life_count)
        time.sleep(PUBLISH_INTERVAL_SECONDS)
    return msg.life_count


def controller_is_ready(timeout_seconds=3.0):
    response_lc = lcm.LCM(RESPONSE_LCM_URL)
    received = False

    def on_response(channel, data):
        nonlocal received
        robot_control_response_lcmt.decode(data)
        received = True

    response_lc.subscribe(RESPONSE_CHANNEL, on_response)
    deadline = time.monotonic() + timeout_seconds
    while not received and time.monotonic() < deadline:
        response_lc.handle_timeout(100)
    return received


def main():
    print("检查 cyberdog_control 是否在线")
    if not controller_is_ready():
        raise SystemExit(
            "未收到 robot_control_response。请先启动 Gazebo，等待模型加载完成，"
            "再启动 cyberdog_control_launch.py。"
        )

    command_lc = lcm.LCM(LCM_URL)
    msg = robot_control_cmd_lcmt()
    life_count = 1

    print("先发送 Recovery Stand，等待机器人站稳")
    life_count = publish_for(command_lc, msg, MODE_RECOVERY_STAND, 0, life_count, 8.0)

    print("发送 FR 踢球动作: mode=62, gait_id=88")
    # Motion-list durations are halved by the parser, so this 11-step action
    # takes about 2.4 seconds. Stop before it can be triggered a second time.
    life_count = publish_for(
        command_lc, msg, MODE_MOTION, GAIT_FR_LIFT, life_count,
        KICK_COMMAND_SECONDS
    )

    print("踢球完成，持续保持 Recovery Stand；按 Ctrl+C 结束")
    try:
        publish_until_interrupted(
            command_lc, msg, MODE_RECOVERY_STAND, 0, life_count
        )
    except KeyboardInterrupt:
        print("\n停止发送命令；控制器超时后会进入 pure_damper")


if __name__ == "__main__":
    main()
