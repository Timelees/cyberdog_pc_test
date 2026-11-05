#!/usr/bin/env python3

import argparse
import getpass
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROBOT_HOST = "192.168.31.91"
ROS_SETUP = "/opt/ros2/cyberdog/setup.bash"
ROS_DOMAIN_ID = "42"
RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp"
CYCLONEDDS_URI = "file:///etc/mi/cyclonedds.xml"
RESULT_SERVICE = "/cyberdog_1/motion_result_cmd"
QUEUE_SERVICE = "/cyberdog_1/motion_queue_cmd"
STATUS_TOPIC = "/cyberdog_1/motion_status"
RECOVERY_STAND = 111
KICK_MODE = 62
KICK_GAIT = 88


def parse_args():
    parser = argparse.ArgumentParser(
        description="通过机器人 motion_manager 执行一次右前腿踢球动作。"
    )
    parser.add_argument("--robot-host", default=ROBOT_HOST)
    parser.add_argument("--ssh-user", default="mi")
    parser.add_argument(
        "--ssh-password", default=os.environ.get("CYBERDOG_SSH_PASSWORD")
    )
    parser.add_argument("--yes", action="store_true", help="跳过安全确认")
    parser.add_argument("--remote-worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--stand-only", action="store_true", help=argparse.SUPPRESS)
    return parser.parse_args()


def call_result_motion(node, motion_id, timeout):
    import rclpy
    from protocol.srv import MotionResultCmd

    client = node.create_client(MotionResultCmd, RESULT_SERVICE)
    if not client.wait_for_service(timeout_sec=8.0):
        raise RuntimeError(f"服务不可用：{RESULT_SERVICE}")
    request = MotionResultCmd.Request()
    request.motion_id = motion_id
    request.cmd_source = MotionResultCmd.Request.APP
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        raise RuntimeError(f"动作 {motion_id} 等待响应超时")
    response = future.result()
    if not response.result:
        raise RuntimeError(
            f"动作 {motion_id} 失败：code={response.code}, "
            f"current_motion_id={response.motion_id}"
        )
    return response


def submit_kick(node):
    import rclpy
    from protocol.msg import MotionCustomCmd
    from protocol.srv import MotionQueueCustomCmd

    client = node.create_client(MotionQueueCustomCmd, QUEUE_SERVICE)
    if not client.wait_for_service(timeout_sec=8.0):
        raise RuntimeError(f"服务不可用：{QUEUE_SERVICE}")
    command = MotionCustomCmd()
    command.mode = KICK_MODE
    command.gait_id = KICK_GAIT
    command.contact = 0x0F
    command.vel_des = [0.0] * 3
    command.rpy_des = [0.0] * 3
    command.pos_des = [0.0] * 3
    command.acc_des = [0.0] * 6
    command.ctrl_point = [0.0] * 3
    command.foot_pose = [0.0] * 6
    command.step_height = [0.0] * 2
    request = MotionQueueCustomCmd.Request()
    request.cmds = [command]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
    if not future.done() or future.result() is None:
        raise RuntimeError("踢球命令提交超时")
    response = future.result()
    if not response.result:
        raise RuntimeError(f"motion_manager 拒绝踢球队列：code={response.code}")


def wait_for_recovery_status(node, timeout_seconds):
    import rclpy
    from protocol.msg import MotionStatus

    state = {"ready": False, "last": None}

    def on_status(status):
        identity = (
            status.motion_id,
            status.order_process_bar,
            status.switch_status,
        )
        if identity != state["last"]:
            print(
                "[机器人端] 控制状态："
                f"motion_id={status.motion_id}, "
                f"progress={status.order_process_bar}%, "
                f"switch={status.switch_status}",
                flush=True,
            )
            state["last"] = identity
        if status.motion_id == RECOVERY_STAND and status.order_process_bar >= 95:
            state["ready"] = True

    subscription = node.create_subscription(MotionStatus, STATUS_TOPIC, on_status, 10)
    try:
        deadline = node.get_clock().now().nanoseconds + int(timeout_seconds * 1e9)
        while node.get_clock().now().nanoseconds < deadline and not state["ready"]:
            rclpy.spin_once(node, timeout_sec=0.1)
        return state["ready"]
    finally:
        node.destroy_subscription(subscription)


def execute_and_monitor_kick(node):
    import rclpy
    from protocol.msg import MotionStatus

    state = {"active": False, "complete": False, "problem": None, "last": None}

    def on_status(status):
        identity = (status.motion_id, status.order_process_bar, status.switch_status)
        if identity != state["last"]:
            print(
                "[机器人端] 控制状态："
                f"motion_id={status.motion_id}, progress={status.order_process_bar}%, "
                f"switch={status.switch_status}",
                flush=True,
            )
            state["last"] = identity
        if status.switch_status not in (MotionStatus.NORMAL, MotionStatus.TRANSITIONING):
            state["problem"] = (
                f"控制器状态异常：switch_status={status.switch_status}, "
                f"motion_id={status.motion_id}"
            )
        errors = [
            (index, value)
            for index, value in enumerate(status.motor_error)
            if value & 0x3FFFFFFF
        ]
        if errors:
            state["problem"] = f"电机异常（索引, 错误位）：{errors}"
        if status.motion_id != RECOVERY_STAND:
            state["active"] = True
        elif state["active"] and status.order_process_bar >= 95:
            state["complete"] = True

    subscription = node.create_subscription(MotionStatus, STATUS_TOPIC, on_status, 10)
    try:
        print("[机器人端] 提交右前腿踢球动作 62/88", flush=True)
        submit_kick(node)
        deadline = node.get_clock().now().nanoseconds + int(10.0 * 1e9)
        while node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if state["problem"]:
                raise RuntimeError(state["problem"])
            if state["complete"]:
                print("[机器人端] 控制器已报告踢球序列完成", flush=True)
                return
        if not state["active"]:
            raise RuntimeError("队列服务已返回，但控制器状态未离开站立，踢球动作没有启动")
        raise RuntimeError("控制器进入了踢球动作，但 10 秒内未报告恢复站立")
    finally:
        node.destroy_subscription(subscription)


def remote_worker(stand_only):
    try:
        import rclpy
    except ModuleNotFoundError as exc:
        raise SystemExit(f"ROS2 环境未加载，请检查 {ROS_SETUP}") from exc
    rclpy.init()
    node = rclpy.create_node("fr_kick_test")
    kick_submitted = False
    kick_completed = False
    try:
        print("[机器人端] 请求 Recovery Stand，并等待动作完成", flush=True)
        response = call_result_motion(node, RECOVERY_STAND, 25.0)
        print(
            f"[机器人端] 已站立：motion_id={response.motion_id}, code={response.code}",
            flush=True,
        )
        if stand_only:
            return
        if not wait_for_recovery_status(node, 3.0):
            raise RuntimeError("站立服务已返回成功，但状态话题未确认 motion_id=111")
        kick_submitted = True
        execute_and_monitor_kick(node)
        kick_completed = True
    finally:
        if kick_submitted and not kick_completed:
            try:
                print("[机器人端] 恢复站立", flush=True)
                call_result_motion(node, RECOVERY_STAND, 25.0)
            except Exception as exc:
                print(f"[机器人端] 恢复站立失败：{exc}", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()


def ssh_base(args, control_path=None, batch_mode=False):
    control_options = []
    if control_path is not None:
        control_options = ["-o", f"ControlPath={control_path}"]
    if batch_mode:
        control_options.extend(["-o", "BatchMode=yes"])
    return [
        "ssh", "-o", "ConnectTimeout=8", "-o", "StrictHostKeyChecking=accept-new",
        "-o", "NumberOfPasswordPrompts=1", *control_options,
        f"{args.ssh_user}@{args.robot_host}",
    ]


def run(command, env, check=True):
    return subprocess.run(command, check=check, env=env, stdin=subprocess.DEVNULL)


def remote_command(script, stand_only=False):
    option = " --stand-only" if stand_only else ""
    return (
        f"source {ROS_SETUP} >/dev/null 2>&1 && "
        f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID} "
        f"RMW_IMPLEMENTATION={RMW_IMPLEMENTATION} "
        f"CYCLONEDDS_URI={CYCLONEDDS_URI} && "
        f"python3 -u {script} --remote-worker{option}"
    )


def stop_remote_worker(ssh, script, env):
    pattern = f"[p]ython3 -u {script} --remote-worker"
    run(ssh + [f"pkill -f '{pattern}' >/dev/null 2>&1 || true"], env, check=False)


def run_on_robot(args):
    if not shutil.which("ssh") or not shutil.which("scp"):
        raise SystemExit("缺少标准 ssh/scp 命令。")
    password = args.ssh_password or getpass.getpass("机器人 Jetson SSH 密码：")
    remote_dir = f"/tmp/cyberdog_fr_kick_{os.getpid()}"
    source = Path(__file__).resolve()
    remote_script = f"{remote_dir}/{source.name}"
    target = f"{args.ssh_user}@{args.robot_host}"
    scp = [
        "scp", "-o", "ConnectTimeout=8", "-o", "StrictHostKeyChecking=accept-new",
        "-o", "NumberOfPasswordPrompts=1", "-o", "BatchMode=yes",
    ]
    with tempfile.TemporaryDirectory(prefix="cyberdog_askpass_") as temp_dir:
        askpass = Path(temp_dir) / "askpass.sh"
        askpass.write_text('#!/bin/sh\nprintf "%s\\n" "$CYBERDOG_ASKPASS_PASSWORD"\n')
        askpass.chmod(0o700)
        env = os.environ.copy()
        env.update({
            "SSH_ASKPASS": str(askpass), "SSH_ASKPASS_REQUIRE": "force",
            "DISPLAY": env.get("DISPLAY", ":0"),
            "CYBERDOG_ASKPASS_PASSWORD": password,
        })
        control_path = str(Path(temp_dir) / "ssh-control")
        master_ssh = ssh_base(args, control_path)
        ssh = ssh_base(args, control_path, batch_mode=True)
        scp.extend(["-o", f"ControlPath={control_path}"])
        master_started = False
        try:
            run(
                master_ssh[:-1] +
                ["-M", "-o", "ControlPersist=60", "-N", "-f", master_ssh[-1]],
                env,
            )
            master_started = True
            run(ssh + [f"mkdir -p {remote_dir}"], env)
            run(scp + [str(source), f"{target}:{remote_dir}/"], env)
            run(ssh + [remote_command(remote_script)], env)
        except subprocess.CalledProcessError as exc:
            raise SystemExit(f"实机测试执行失败，退出码：{exc.returncode}") from exc
        except KeyboardInterrupt:
            print("\n正在中断踢球并请求机器人恢复站立...", flush=True)
            stop_remote_worker(ssh, remote_script, env)
            run(ssh + [remote_command(remote_script, True)], env, check=False)
        finally:
            if master_started:
                stop_remote_worker(ssh, remote_script, env)
                run(ssh + [f"rm -rf {remote_dir}"], env, check=False)
                run(ssh[:-1] + ["-O", "exit", ssh[-1]], env, check=False)


def main():
    args = parse_args()
    if args.remote_worker:
        remote_worker(args.stand_only)
        return
    if not args.yes:
        answer = input("确认周围无人、急停可用且右前腿前方无障碍物？输入 YES 继续：")
        if answer != "YES":
            raise SystemExit("已取消实机踢球测试。")
    print(f"通过 {args.ssh_user}@{args.robot_host} 调用机器人 motion_manager")
    run_on_robot(args)


if __name__ == "__main__":
    main()
