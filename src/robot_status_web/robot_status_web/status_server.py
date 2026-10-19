#!/usr/bin/env python3
"""ROS 2 health monitor and FastAPI dashboard for multiple CyberDogs."""

import asyncio
from collections import deque
from datetime import datetime, timezone
import json
import threading
import time
from pathlib import Path

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry
from protocol.srv import MotionResultCmd
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
import uvicorn


STATUS_KEYS = ("odom_slam", "apriltag", "odom_global", "global_vio")
LIFECYCLE_NODES = ("camera", "vinslocalization")
LIFECYCLE_PATHS = {"camera": "camera/camera", "vinslocalization": "vinslocalization"}


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


class RobotStatusMonitor(Node):
    def __init__(self):
        super().__init__("robot_status_web")
        self.robot_namespaces = self.declare_parameter(
            "robot_namespaces", [f"cyberdog_{i}" for i in range(1, 11)]
        ).value
        self.bind_host = self.declare_parameter("bind_host", "0.0.0.0").value
        self.bind_port = self.declare_parameter("bind_port", 8080).value
        self.status_period = self.declare_parameter("status_publish_period_sec", 1.0).value
        self.lifecycle_poll_period = self.declare_parameter(
            "lifecycle_poll_period_sec", 5.0
        ).value
        self.timeouts = {
            "odom_slam": self.declare_parameter("odom_slam_timeout_sec", 2.0).value,
            "camera": self.declare_parameter("camera_timeout_sec", 3.0).value,
            "apriltag": self.declare_parameter("apriltag_timeout_sec", 5.0).value,
            "odom_global": self.declare_parameter("odom_global_timeout_sec", 3.0).value,
            "global_vio": self.declare_parameter("global_vio_timeout_sec", 3.0).value,
        }
        self.lock = threading.RLock()
        self.robots = {}
        self.lifecycle_clients = {}
        self.frame_service_clients = {}
        self.motion_result_clients = {}
        self._subscriptions = []
        self._legacy_apriltag_assigned = set()
        self._topic_aliases = set()
        self._published_json = "{}"
        self._websockets = set()
        self._loop = None
        for raw_namespace in self.robot_namespaces:
            namespace = str(raw_namespace).strip("/")
            if not namespace:
                continue
            self.robots[namespace] = {
                "topics": {key: {"state": "unknown", "hz": 0.0,
                                  "last_seen": None, "age_sec": None,
                                  "detection_count": 0 if key == "apriltag" else None}
                           for key in STATUS_KEYS},
                "lifecycle": {name: {"state": "unknown", "detail": "waiting"}
                              for name in LIFECYCLE_NODES},
            }
            self._subscribe(namespace, "odom_slam", Odometry,
                            f"/{namespace}/odom_slam")
            self._subscribe(namespace, "apriltag", AprilTagDetectionArray,
                            f"/{namespace}/apriltag/detections")
            self._subscribe(namespace, "odom_global", Odometry,
                            f"/{namespace}/odom_global")
            self._subscribe(namespace, "global_vio", Odometry,
                            f"/global_vio/{namespace}/odom")
            for node_name in LIFECYCLE_NODES:
                base = f"/{namespace}/{LIFECYCLE_PATHS[node_name]}"
                self.lifecycle_clients[(namespace, node_name)] = (
                    self.create_client(GetState, base + "/get_state"),
                    self.create_client(ChangeState, base + "/change_state"),
                )
            self.frame_service_clients[namespace] = self.create_client(
                SetBool, f"/{namespace}/camera/realsense_frame_service")
            self.motion_result_clients[namespace] = self.create_client(
                MotionResultCmd, f"/{namespace}/motion_result_cmd")
        self.create_timer(self.status_period, self._publish_status)
        self.create_timer(self.lifecycle_poll_period, self._poll_lifecycle)
        self.create_timer(2.0, self._discover_legacy_apriltag)
        self.create_timer(2.0, self._discover_topic_aliases)
        self.get_logger().info(
            f"Monitoring {len(self.robots)} robots; dashboard http://{self.bind_host}:{self.bind_port}"
        )

    def _subscribe(self, namespace, key, message_type, topic):
        # Measure the rate from wall-clock receive events. This matches
        # ``ros2 topic hz`` and remains correct when message header clocks are
        # simulated, reset, or not monotonic across lifecycle restarts.
        received = deque()

        def callback(message):
            now = time.monotonic()
            received.append(now)
            while received and now - received[0] > 10.0:
                received.popleft()
            span = received[-1] - received[0] if len(received) > 1 else 0.0
            hz = (len(received) - 1) / span if span > 0.0 else 0.0
            with self.lock:
                entry = self.robots[namespace]["topics"][key]
                entry.update(state="healthy", hz=round(hz, 2),
                             last_seen=utc_now(), age_sec=0.0,
                             _last_monotonic=now)
                if key == "apriltag":
                    entry["detection_count"] = len(message.detections)

        qos = qos_profile_sensor_data
        if key in ("odom_slam", "odom_global"):
            # VINS is RELIABLE while odom_transform is BEST_EFFORT. Keep the
            # publisher's reliability and a deep queue for WLAN scheduling
            # jitter without making either endpoint incompatible.
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=100,
                reliability=(ReliabilityPolicy.RELIABLE if key == "odom_slam"
                             else ReliabilityPolicy.BEST_EFFORT))
        self._subscriptions.append(self.create_subscription(
            message_type, topic, callback, qos))

    def _discover_legacy_apriltag(self):
        # Old launches publish a global /detections topic. Attribute it only to
        # the robot camera subscribed by that detector node, never to all robots.
        if not self.get_publishers_info_by_topic("/detections"):
            return
        for namespace in self.robots:
            if namespace in self._legacy_apriltag_assigned:
                continue
            camera_topic = f"/{namespace}/camera/infra1/image_rect_raw"
            endpoints = self.get_subscriptions_info_by_topic(camera_topic)
            if not any(endpoint.node_name == "apriltag" for endpoint in endpoints):
                continue
            self._subscribe(namespace, "apriltag", AprilTagDetectionArray, "/detections")
            self._legacy_apriltag_assigned.add(namespace)

    def _discover_topic_aliases(self):
        """Attach to deployments that omit the namespace remap.

        Bind an alias only when its publisher belongs to the matching node, so
        one robot's data is never reported for every robot.
        """
        candidates = (
            ("apriltag", "/detections", "apriltag", AprilTagDetectionArray),
            ("odom_global", "/odom_global", "odom_transform", Odometry),
        )
        for namespace in self.robots:
            for key, topic, node_name, message_type in candidates:
                marker = (namespace, key, topic)
                if marker in self._topic_aliases:
                    continue
                endpoints = self.get_publishers_info_by_topic(topic)
                if not any(endpoint.node_name == node_name for endpoint in endpoints):
                    continue
                self._subscribe(namespace, key, message_type, topic)
                self._topic_aliases.add(marker)

    def _poll_lifecycle(self):
        for (namespace, node_name), (get_client, _change_client) in self.lifecycle_clients.items():
            if not get_client.service_is_ready():
                with self.lock:
                    self.robots[namespace]["lifecycle"][node_name] = {
                        "state": "unavailable", "detail": "get_state service unavailable"
                    }
                continue
            future = get_client.call_async(GetState.Request())

            def update_state(done, ns=namespace, name=node_name):
                try:
                    label = done.result().current_state.label.lower()
                    state = "active" if label == "active" else label
                    detail = label
                except Exception as exc:  # service may disappear during discovery
                    state, detail = "error", str(exc)
                with self.lock:
                    self.robots[ns]["lifecycle"][name] = {
                        "state": state, "detail": detail
                    }
            future.add_done_callback(update_state)

    def _publish_status(self):
        now = time.monotonic()
        with self.lock:
            for namespace, robot in self.robots.items():
                topics = robot["topics"]
                topic_names = {
                    "odom_slam": f"/{namespace}/odom_slam",
                    "apriltag": f"/{namespace}/apriltag/detections",
                    "odom_global": f"/{namespace}/odom_global",
                    "global_vio": f"/global_vio/{namespace}/odom",
                }
                dependencies_active = (
                    robot["lifecycle"].get("camera", {}).get("state") == "active"
                    and robot["lifecycle"].get("vinslocalization", {}).get("state") == "active"
                )
                for key, entry in topics.items():
                    # Tag/global odometry processes may stay alive while their
                    # camera/VINS inputs are inactive; expose those streams as
                    # unavailable until both lifecycle nodes are ACTIVE.
                    if key in ("apriltag", "odom_global") and not dependencies_active:
                        entry.update(state="unknown", hz=0.0, age_sec=None)
                        entry.pop("_last_monotonic", None)
                        if key == "apriltag":
                            entry["detection_count"] = 0
                        continue
                    publishers = self.get_publishers_info_by_topic(topic_names[key])
                    if not publishers:
                        entry.update(state="unknown", hz=0.0, age_sec=None)
                        entry.pop("_last_monotonic", None)
                        if key == "apriltag":
                            entry["detection_count"] = 0
                        continue
                    entry["state"] = "healthy"
                    last_seen = entry.get("_last_monotonic")
                    entry["age_sec"] = round(max(0.0, now - last_seen), 2) if last_seen else None
            public_robots = {}
            for namespace, robot in self.robots.items():
                public_robots[namespace] = {
                    "topics": {
                        key: {name: value for name, value in entry.items()
                              if not name.startswith("_")}
                        for key, entry in robot["topics"].items()
                    },
                    "lifecycle": robot["lifecycle"],
                }
            payload = {"updated_at": utc_now(), "robots": public_robots}
            self._published_json = json.dumps(payload)
            loop = self._loop
        if loop and loop.is_running():
            loop.call_soon_threadsafe(self._broadcast, self._published_json)

    def _broadcast(self, payload):
        for websocket in tuple(self._websockets):
            asyncio.create_task(self._send(websocket, payload))

    async def _send(self, websocket, payload):
        try:
            await websocket.send_text(payload)
        except Exception:
            self._websockets.discard(websocket)

    def snapshot(self):
        self._publish_status()
        with self.lock:
            return json.loads(self._published_json)

    def activate_robot(self, namespace):
        """Start one robot in the documented camera -> frame service -> VINS order."""
        if namespace not in self.robots:
            raise KeyError(namespace)

        results = {}
        camera_ok = self._ensure_lifecycle_active(namespace, "camera", results)
        if not camera_ok:
            results["realsense_frame_service"] = {
                "ok": False, "detail": "skipped because camera/camera is not active"
            }
            results["vinslocalization"] = {
                "ok": False, "detail": "skipped because camera startup failed"
            }
            self._poll_lifecycle()
            return {"robot": namespace, "nodes": results,
                    "ok": all(value["ok"] for value in results.values())}

        frame_client = self.frame_service_clients[namespace]
        if not frame_client.wait_for_service(timeout_sec=15.0):
            response = None
            frame_detail = "service unavailable after 15 seconds"
        else:
            response = self._call_service(
                frame_client, SetBool.Request(data=True), timeout=15.0)
            if response is None:
                # The enable request is idempotent; retry once if DDS loses the response.
                response = self._call_service(
                    frame_client, SetBool.Request(data=True), timeout=15.0)
            if response is None:
                frame_detail = "no service response after retry"
            elif not response.success:
                frame_detail = response.message or "service returned success=false"
            else:
                frame_detail = response.message or "enabled"
        frame_ok = bool(response and response.success)
        results["realsense_frame_service"] = {
            "ok": frame_ok,
            "detail": frame_detail,
        }
        # Attempt VINS even if the camera service returned an error or timed
        # out. The service request has already been issued; a lost response
        # must not silently prevent the downstream lifecycle transition.
        vins_ok = self._ensure_lifecycle_active(namespace, "vinslocalization", results)
        self._poll_lifecycle()
        return {"robot": namespace, "nodes": results,
                "ok": camera_ok and frame_ok and vins_ok}

    def deactivate_robot(self, namespace):
        """Stop VINS first, then camera, using deactivate -> cleanup transitions."""
        if namespace not in self.robots:
            raise KeyError(namespace)
        results = {}
        vins_ok = self._cleanup_lifecycle(namespace, "vinslocalization", results)
        camera_ok = self._cleanup_lifecycle(namespace, "camera", results)
        self._poll_lifecycle()
        return {"robot": namespace, "nodes": results,
                "ok": vins_ok and camera_ok}

    def _motion_result(self, namespace, motion_id, action_name):
        if namespace not in self.robots:
            raise KeyError(namespace)
        client = self.motion_result_clients[namespace]
        if not client.wait_for_service(timeout_sec=5.0):
            return {"robot": namespace, "ok": False,
                    "detail": "motion_result_cmd service unavailable"}

        request = MotionResultCmd.Request()
        request.motion_id = motion_id
        request.cmd_source = MotionResultCmd.Request.APP
        request.vel_des = [0.0, 0.0, 0.0]
        request.rpy_des = [0.0, 0.0, 0.0]
        request.pos_des = [0.0, 0.0, 0.0]
        request.acc_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        request.ctrl_point = [0.0, 0.0, 0.0]
        request.foot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        request.step_height = [0.0, 0.0]
        request.duration = 0
        request.value = 0
        request.contact = 0
        response = self._call_service(client, request, timeout=8.0)
        if response is None:
            return {"robot": namespace, "ok": False,
                    "detail": f"{action_name}指令调用超时"}
        if not response.result:
            code = getattr(response, "code", "unknown")
            details = {3003: "motion manager 尚未 Active", 3010: f"{action_name}参数无效",
                       3011: "motion manager 正在执行其他动作", 3013: "低电量保护"}
            return {"robot": namespace, "ok": False, "code": code,
                    "detail": f"{action_name}未执行：{details.get(code, '服务拒绝请求')} (code={code})"}
        return {"robot": namespace, "ok": True, "code": getattr(response, "code", 0),
                "detail": f"{action_name}执行成功"}

    def stand_robot(self, namespace):
        """Request recovery stand (motion id 111)."""
        return self._motion_result(namespace, 111, "站立")

    def down_robot(self, namespace):
        """Request get-down (motion id 101)."""
        return self._motion_result(namespace, 101, "趴下")

    def _cleanup_lifecycle(self, namespace, node_name, results):
        get_client, change_client = self.lifecycle_clients[(namespace, node_name)]
        if not get_client.wait_for_service(timeout_sec=10.0):
            results[node_name] = {"ok": False, "detail": "get_state service unavailable"}
            return False
        current = self._call_service(get_client, GetState.Request())
        if current is None:
            results[node_name] = {"ok": False, "detail": "get_state timed out"}
            return False
        label = current.current_state.label.lower()
        if label == "unconfigured":
            results[node_name] = {"ok": True, "detail": "already unconfigured"}
            return True
        if label == "active":
            deactivated, detail = self._transition(change_client, Transition.TRANSITION_DEACTIVATE, get_client)
            if not deactivated:
                results[node_name] = {"ok": False, "detail": f"deactivate failed: {detail}"}
                return False
            label = "inactive"
        if label == "inactive":
            cleaned, detail = self._transition(change_client, Transition.TRANSITION_CLEANUP, get_client)
            results[node_name] = {
                "ok": cleaned,
                "detail": "unconfigured" if cleaned else f"cleanup failed: {detail}",
            }
            return cleaned
        results[node_name] = {"ok": False, "detail": f"cannot close from {label}"}
        return False

    def _ensure_lifecycle_active(self, namespace, node_name, results):
        get_client, change_client = self.lifecycle_clients[(namespace, node_name)]
        result_key = node_name
        if not get_client.wait_for_service(timeout_sec=10.0):
            results[result_key] = {"ok": False, "detail": "get_state service unavailable"}
            return False
        current = self._call_service(get_client, GetState.Request())
        if current is None:
            results[result_key] = {"ok": False, "detail": "get_state timed out"}
            return False

        label = current.current_state.label.lower()
        if label == "active":
            results[result_key] = {"ok": True, "detail": "already active"}
            return True
        if label == "inactive":
            # A partially started VINS node can remain inactive. Reset it to
            # unconfigured before running the normal configure/activate flow.
            cleaned, detail = self._transition(change_client, Transition.TRANSITION_CLEANUP, get_client)
            if not cleaned:
                results[result_key] = {"ok": False, "detail": f"cleanup before configure failed: {detail}"}
                return False
            label = "unconfigured"
        if label == "unconfigured":
            configured, detail = self._transition(change_client, Transition.TRANSITION_CONFIGURE, get_client)
            if not configured:
                results[result_key] = {"ok": False, "detail": f"configure failed: {detail}"}
                return False
            label = "inactive"
        if label != "inactive":
            results[result_key] = {"ok": False, "detail": f"cannot activate from {label}"}
            return False
        activated, detail = self._transition(change_client, Transition.TRANSITION_ACTIVATE, get_client)
        results[result_key] = {
            "ok": activated,
            "detail": "active" if activated else f"activate failed: {detail}",
        }
        return activated

    @staticmethod
    def _call_service(client, request, timeout=10.0):
        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        if not event.wait(timeout):
            return None
        try:
            return future.result()
        except Exception:
            return None

    def _transition(self, client, transition_id, state_client=None):
        if not client.wait_for_service(timeout_sec=15.0):
            return False, "change_state service unavailable"
        request = ChangeState.Request()
        request.transition.id = transition_id
        response = self._call_service(client, request, timeout=30.0)
        if response and response.success:
            return True, "success"

        # Some remote lifecycle servers report a rejected/late response while
        # completing the transition. Trust the observed state, not only the reply.
        expected_state = {
            Transition.TRANSITION_CONFIGURE: "inactive",
            Transition.TRANSITION_CLEANUP: "unconfigured",
            Transition.TRANSITION_ACTIVATE: "active",
            Transition.TRANSITION_DEACTIVATE: "inactive",
        }.get(transition_id)
        if state_client and expected_state:
            for _ in range(6):
                state = self._call_service(state_client, GetState.Request(), timeout=5.0)
                if state and state.current_state.label.lower() == expected_state:
                    return True, f"{expected_state} state confirmed"
                time.sleep(0.5)
        if response is None:
            return False, "change_state timed out; target state not confirmed"
        return False, "transition response unsuccessful; target state not confirmed"


def create_app(monitor):
    app = FastAPI(title="CyberDog Fleet Status", docs_url=None, redoc_url=None)
    web_dir = Path(get_package_share_directory("robot_status_web")) / "web"

    @app.get("/")
    async def index():
        return FileResponse(web_dir / "index.html")

    @app.get("/api/status")
    async def status():
        return monitor.snapshot()

    @app.post("/api/robots/{namespace}/activate")
    async def activate(namespace: str):
        if namespace not in monitor.robots:
            raise HTTPException(status_code=404, detail="unknown robot")
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(None, monitor.activate_robot, namespace)

    @app.post("/api/robots/{namespace}/deactivate")
    async def deactivate(namespace: str):
        if namespace not in monitor.robots:
            raise HTTPException(status_code=404, detail="unknown robot")
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(None, monitor.deactivate_robot, namespace)

    @app.post("/api/robots/{namespace}/down")
    async def down(namespace: str):
        if namespace not in monitor.robots:
            raise HTTPException(status_code=404, detail="unknown robot")
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(None, monitor.down_robot, namespace)

    @app.post("/api/robots/{namespace}/stand")
    async def stand(namespace: str):
        if namespace not in monitor.robots:
            raise HTTPException(status_code=404, detail="unknown robot")
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(None, monitor.stand_robot, namespace)

    @app.post("/api/activate-all")
    async def activate_all():
        loop = asyncio.get_running_loop()
        results = await asyncio.gather(*[
            loop.run_in_executor(None, monitor.activate_robot, namespace)
            for namespace in monitor.robots
        ])
        return {"ok": all(item["ok"] for item in results), "results": results}

    @app.post("/api/down-all")
    async def down_all():
        loop = asyncio.get_running_loop()
        results = await asyncio.gather(*[
            loop.run_in_executor(None, monitor.down_robot, namespace)
            for namespace in monitor.robots
        ])
        return {"ok": all(item["ok"] for item in results), "results": results}

    @app.post("/api/stand-all")
    async def stand_all():
        loop = asyncio.get_running_loop()
        results = await asyncio.gather(*[
            loop.run_in_executor(None, monitor.stand_robot, namespace)
            for namespace in monitor.robots
        ])
        return {"ok": all(item["ok"] for item in results), "results": results}

    @app.websocket("/ws")
    async def websocket_status(websocket: WebSocket):
        await websocket.accept()
        monitor._loop = asyncio.get_running_loop()
        monitor._websockets.add(websocket)
        try:
            await websocket.send_text(json.dumps(monitor.snapshot()))
            while True:
                await websocket.receive_text()
        except WebSocketDisconnect:
            monitor._websockets.discard(websocket)
        finally:
            if not monitor._websockets:
                monitor._loop = None

    return app


def main():
    rclpy.init()
    monitor = RobotStatusMonitor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(monitor)
    ros_thread = threading.Thread(target=executor.spin, name="ros-executor", daemon=True)
    ros_thread.start()
    server = uvicorn.Server(uvicorn.Config(
        create_app(monitor), host=monitor.bind_host, port=monitor.bind_port,
        log_level="info", lifespan="off",
    ))
    try:
        server.run()
    finally:
        executor.shutdown(timeout_sec=3.0)
        ros_thread.join(timeout=3.0)
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
