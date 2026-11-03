#!/usr/bin/env python3
"""Calibrate the RTSP camera with a checkerboard.

Press SPACE to add a frame when the checkerboard is detected, ENTER to run
calibration, R to clear captured frames, and Q/ESC to quit.
"""

import argparse
import json
import os
import threading
import time
from pathlib import Path
from datetime import datetime
from urllib.parse import quote

import cv2
import numpy as np

DEFAULT_STREAM = "rtsp://192.168.31.105:554/Streaming/Channels/101"
DEFAULT_USERNAME = "admin"
DEFAULT_PASSWORD = "DRGK_123456sxt"
DEFAULT_OUTPUT = Path(__file__).resolve().parent / "camera_calibration"
IMAGE_DIR = Path(__file__).resolve().parent / "images"


class RTSPReader:
    """Read RTSP in a daemon thread so GUI/key handling never blocks on read()."""

    def __init__(self, url, reconnect_delay=1.0):
        self.url = url
        self.reconnect_delay = reconnect_delay
        self._lock = threading.Lock()
        self._frame = None
        self._frame_time = 0.0
        self._frame_id = 0
        self._started_at = time.monotonic()
        self._error = "正在连接 RTSP"
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="rtsp-reader", daemon=True)

    def start(self):
        self._thread.start()
        return self

    def _run(self):
        # FFmpeg options are honored by OpenCV builds with FFmpeg support.
        os.environ.setdefault(
            "OPENCV_FFMPEG_CAPTURE_OPTIONS",
            "rtsp_transport;tcp|stimeout;5000000|rw_timeout;5000000|max_delay;500000",
        )
        while not self._stop.is_set():
            with self._lock:
                self._error = "正在连接 RTSP"
            try:
                capture = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
            except Exception as exc:
                with self._lock:
                    self._error = f"RTSP 打开异常: {type(exc).__name__}: {exc}"
                self._stop.wait(self.reconnect_delay)
                continue
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not capture.isOpened():
                with self._lock:
                    self._error = "无法打开 RTSP，正在重连"
                capture.release()
                self._stop.wait(self.reconnect_delay)
                continue
            failures = 0
            while not self._stop.is_set():
                try:
                    ok, frame = capture.read()
                except Exception as exc:
                    with self._lock:
                        self._error = f"RTSP 读取异常: {type(exc).__name__}: {exc}"
                    break
                if ok and frame is not None:
                    with self._lock:
                        self._frame = frame
                        self._frame_time = time.monotonic()
                        self._frame_id += 1
                        self._error = ""
                    failures = 0
                else:
                    failures += 1
                    if failures >= 3:
                        with self._lock:
                            self._error = "RTSP 读取失败，正在重连"
                        break
            capture.release()
            self._stop.wait(self.reconnect_delay)

    def read(self):
        with self._lock:
            if self._frame is None:
                elapsed = time.monotonic() - self._started_at
                status = self._error or "等待 RTSP 视频帧"
                return None, 0, 0.0, f"{status} ({elapsed:.0f}s)"
            return self._frame.copy(), self._frame_id, self._frame_time, self._error

    def close(self):
        self._stop.set()
        # read() is isolated in a daemon thread; never wait indefinitely here.
        self._thread.join(timeout=1.0)


class ImageSequenceReader:
    """Present a directory of still images as a manually advanced frame source."""

    def __init__(self, paths):
        self.paths = paths
        self.index = 0
        self._frame_id = 0
        self._load_current()

    def _load_current(self):
        self._frame = cv2.imread(str(self.paths[self.index]))
        if self._frame is None:
            raise OSError(f"无法读取图片: {self.paths[self.index]}")
        self._frame_id += 1
        self._frame_time = time.monotonic()

    @property
    def current_path(self):
        return self.paths[self.index]

    def read(self):
        return self._frame.copy(), self._frame_id, self._frame_time, str(self.current_path)

    def next(self):
        if self.index + 1 >= len(self.paths):
            return False
        self.index += 1
        self._load_current()
        return True

    def close(self):
        pass


class CornerDetector:
    """Detect corners off the GUI thread, dropping superseded input frames."""

    def __init__(self, pattern, detection_interval=0.2):
        self.pattern = pattern
        self.detection_interval = detection_interval
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._pending = None
        self._result = (False, None, -1, 0.0, None)
        self._thread = threading.Thread(
            target=self._run, name="checkerboard-detector", daemon=True
        )

    def start(self):
        self._thread.start()
        return self

    def submit(self, frame, frame_id, frame_time):
        with self._lock:
            # Keeping only the newest frame prevents detection latency from
            # accumulating when findChessboardCorners is slower than the stream.
            self._pending = (frame, frame_id, frame_time)
            self._wake.set()

    def _run(self):
        while not self._stop.is_set():
            self._wake.wait()
            if self._stop.is_set():
                break
            with self._lock:
                task = self._pending
                self._pending = None
                self._wake.clear()
            if task is None:
                continue
            frame, frame_id, frame_time = task
            found, corners = detect_corners(frame, self.pattern)
            with self._lock:
                self._result = (found, corners, frame_id, frame_time, frame)
            self._stop.wait(self.detection_interval)

    def read(self):
        with self._lock:
            return self._result

    def close(self):
        self._stop.set()
        self._wake.set()
        # OpenCV detection may still be running; never hold up shutdown on it.
        self._thread.join(timeout=1.0)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stream", default=DEFAULT_STREAM, help="RTSP URL")
    parser.add_argument("--username", default=DEFAULT_USERNAME)
    parser.add_argument("--password", default=DEFAULT_PASSWORD)
    # The supplied board is 12 x 9 squares, hence 11 x 8 inner corners.
    parser.add_argument("--corners-cols", type=int, default=11, help="checkerboard inner corners horizontally")
    parser.add_argument("--corners-rows", type=int, default=8, help="checkerboard inner corners vertically")
    parser.add_argument("--square-size", type=float, default=25.0, help="checker square size in millimetres")
    parser.add_argument("--minimum-frames", type=int, default=15, help="minimum views before ENTER")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="output basename")
    parser.add_argument("--images", action="store_true", help="从图片目录逐张分析，不连接 RTSP")
    parser.add_argument("--image-dir", type=Path, default=IMAGE_DIR, help="图片分析目录")
    return parser.parse_args()


def authenticated_url(url, username, password):
    host = url.split("://", 1)[-1].split("/", 1)[0]
    if not username or "@" in host:
        return url
    scheme, remainder = url.split("://", 1)
    return f"{scheme}://{quote(username, safe='')}:{quote(password, safe='')}@{remainder}"


def list_images(image_dir):
    extensions = {".jpg", ".jpeg", ".png", ".bmp", ".webp", ".tif", ".tiff"}
    image_dir = image_dir.expanduser().resolve()
    if not image_dir.is_dir():
        return []
    return sorted(p for p in image_dir.iterdir() if p.is_file() and p.suffix.lower() in extensions)


def detect_corners(frame, pattern):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    scale = min(1.0, 960.0 / max(gray.shape))
    if scale < 1.0:
        search_gray = cv2.resize(
            gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
        )
    else:
        search_gray = gray
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(search_gray, pattern, flags)
    if not found:
        return False, None
    if scale < 1.0:
        corners /= scale
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    return True, cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)


def calibrate(objpoints, imgpoints, image_size):
    rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, image_size, None, None)
    new_matrix, roi = cv2.getOptimalNewCameraMatrix(matrix, distortion, image_size, 1.0, image_size)
    errors = []
    for obj, image, rvec, tvec in zip(objpoints, imgpoints, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, matrix, distortion)
        errors.append(float(cv2.norm(image, projected, cv2.NORM_L2) / len(projected) ** 0.5))
    fx, fy = matrix[0, 0], matrix[1, 1]
    width, height = image_size
    return {
        "rms": float(rms), "camera_matrix": matrix, "distortion": distortion,
        "new_camera_matrix": new_matrix, "roi": tuple(int(v) for v in roi),
        "rvecs": rvecs, "tvecs": tvecs, "view_errors": errors,
        "fov_x_degrees": float(2 * np.degrees(np.arctan(width / (2 * fx)))),
        "fov_y_degrees": float(2 * np.degrees(np.arctan(height / (2 * fy)))),
        "image_size": image_size,
    }


def calibration_summary(result):
    matrix = result["camera_matrix"]
    new_matrix = result["new_camera_matrix"]
    width, height = result["image_size"]
    return {
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "view_count": len(result["rvecs"]),
        "image_width": int(width),
        "image_height": int(height),
        "image_size": [int(width), int(height)],
        "rms": result["rms"],
        "fx": float(matrix[0, 0]),
        "fy": float(matrix[1, 1]),
        "cx": float(matrix[0, 2]),
        "cy": float(matrix[1, 2]),
        "fov_x_degrees": result["fov_x_degrees"],
        "fov_y_degrees": result["fov_y_degrees"],
        "camera_matrix": matrix.tolist(),
        "distortion_coefficients": result["distortion"].ravel().tolist(),
        "new_camera_matrix": new_matrix.tolist(),
        "new_fx": float(new_matrix[0, 0]),
        "new_fy": float(new_matrix[1, 1]),
        "new_cx": float(new_matrix[0, 2]),
        "new_cy": float(new_matrix[1, 2]),
        "roi": list(result["roi"]),
        "view_errors": result["view_errors"],
        "rotation_vectors": [v.ravel().tolist() for v in result["rvecs"]],
        "translation_vectors": [v.ravel().tolist() for v in result["tvecs"]],
    }


def save_results(result, output):
    output = output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    summary = calibration_summary(result)

    npz_path = output.with_suffix(".npz")
    yaml_path = output.with_suffix(".yaml")
    json_path = output.with_suffix(".json")
    npz_tmp = output.with_name(output.name + ".tmp.npz")
    yaml_tmp = output.with_name(output.name + ".tmp.yaml")
    json_tmp = output.with_name(output.name + ".tmp.json")

    try:
        np.savez(
            npz_tmp,
            camera_matrix=result["camera_matrix"],
            distortion_coefficients=result["distortion"],
            new_camera_matrix=result["new_camera_matrix"],
            roi=result["roi"],
            rvecs=np.asarray(result["rvecs"]),
            tvecs=np.asarray(result["tvecs"]),
            view_errors=np.asarray(result["view_errors"]),
            image_size=result["image_size"],
            rms=result["rms"],
            fx=summary["fx"], fy=summary["fy"],
            cx=summary["cx"], cy=summary["cy"],
        )

        fs = cv2.FileStorage(str(yaml_tmp), cv2.FILE_STORAGE_WRITE)
        if not fs.isOpened():
            raise OSError(f"无法创建 YAML 文件: {yaml_tmp}")
        for key in (
            "generated_at", "view_count", "image_width", "image_height", "rms",
            "fx", "fy", "cx", "cy", "fov_x_degrees", "fov_y_degrees",
            "new_fx", "new_fy", "new_cx", "new_cy",
        ):
            fs.write(key, summary[key])
        fs.write("image_size", np.asarray(summary["image_size"], dtype=np.int32))
        fs.write("camera_matrix", result["camera_matrix"])
        fs.write("distortion_coefficients", result["distortion"])
        fs.write("new_camera_matrix", result["new_camera_matrix"])
        fs.write("roi", np.asarray(summary["roi"], dtype=np.int32))
        fs.write("view_errors", np.asarray(summary["view_errors"], dtype=np.float64))
        fs.write("rotation_vectors", np.asarray(summary["rotation_vectors"], dtype=np.float64))
        fs.write("translation_vectors", np.asarray(summary["translation_vectors"], dtype=np.float64))
        fs.release()

        json_tmp.write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        os.replace(npz_tmp, npz_path)
        os.replace(yaml_tmp, yaml_path)
        os.replace(json_tmp, json_path)
    finally:
        for temporary in (npz_tmp, yaml_tmp, json_tmp):
            temporary.unlink(missing_ok=True)


def print_results(result, output):
    matrix = result["camera_matrix"]
    new_matrix = result["new_camera_matrix"]
    print(f"\nCalibration complete ({len(result['rvecs'])} views)")
    print(f"RMS reprojection error: {result['rms']:.6f} px")
    print("Intrinsic parameters (pixels):")
    print(f"  fx = {matrix[0, 0]:.9f}")
    print(f"  fy = {matrix[1, 1]:.9f}")
    print(f"  cx = {matrix[0, 2]:.9f}")
    print(f"  cy = {matrix[1, 2]:.9f}")
    print("Camera matrix:\n", matrix)
    print("Distortion coefficients (k1, k2, p1, p2, k3[, ...]):\n", result["distortion"].ravel())
    print("Optimal camera matrix:\n", new_matrix)
    print("Optimal intrinsic parameters (pixels):")
    print(f"  fx = {new_matrix[0, 0]:.9f}, fy = {new_matrix[1, 1]:.9f}")
    print(f"  cx = {new_matrix[0, 2]:.9f}, cy = {new_matrix[1, 2]:.9f}")
    print("Valid ROI (x, y, width, height):", result["roi"])
    print(f"Horizontal/vertical FOV: {result['fov_x_degrees']:.3f} / {result['fov_y_degrees']:.3f} deg")
    print("Per-view reprojection errors (px):", ", ".join(f"{e:.3f}" for e in result["view_errors"]))
    for index, (rvec, tvec) in enumerate(zip(result["rvecs"], result["tvecs"]), 1):
        print(f"View {index} rotation vector: {rvec.ravel()}")
        print(f"View {index} translation vector (mm): {tvec.ravel()}")
    paths = [output.with_suffix(suffix).resolve() for suffix in (".npz", ".yaml", ".json")]
    print("Saved calibration files:")
    for path in paths:
        print(f"  {path}")


def main():
    args = parse_args()
    if args.corners_cols < 2 or args.corners_rows < 2 or args.square_size <= 0:
        raise SystemExit("棋盘格内角点数必须至少为 2，方格尺寸必须为正数")
    pattern = (args.corners_cols, args.corners_rows)
    cv2.setNumThreads(1)

    if args.images:
        paths = list_images(args.image_dir)
        if not paths:
            raise SystemExit(f"图片目录中没有可分析图片: {args.image_dir.resolve()}")
        reader = ImageSequenceReader(paths)
        print(
            f"图片模式: {len(paths)} 张，目录 {args.image_dir.resolve()}。"
            "SPACE=采集并下一张，N=跳过，ENTER=标定，Q/ESC=退出。",
            flush=True,
        )
    else:
        args.image_dir.mkdir(parents=True, exist_ok=True)
        stream = authenticated_url(args.stream, args.username, args.password)
        reader = RTSPReader(stream).start()
        print(
            f"实时模式: 采集帧将保存至 {args.image_dir.resolve()}。"
            "SPACE=采集，ENTER=标定，R=清空，Q/ESC=退出。",
            flush=True,
        )

    detector = CornerDetector(pattern).start()
    objp = np.zeros((args.corners_cols * args.corners_rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.corners_cols, 0:args.corners_rows].T.reshape(-1, 2)
    objp *= args.square_size
    objpoints, imgpoints, image_size = [], [], None
    window_name = "RTSP Checkerboard Calibration"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 720)
    processed_frame_id = -1
    last_captured_frame_id = -1

    try:
        while True:
            frame, frame_id, frame_time, status = reader.read()
            detected, corners, detected_frame_id, detected_frame_time, detected_frame = detector.read()
            if frame is not None:
                image_size = (frame.shape[1], frame.shape[0])
                stale = not args.images and time.monotonic() - frame_time > 2.0
                if frame_id != processed_frame_id:
                    if not args.images and processed_frame_id == -1:
                        print(
                            f"RTSP 已连接: {image_size[0]}x{image_size[1]}, frame={frame_id}",
                            flush=True,
                        )
                    if args.images or not stale:
                        detector.submit(frame, frame_id, frame_time)
                    processed_frame_id = frame_id

                result_is_current = not args.images or detected_frame_id == frame_id
                found = (
                    detected and result_is_current
                    and time.monotonic() - detected_frame_time <= 2.0 and not stale
                )
                preview = frame
                if found:
                    cv2.drawChessboardCorners(preview, pattern, corners, found)
                if args.images:
                    status_text = (
                        f"{reader.index + 1}/{len(reader.paths)} {reader.current_path.name} | "
                        f"views: {len(objpoints)} | {'FOUND' if found else 'not found'}"
                    )
                else:
                    status_text = (
                        "视频帧停滞，等待恢复" if stale else
                        f"frame: {frame_id} | views: {len(objpoints)} | "
                        f"{'FOUND' if found else 'not found'}"
                    )
                color = (0, 220, 0) if found else (0, 0, 255)
            else:
                found, detected_frame = False, None
                preview = np.zeros((480, 800, 3), dtype=np.uint8)
                status_text = status or "等待 RTSP 视频帧"
                color = (0, 180, 255)

            cv2.putText(preview, status_text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.imshow(window_name, preview)
            key = cv2.waitKey(16 if frame is not None else 30) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r") and not args.images:
                objpoints.clear()
                imgpoints.clear()
                last_captured_frame_id = -1
                print("已清空采集帧")
            elif key == 32 and found and detected_frame_id != last_captured_frame_id:
                if args.images:
                    capture_path = reader.current_path
                else:
                    args.image_dir.mkdir(parents=True, exist_ok=True)
                    capture_path = args.image_dir / datetime.now().strftime(
                        "capture_%Y%m%d_%H%M%S_%f.png"
                    )
                    if detected_frame is None or not cv2.imwrite(str(capture_path), detected_frame):
                        print(f"保存采集画面失败: {capture_path}", flush=True)
                        capture_path = None
                if capture_path is not None:
                    objpoints.append(objp.copy())
                    imgpoints.append(corners.copy())
                    last_captured_frame_id = detected_frame_id
                    print(f"已采集第 {len(objpoints)} 帧: {capture_path}", flush=True)
                    if args.images:
                        if reader.next():
                            processed_frame_id = -1
                        else:
                            print("已到图片列表末尾，按 Enter 开始标定或 Q 退出", flush=True)
            elif key == ord("n") and args.images:
                if reader.next():
                    processed_frame_id = -1
                else:
                    print("已到图片列表末尾", flush=True)
            elif key in (13, 10) and len(objpoints) >= args.minimum_frames:
                print("正在计算标定结果...", flush=True)
                result = calibrate(objpoints, imgpoints, image_size)
                save_results(result, args.output)
                print_results(result, args.output)
                break
            elif key in (13, 10):
                print(f"至少需要 {args.minimum_frames} 帧，当前 {len(objpoints)} 帧")
    finally:
        detector.close()
        reader.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
