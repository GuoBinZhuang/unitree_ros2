#!/usr/bin/env python3
"""Bridge teleimager ZMQ camera frames into ROS2 image topics."""

from __future__ import annotations

import argparse
import base64
import json
import os
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import BinaryIO


CAMERA_KEYS = {
    "head": ("head_camera", "teleimager_head_camera"),
    "left_wrist": ("left_wrist_camera", "teleimager_left_wrist_camera"),
    "right_wrist": ("right_wrist_camera", "teleimager_right_wrist_camera"),
}


@dataclass(frozen=True)
class CameraSource:
    name: str
    config_key: str
    frame_id: str


def normalize_topic_prefix(prefix: str) -> str:
    prefix = prefix.strip() or "/teleimager"
    if not prefix.startswith("/"):
        prefix = f"/{prefix}"
    return prefix.rstrip("/") or "/teleimager"


def parse_cameras(value: str) -> list[CameraSource]:
    sources: list[CameraSource] = []
    for item in value.split(","):
        camera_name = item.strip()
        if not camera_name:
            continue
        if camera_name not in CAMERA_KEYS:
            choices = ", ".join(CAMERA_KEYS)
            raise ValueError(f"Unknown camera '{camera_name}'. Expected one of: {choices}")
        config_key, frame_id = CAMERA_KEYS[camera_name]
        sources.append(CameraSource(camera_name, config_key, frame_id))

    if not sources:
        raise ValueError("At least one camera must be selected.")
    return sources


def build_raw_image(frame, frame_id: str, stamp):
    from sensor_msgs.msg import Image

    height, width = frame.shape[:2]
    channels = 1 if len(frame.shape) == 2 else frame.shape[2]

    image = Image()
    image.header.stamp = stamp
    image.header.frame_id = frame_id
    image.height = height
    image.width = width
    image.encoding = "mono8" if channels == 1 else "bgr8"
    image.is_bigendian = False
    image.step = width * channels
    image.data = frame.tobytes()
    return image


def build_compressed_image(jpg: bytes, frame_id: str, stamp):
    from sensor_msgs.msg import CompressedImage

    image = CompressedImage()
    image.header.stamp = stamp
    image.header.frame_id = frame_id
    image.format = "jpeg"
    image.data = jpg
    return image


def run_zmq_helper(args: argparse.Namespace) -> None:
    import zmq

    sources = parse_cameras(args.cameras)
    context = zmq.Context()
    request_socket = context.socket(zmq.REQ)
    request_socket.connect(f"tcp://{args.host}:{args.request_port}")
    request_socket.send_json({"cmd": "get_cam_config"})
    cam_config = request_socket.recv_json()
    request_socket.close()

    poller = zmq.Poller()
    sockets: dict[object, CameraSource] = {}
    for source in sources:
        camera_config = cam_config.get(source.config_key, {})
        if not camera_config.get("enable_zmq", False):
            print(
                json.dumps({"type": "warning", "message": f"Skipping {source.name}: ZMQ disabled"}),
                flush=True,
            )
            continue

        port = camera_config.get("zmq_port")
        socket = context.socket(zmq.SUB)
        socket.connect(f"tcp://{args.host}:{port}")
        socket.setsockopt(zmq.CONFLATE, 1)
        socket.setsockopt(zmq.SUBSCRIBE, b"")
        poller.register(socket, zmq.POLLIN)
        sockets[socket] = source

    if not sockets:
        raise RuntimeError("No selected teleimager ZMQ cameras are enabled.")

    print(
        json.dumps({"type": "ready", "cameras": [source.name for source in sockets.values()]}),
        flush=True,
    )

    try:
        while True:
            events = dict(poller.poll(timeout=1000))
            for socket, source in sockets.items():
                if socket not in events:
                    continue

                jpg = socket.recv()
                print(
                    json.dumps(
                        {
                            "type": "frame",
                            "camera": source.name,
                            "frame_id": source.frame_id,
                            "jpg": base64.b64encode(jpg).decode("ascii"),
                        },
                        separators=(",", ":"),
                    ),
                    flush=True,
                )
    finally:
        for socket in sockets:
            socket.close()
        context.term()


class TeleimagerRosBridge:
    def __init__(self, args: argparse.Namespace) -> None:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import CompressedImage, Image

        self._rclpy = rclpy
        self._node: Node = Node("teleimager_to_ros_image")
        self._topic_prefix = normalize_topic_prefix(args.topic_prefix)
        self._publish_raw = args.raw
        self._sources = {source.name: source for source in parse_cameras(args.cameras)}
        self._compressed_publishers = {
            name: self._node.create_publisher(CompressedImage, f"{self._topic_prefix}/{name}/compressed", 10)
            for name in self._sources
        }
        self._raw_publishers = (
            {
                name: self._node.create_publisher(Image, f"{self._topic_prefix}/{name}/image", 10)
                for name in self._sources
            }
            if self._publish_raw
            else {}
        )
        self._helper = self._start_helper(args)
        self._reader_thread = threading.Thread(target=self._read_helper_stdout, daemon=True)
        self._reader_thread.start()

    @property
    def node(self):
        return self._node

    def close(self) -> None:
        if self._helper.poll() is None:
            self._helper.terminate()
            try:
                self._helper.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._helper.kill()
        self._reader_thread.join(timeout=1.0)
        self._node.destroy_node()

    def _start_helper(self, args: argparse.Namespace) -> subprocess.Popen:
        helper_cmd = [
            args.client_python,
            os.path.abspath(__file__),
            "--zmq-helper",
            "--host",
            args.host,
            "--request-port",
            str(args.request_port),
            "--cameras",
            args.cameras,
        ]
        return subprocess.Popen(
            helper_cmd,
            stderr=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=False,
        )

    def _read_helper_stdout(self) -> None:
        if self._helper.stdout is None:
            return

        for raw_line in self._helper.stdout:
            line = raw_line.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                self._node.get_logger().warning(f"Teleimager helper emitted invalid JSON: {line}")
                continue

            message_type = payload.get("type")
            if message_type == "ready":
                cameras = ", ".join(payload.get("cameras", []))
                self._node.get_logger().info(f"Teleimager helper ready; cameras: {cameras}")
            elif message_type == "warning":
                self._node.get_logger().warning(str(payload.get("message", "")))
            elif message_type == "frame":
                self._publish_frame(payload)

    def _publish_frame(self, payload: dict[str, object]) -> None:
        import cv2
        import numpy as np

        camera = str(payload.get("camera", ""))
        frame_id = str(payload.get("frame_id", ""))
        jpg_text = payload.get("jpg")
        if camera not in self._compressed_publishers or not isinstance(jpg_text, str):
            return

        jpg = base64.b64decode(jpg_text)
        stamp = self._node.get_clock().now().to_msg()
        self._compressed_publishers[camera].publish(build_compressed_image(jpg, frame_id, stamp))

        raw_publisher = self._raw_publishers.get(camera)
        if raw_publisher is None:
            return

        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            raw_publisher.publish(build_raw_image(frame, frame_id, stamp))

    def read_helper_stderr(self) -> str:
        stderr = self._helper.stderr
        if stderr is None:
            return ""
        return _read_available(stderr)


def _read_available(stream: BinaryIO) -> str:
    chunks: list[bytes] = []
    while True:
        chunk = stream.read1(4096) if hasattr(stream, "read1") else stream.read(4096)
        if not chunk:
            break
        chunks.append(chunk)
        if len(chunk) < 4096:
            break
    return b"".join(chunks).decode("utf-8", errors="replace")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.123.164", help="Teleimager server IP on PC2.")
    parser.add_argument("--request-port", type=int, default=60000, help="Teleimager camera config REQ port.")
    parser.add_argument("--topic-prefix", default="/teleimager", help="ROS2 topic prefix for published images.")
    parser.add_argument(
        "--cameras",
        default="head,left_wrist,right_wrist",
        help="Comma-separated cameras: head,left_wrist,right_wrist.",
    )
    parser.add_argument("--rate", type=float, default=30.0, help="Reserved for compatibility.")
    parser.add_argument("--raw", action="store_true", help="Also publish decoded sensor_msgs/Image topics.")
    parser.add_argument(
        "--client-python",
        default=os.environ.get("TELEIMAGER_CLIENT_PYTHON", "/home/guobing/anaconda3/bin/python"),
        help="Python interpreter used only for the pyzmq teleimager client helper.",
    )
    parser.add_argument("--zmq-helper", action="store_true", help=argparse.SUPPRESS)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.zmq_helper:
        run_zmq_helper(args)
        return

    import rclpy

    rclpy.init()
    bridge = None
    try:
        bridge = TeleimagerRosBridge(args)
        while rclpy.ok():
            rclpy.spin_once(bridge.node, timeout_sec=0.1)
            if bridge._helper.poll() is not None:
                stderr = bridge.read_helper_stderr()
                raise RuntimeError(f"Teleimager helper exited with code {bridge._helper.returncode}:\n{stderr}")
            time.sleep(0.01)
    finally:
        if bridge is not None:
            bridge.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
