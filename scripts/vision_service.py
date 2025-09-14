#!/usr/bin/env python3
"""USB webcam vision service publishing simple detection stats via ROS 2.

Features (no heavy models by default):
- Face detection with OpenCV Haar cascades (frontal faces)
- Person detection with HOG+SVM (pedestrian detector)
- Simple motion metric via frame differencing

Publishes compact JSON on these topics:
- ``vision/faces``: {"count": N, "boxes": [[x,y,w,h], ...]}
- ``vision/objects``: {"objects": [{"label": "person", "box": [x,y,w,h]}]}
- ``vision/motion``: {"magnitude": float(0..1)}

Examples:
    Run against default camera at 640x480, ~5 FPS::

        $ python3 scripts/vision_service.py --device 0 --width 640 --height 480 --fps 5  # doctest: +SKIP
        $ ros2 topic echo vision/faces  # doctest: +SKIP
"""
from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from typing import List, Tuple

import cv2  # type: ignore
import numpy as np  # type: ignore

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class _Det:
    label: str
    box: Tuple[int, int, int, int]


class VisionNode(Node):
    """Read frames from a USB webcam and publish detection summaries."""

    def __init__(self, device: int = 0, width: int = 640, height: int = 480, fps: float = 5.0):
        super().__init__("vision")
        self._pub_faces = self.create_publisher(String, "vision/faces", 10)
        self._pub_objs = self.create_publisher(String, "vision/objects", 10)
        self._pub_motion = self.create_publisher(String, "vision/motion", 10)
        self._cap = cv2.VideoCapture(int(device))
        try:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
            if fps:
                self._cap.set(cv2.CAP_PROP_FPS, float(fps))
        except Exception:
            pass
        # Haar cascade for faces
        cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        self._face = cv2.CascadeClassifier(cascade_path)
        # HOG person detector
        self._hog = cv2.HOGDescriptor()
        self._hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        # Motion baseline
        self._prev_gray: np.ndarray | None = None
        interval = 1.0 / max(0.1, float(fps))
        self.create_timer(interval, self._tick)

    def _tick(self) -> None:
        ok, frame = self._cap.read()
        if not ok or frame is None:
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.equalizeHist(gray, gray)
        faces = self._detect_faces(gray)
        objs = self._detect_objects(gray)
        motion = self._motion_metric(gray)
        # Publish JSON summaries
        self._pub_json(self._pub_faces, {"count": len(faces), "boxes": faces})
        self._pub_json(self._pub_objs, {"objects": [{"label": d.label, "box": list(d.box)} for d in objs]})
        self._pub_json(self._pub_motion, {"magnitude": motion})

    def _detect_faces(self, gray: np.ndarray) -> List[List[int]]:
        try:
            rects = self._face.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
        except Exception:
            rects = []
        return [[int(x), int(y), int(w), int(h)] for (x, y, w, h) in rects]

    def _detect_objects(self, gray: np.ndarray) -> List[_Det]:
        dets: List[_Det] = []
        try:
            rects, _ = self._hog.detectMultiScale(gray, winStride=(8, 8), padding=(8, 8), scale=1.05)
            for (x, y, w, h) in rects:
                dets.append(_Det("person", (int(x), int(y), int(w), int(h))))
        except Exception:
            pass
        return dets

    def _motion_metric(self, gray: np.ndarray) -> float:
        mag = 0.0
        if self._prev_gray is None:
            self._prev_gray = gray
            return 0.0
        try:
            diff = cv2.absdiff(gray, self._prev_gray)
            # Normalize magnitude to 0..1 based on mean absolute difference
            mag = float(np.mean(diff)) / 255.0
        except Exception:
            mag = 0.0
        self._prev_gray = gray
        return round(mag, 3)

    def _pub_json(self, pub, obj) -> None:
        msg = String()
        try:
            msg.data = json.dumps(obj, separators=(",", ":"))
        except Exception:
            msg.data = str(obj)
        pub.publish(msg)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=5.0)
    ns = parser.parse_args(argv)
    rclpy.init()
    node = VisionNode(device=ns.device, width=ns.width, height=ns.height, fps=ns.fps)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

