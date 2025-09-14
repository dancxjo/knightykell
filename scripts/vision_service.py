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
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time as RosTime


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
        # Landmarks (LBF) and face embeddings (SFace)
        import os
        models_dir = os.getenv("VISION_MODELS_DIR") or "/opt/psyche/models/vision"
        self._lbf_path = models_dir + "/lbfmodel.yaml"
        self._sface_path = models_dir + "/face_recognition_sface_2021dec.onnx"
        try:
            self._facemark = cv2.face.createFacemarkLBF()
            self._facemark.loadModel(self._lbf_path)
        except Exception:
            self._facemark = None
        try:
            self._sface = cv2.dnn.readNetFromONNX(self._sface_path)
        except Exception:
            self._sface = None
        # HOG person detector
        self._hog = cv2.HOGDescriptor()
        self._hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        # Motion baseline
        self._prev_gray: np.ndarray | None = None
        interval = 1.0 / max(0.1, float(fps))
        self.create_timer(interval, self._tick)
        # Qdrant client: prefer server URL, fallback to embedded path
        try:
            import os
            from qdrant_client import QdrantClient
            from qdrant_client.http.models import Distance, VectorParams
            url = os.getenv("QDRANT_URL")
            if url:
                self._qdr = QdrantClient(url=url)
            else:
                path = os.getenv("QDRANT_PATH", "/opt/psyche/qdrant")
                self._qdr = QdrantClient(path=path)
            if "faces" not in [c.name for c in self._qdr.get_collections().collections]:
                self._qdr.recreate_collection(
                    collection_name="faces",
                    vectors_config=VectorParams(size=128, distance=Distance.COSINE),
                )
        except Exception:
            self._qdr = None
        # Neo4j driver (optional)
        try:
            import os
            from neo4j import GraphDatabase
            uri = os.getenv("NEO4J_URI", "bolt://localhost:7687")
            user = os.getenv("NEO4J_USER", "neo4j")
            pwd = os.getenv("NEO4J_PASSWORD", "neo4j")
            self._neo = GraphDatabase.driver(uri, auth=(user, pwd))
        except Exception:
            self._neo = None
        # Additional publishers
        self._pub_land = self.create_publisher(String, "vision/face_landmarks", 10)
        self._pub_id = self.create_publisher(String, "vision/face_id", 10)
        self._pub_img = self.create_publisher(Image, "vision/image_color", 2)

    def _tick(self) -> None:
        ok, frame = self._cap.read()
        if not ok or frame is None:
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.equalizeHist(gray, gray)
        faces = self._detect_faces(gray)
        objs = self._detect_objects(gray)
        motion = self._motion_metric(gray)
        # Publish images (color)
        self._publish_image(frame, encoding="bgr8")
        # Landmarks & embeddings
        lmarks = []
        idents = []
        if faces:
            lmarks = self._landmarks(gray, faces)
            for (x, y, w, h) in faces:
                emb = self._embed_face(frame[y:y+h, x:x+w])
                if emb is not None:
                    ident = self._identify(emb)
                    idents.append({"box": [x, y, w, h], **ident})
        if lmarks:
            self._pub_json(self._pub_land, {"faces": lmarks})
        if idents:
            self._pub_json(self._pub_id, {"faces": idents})
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

    def _publish_image(self, img: np.ndarray, encoding: str = "bgr8") -> None:
        try:
            msg = Image()
            msg.height, msg.width = img.shape[:2]
            msg.encoding = encoding
            msg.is_bigendian = 0
            msg.step = int(img.strides[0])
            msg.data = img.tobytes()
            self._pub_img.publish(msg)
        except Exception:
            pass

    def _landmarks(self, gray: np.ndarray, faces: List[List[int]]):
        if self._facemark is None:
            return []
        rects = [tuple(map(int, f)) for f in faces]
        try:
            ok, pts = self._facemark.fit(gray, rects)
            if not ok:
                return []
            out = []
            for p in pts:
                out.append({"points": [[int(x), int(y)] for (x, y) in p.reshape(-1, 2)]})
            return out
        except Exception:
            return []

    def _embed_face(self, roi_bgr: np.ndarray):
        if self._sface is None or roi_bgr is None or roi_bgr.size == 0:
            return None
        try:
            face = cv2.resize(roi_bgr, (112, 112))
            blob = cv2.dnn.blobFromImage(face, scalefactor=1/255.0, size=(112, 112), mean=(0,0,0), swapRB=True, crop=False)
            self._sface.setInput(blob)
            vec = self._sface.forward().flatten().astype(np.float32)
            # Normalize
            n = np.linalg.norm(vec) + 1e-9
            return (vec / n).tolist()
        except Exception:
            return None

    def _identify(self, embedding: List[float]):
        # Search in Qdrant; add if new
        name = None
        score = 0.0
        face_id = None
        try:
            if self._qdr is not None:
                res = self._qdr.search(collection_name="faces", query_vector=embedding, limit=1)
                if res and len(res) > 0 and res[0].score is not None:
                    score = float(res[0].score)
                    payload = res[0].payload or {}
                    name = payload.get("name")
                    face_id = res[0].id
                # Add/update current embedding
                self._qdr.upsert(
                    collection_name="faces",
                    points=[{
                        "vector": embedding,
                        "payload": {"name": name or None, "ts": time.time()},
                    }],
                )
        except Exception:
            pass
        # Optional: link to Neo4j Person by name
        if name and self._neo is not None:
            try:
                with self._neo.session() as session:
                    session.run("MERGE (:Person {name: $n})", n=name)
            except Exception:
                pass
        return {"name": name or "unknown", "score": round(score, 3), "id": face_id}


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
