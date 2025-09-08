#!/usr/bin/env python3
import os
import sys
import json
import time
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from lifecycle_msgs.srv import GetState

from std_msgs.msg import Header  # only for typing
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

OLED_SOCK = os.environ.get('OLED_SOCK', '/run/oled/statusd.sock')
HTTP_PORT = int(os.environ.get('ROS_STATUS_HTTP_PORT', '8080'))


def oled_send(header, *lines):
    try:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        s.connect(OLED_SOCK)
        s.send(json.dumps({'header': header, 'lines': list(lines)}).encode('utf-8'))
        s.close()
    except Exception:
        pass


class RateTracker:
    def __init__(self, name):
        self.name = name
        self.count = 0
        self.last_t = 0.0
        self.rate = 0.0
        self.last_seen_wall = 0.0
        self.lock = threading.Lock()

    def tick(self, stamp_sec: float):
        now = time.time()
        with self.lock:
            if self.last_t > 0:
                dt = stamp_sec - self.last_t
                if dt > 0:
                    # EMA toward instantaneous rate
                    inst = 1.0 / dt
                    self.rate = 0.7 * self.rate + 0.3 * inst if self.rate > 0 else inst
            self.count += 1
            self.last_t = stamp_sec
            self.last_seen_wall = now

    def snapshot(self):
        with self.lock:
            age = max(0.0, time.time() - self.last_seen_wall) if self.last_seen_wall else None
            return {
                'name': self.name,
                'count': self.count,
                'rate_hz': round(self.rate, 2) if self.rate else 0.0,
                'last_seen_s': round(age, 2) if age is not None else None,
            }


class ROSStatus(Node):
    def __init__(self):
        super().__init__('ros_statusd')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.trackers = {
            'odom': RateTracker('odom'),
            'imu': RateTracker('imu'),
            'scan': RateTracker('scan'),
            'image': RateTracker('image'),
        }

        self.create_subscription(Odometry, '/odom', self._cb_odom, qos)
        self.create_subscription(Imu, '/imu/data', self._cb_imu, qos)
        self.create_subscription(LaserScan, '/scan', self._cb_scan, qos)
        self.create_subscription(Image, '/camera/image_raw', self._cb_image, qos)

        self.lifecycle_nodes = [
            'controller_server', 'planner_server', 'behavior_server', 'bt_navigator',
            'map_server', 'amcl', 'smoother_server', 'waypoint_follower'
        ]
        self.lifecycle_states = {}
        self.state_lock = threading.Lock()

        self.create_timer(5.0, self.refresh_lifecycle_states)
        self.create_timer(2.0, self.update_oled)

    def _stamp_to_sec(self, header: Header):
        # Use ROS stamp if present, else wall time
        try:
            return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9
        except Exception:
            return time.time()

    def _cb_odom(self, msg: Odometry):
        self.trackers['odom'].tick(self._stamp_to_sec(msg.header))

    def _cb_imu(self, msg: Imu):
        self.trackers['imu'].tick(self._stamp_to_sec(msg.header))

    def _cb_scan(self, msg: LaserScan):
        # LaserScan's header is in msg.header
        self.trackers['scan'].tick(self._stamp_to_sec(msg.header))

    def _cb_image(self, msg: Image):
        self.trackers['image'].tick(self._stamp_to_sec(msg.header))

    def refresh_lifecycle_states(self):
        # Best-effort query of lifecycle states
        results = {}
        for name in self.lifecycle_nodes:
            srv = f'/{name}/get_state'
            if not self.wait_for_service_once(srv, timeout_sec=0.01):
                continue
            try:
                cli = self.create_client(GetState, srv)
                if not cli.wait_for_service(timeout_sec=0.2):
                    continue
                req = GetState.Request()
                future = cli.call_async(req)
                rclpy.task.Future  # type hint guard
                rclpy.spin_until_future_complete(self, future, timeout_sec=0.3)
                if future.done() and future.result() is not None:
                    results[name] = future.result().current_state.label
            except Exception:
                continue
        with self.state_lock:
            self.lifecycle_states = results

    def wait_for_service_once(self, name, timeout_sec=0.0):
        try:
            # Avoid blocking; query service names
            for srv_name, _ in self.get_service_names_and_types():
                if srv_name == name:
                    return True
        except Exception:
            pass
        if timeout_sec > 0:
            end = time.time() + timeout_sec
            while time.time() < end:
                try:
                    for srv_name, _ in self.get_service_names_and_types():
                        if srv_name == name:
                            return True
                except Exception:
                    pass
                time.sleep(0.01)
        return False

    def snapshot(self):
        try:
            nodes = self.get_node_names_and_namespaces()
            topics = self.get_topic_names_and_types()
            ncount, tcount = len(nodes), len(topics)
        except Exception:
            ncount = tcount = 0
        trackers = {k: v.snapshot() for k, v in self.trackers.items()}
        with self.state_lock:
            lifecycle = dict(self.lifecycle_states)
        meminfo = {}
        try:
            with open('/proc/meminfo') as f:
                for line in f:
                    k, v = line.split(':', 1)
                    meminfo[k.strip()] = v.strip()
        except Exception:
            pass
        loadavg = os.getloadavg() if hasattr(os, 'getloadavg') else (0, 0, 0)
        return {
            'time': datetime.utcnow().isoformat() + 'Z',
            'nodes': ncount,
            'topics': tcount,
            'rates': trackers,
            'lifecycle': lifecycle,
            'system': {
                'loadavg': loadavg,
                'meminfo': meminfo,
            }
        }

    def update_oled(self):
        snap = self.snapshot()
        r = snap['rates']
        lines = [
            f"nodes:{snap['nodes']} topics:{snap['topics']}",
            f"odom:{r['odom']['rate_hz']:.1f} imu:{r['imu']['rate_hz']:.1f}",
            f"scan:{r['scan']['rate_hz']:.1f} cam:{r['image']['rate_hz']:.1f}",
        ]
        active = [k for k, v in snap['lifecycle'].items() if v == 'active']
        if active:
            lines.append(f"nav2:{len(active)} up")
        oled_send('ROS2', *lines)


class HTTPHandler(BaseHTTPRequestHandler):
    statusd: ROSStatus = None  # injected

    def do_GET(self):
        if self.path.startswith('/api/status'):
            snap = self.statusd.snapshot()
            data = json.dumps(snap, indent=2).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        if self.path.startswith('/health'):
            snap = self.statusd.snapshot()
            ok = snap['nodes'] > 0
            self.send_response(200 if ok else 503)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'OK' if ok else b'DEAD')
            return
        # HTML dashboard
        snap = self.statusd.snapshot()
        html = self.render_html(snap).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(html)))
        self.end_headers()
        self.wfile.write(html)

    def log_message(self, fmt, *args):
        # Quieter logs
        return

    @staticmethod
    def render_html(snap):
        def safe(s):
            return str(s)
        rates = snap['rates']
        lifecycle = snap['lifecycle']
        rows = ''
        for key in ['odom', 'imu', 'scan', 'image']:
            v = rates[key]
            rows += f"<tr><td>{key}</td><td>{v['rate_hz']}</td><td>{v['last_seen_s']}</td><td>{v['count']}</td></tr>"
        liferows = ''.join(f"<tr><td>{k}</td><td>{v}</td></tr>" for k, v in lifecycle.items())
        return f"""
<!doctype html>
<html><head><meta charset='utf-8'><meta http-equiv='refresh' content='2'>
<title>ROS 2 Status</title>
<style>body{{font-family:sans-serif;margin:1rem}} table{{border-collapse:collapse}} td,th{{border:1px solid #ccc;padding:4px 8px}}</style>
</head>
<body>
<h2>ROS 2 Status — {safe(snap['time'])}</h2>
<p>Nodes: {snap['nodes']} — Topics: {snap['topics']}</p>
<h3>Rates</h3>
<table><thead><tr><th>Topic</th><th>Rate (Hz)</th><th>Last Seen (s)</th><th>Count</th></tr></thead>
<tbody>{rows}</tbody></table>
<h3>Lifecycle</h3>
<table><thead><tr><th>Node</th><th>State</th></tr></thead>
<tbody>{liferows}</tbody></table>
<h3>System</h3>
<pre>{safe(json.dumps(snap['system'], indent=2))}</pre>
</body></html>
"""


def main():
    rclpy.init()
    node = ROSStatus()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Start ROS spinning thread
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # Start HTTP server (main thread)
    HTTPHandler.statusd = node
    server = ThreadingHTTPServer(('0.0.0.0', HTTP_PORT), HTTPHandler)
    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    server.server_close()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

