#!/usr/bin/env python3
import threading
from typing import Optional, Tuple

import zmq
import msgpack

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time


def _time(st) -> Time:
    t = Time()
    if isinstance(st, dict):
        t.sec = int(st.get("sec", 0))
        t.nanosec = int(st.get("nanosec", st.get("nsec", 0)))
    return t


def _stamp_key(d: dict) -> Tuple[int, int]:
    s = (d.get("header", {}) or {}).get("stamp", {}) or {}
    return (int(s.get("sec", 0)), int(s.get("nanosec", s.get("nsec", 0))))


def _fill(m: Odometry, d: dict):
    h = d.get("header", {}) or {}
    m.header.frame_id = str(h.get("frame_id", ""))
    m.header.stamp = _time(h.get("stamp", {}) or {})
    m.child_frame_id = str(d.get("child_frame_id", ""))

    p = d.get("pose", {}) or {}
    pos = p.get("position", {}) or {}
    ori = p.get("orientation", {}) or {}

    m.pose.pose.position.x = float(pos.get("x", 0.0))
    m.pose.pose.position.y = float(pos.get("y", 0.0))
    m.pose.pose.position.z = float(pos.get("z", 0.0))

    m.pose.pose.orientation.x = float(ori.get("x", 0.0))
    m.pose.pose.orientation.y = float(ori.get("y", 0.0))
    m.pose.pose.orientation.z = float(ori.get("z", 0.0))
    m.pose.pose.orientation.w = float(ori.get("w", 1.0))

    pc = p.get("covariance", [0.0] * 36) or ([0.0] * 36)
    m.pose.covariance = [float(x) for x in (list(pc)[:36] + [0.0] * 36)[:36]]

    tw = d.get("twist", {}) or {}
    lin = tw.get("linear", {}) or {}
    ang = tw.get("angular", {}) or {}

    m.twist.twist.linear.x = float(lin.get("x", 0.0))
    m.twist.twist.linear.y = float(lin.get("y", 0.0))
    m.twist.twist.linear.z = float(lin.get("z", 0.0))

    m.twist.twist.angular.x = float(ang.get("x", 0.0))
    m.twist.twist.angular.y = float(ang.get("y", 0.0))
    m.twist.twist.angular.z = float(ang.get("z", 0.0))

    tc = tw.get("covariance", [0.0] * 36) or ([0.0] * 36)
    m.twist.covariance = [float(x) for x in (list(tc)[:36] + [0.0] * 36)[:36]]


class Bridge(Node):
    def __init__(self, host="192.168.1.202", port=5560, out_topic="/SLAM/Pose", publish_hz=60.0):
        super().__init__("odom_zmq_ros2_bridge")

        self.pub = self.create_publisher(Odometry, out_topic, 10)

        self._lock = threading.Lock()
        self._latest: Optional[dict] = None
        self._latest_stamp: Optional[Tuple[int, int]] = None
        self._last_pub_stamp: Optional[Tuple[int, int]] = None

        self._running = True
        self._logged_first_rx = False
        self._logged_first_tx = False

        self.ctx = zmq.Context.instance()
        self.sock = self.ctx.socket(zmq.SUB)
        self.sock.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sock.setsockopt(zmq.RCVHWM, 1)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.setsockopt(zmq.CONFLATE, 1)

        endpoint = f"tcp://{host}:{port}"
        self.sock.connect(endpoint)

        self.poller = zmq.Poller()
        self.poller.register(self.sock, zmq.POLLIN)

        self.get_logger().info(f"up: ZMQ {endpoint} -> ROS2 {out_topic}")

        self.thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.thread.start()

        self.timer = self.create_timer(1.0 / float(publish_hz), self._publish_latest)

    def _recv_loop(self):
        while self._running and rclpy.ok():
            try:
                events = dict(self.poller.poll(timeout=200))
                if self.sock not in events:
                    continue

                payload = self.sock.recv(flags=zmq.DONTWAIT)
                d = msgpack.unpackb(payload, raw=False)
                st = _stamp_key(d)

                with self._lock:
                    self._latest = d
                    self._latest_stamp = st

                if not self._logged_first_rx:
                    self._logged_first_rx = True
                    self.get_logger().info(f"rx: first message stamp={st}")

            except (zmq.Again, Exception):
                continue

    def _publish_latest(self):
        with self._lock:
            d = self._latest
            st = self._latest_stamp

        if d is None or st is None or st == self._last_pub_stamp:
            return

        msg = Odometry()
        try:
            _fill(msg, d)
            self.pub.publish(msg)
            self._last_pub_stamp = st

            if not self._logged_first_tx:
                self._logged_first_tx = True
                self.get_logger().info(f"tx: first publish stamp={st}")

        except Exception:
            pass

    def destroy_node(self):
        self._running = False
        try:
            self.poller.unregister(self.sock)
        except Exception:
            pass
        try:
            self.sock.close(0)
        except Exception:
            pass
        try:
            self.thread.join(timeout=1.0)
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()