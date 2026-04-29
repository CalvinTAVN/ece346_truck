#!/usr/bin/env python3
import zmq
import msgpack
import rospy

from nav_msgs.msg import Odometry
from tagslam_ros.msg import AprilTagDetectionArray

def _hdr_to_dict(h):
    # ROS1 header: stamp has .secs/.nsecs
    return {
        "seq": int(getattr(h, "seq", 0)),
        "stamp": {"sec": int(h.stamp.secs), "nanosec": int(h.stamp.nsecs)},
        "frame_id": str(h.frame_id),
    }

def _odom_to_dict(msg: Odometry):
    # Covariances are length 36 lists in ROS1
    return {
        "header": _hdr_to_dict(msg.header),
        "child_frame_id": str(msg.child_frame_id),

        "pose": {
            "position": {
                "x": float(msg.pose.pose.position.x),
                "y": float(msg.pose.pose.position.y),
                "z": float(msg.pose.pose.position.z),
            },
            "orientation": {
                "x": float(msg.pose.pose.orientation.x),
                "y": float(msg.pose.pose.orientation.y),
                "z": float(msg.pose.pose.orientation.z),
                "w": float(msg.pose.pose.orientation.w),
            },
            "covariance": [float(x) for x in msg.pose.covariance],
        },

        "twist": {
            "linear": {
                "x": float(msg.twist.twist.linear.x),
                "y": float(msg.twist.twist.linear.y),
                "z": float(msg.twist.twist.linear.z),
            },
            "angular": {
                "x": float(msg.twist.twist.angular.x),
                "y": float(msg.twist.twist.angular.y),
                "z": float(msg.twist.twist.angular.z),
            },
            "covariance": [float(x) for x in msg.twist.covariance],
        },
    }

def _pose_to_dict(pose):
    return {
        "position": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
        },
        "orientation": {
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
            "w": float(pose.orientation.w),
        },
    }

def _point_to_dict(pt):
    return {"x": float(pt.x), "y": float(pt.y), "z": float(pt.z)}

def _tag_array_to_dict(msg):
    return {
        "header": _hdr_to_dict(msg.header),
        "detections": [
            {
                "id": int(d.id),
                "size": float(d.size),
                "static_tag": bool(d.static_tag),
                "pose": _pose_to_dict(d.pose),
                "center": _point_to_dict(d.center),
                "corners": [_point_to_dict(c) for c in d.corners],
            }
            for d in msg.detections
        ],
    }


class OdomZMQPublisher:
    def __init__(self,
                 odom_topic="/SLAM/Pose",
                 tag_topic="/SLAM/Tag_Detections_Dynamic",
                 bind_host="0.0.0.0",
                 bind_port=5560,
                 tag_port=5561,
                 snd_hwm=20):

        rospy.init_node("odom_ros1_to_zmq_pub", anonymous=False)

        self._ctx = zmq.Context.instance()

        # Odom socket
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.SNDHWM, int(snd_hwm))
        self._sock.setsockopt(zmq.LINGER, 0)
        endpoint = f"tcp://{bind_host}:{bind_port}"
        self._sock.bind(endpoint)
        rospy.loginfo(f"[ZMQ PUB] Odom bound to {endpoint}")

        # Tag detection socket
        self._tag_sock = self._ctx.socket(zmq.PUB)
        self._tag_sock.setsockopt(zmq.SNDHWM, int(snd_hwm))
        self._tag_sock.setsockopt(zmq.LINGER, 0)
        tag_endpoint = f"tcp://{bind_host}:{tag_port}"
        self._tag_sock.bind(tag_endpoint)
        rospy.loginfo(f"[ZMQ PUB] Tags bound to {tag_endpoint}")

        self._sub = rospy.Subscriber(odom_topic, Odometry, self._cb, queue_size=20)
        rospy.loginfo(f"[ROS1] Subscribed to {odom_topic}")

        self._tag_sub = rospy.Subscriber(tag_topic, AprilTagDetectionArray, self._tag_cb, queue_size=10)
        rospy.loginfo(f"[ROS1] Subscribed to {tag_topic}")

        rospy.on_shutdown(self.shutdown)

    def _cb(self, msg: Odometry):
        try:
            data = _odom_to_dict(msg)
            payload = msgpack.packb(data, use_bin_type=True)
            self._sock.send(payload, flags=zmq.NOBLOCK)
        except zmq.Again:
            rospy.logwarn_throttle(2.0, "[ZMQ PUB] Send would block (dropping odom)")
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"[ZMQ PUB] Failed to publish odom: {e}")

    def _tag_cb(self, msg):
        try:
            data = _tag_array_to_dict(msg)
            payload = msgpack.packb(data, use_bin_type=True)
            self._tag_sock.send(payload, flags=zmq.NOBLOCK)
        except zmq.Again:
            rospy.logwarn_throttle(2.0, "[ZMQ PUB] Send would block (dropping tags)")
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"[ZMQ PUB] Failed to publish tags: {e}")

    def spin(self):
        rospy.loginfo("[ROS1→ZMQ] Odometry bridge running")
        rospy.spin()

    def shutdown(self):
        try:
            self._sock.close(0)
        except Exception:
            pass
        try:
            self._tag_sock.close(0)
        except Exception:
            pass
        rospy.loginfo("[ROS1→ZMQ] Shutdown complete")

if __name__ == "__main__":
    node = OdomZMQPublisher(
        odom_topic="/SLAM/Pose",
        bind_host="0.0.0.0",   # allow remote ROS2 machine to connect
        bind_port=5560,
        snd_hwm=50,
    )
    node.spin()