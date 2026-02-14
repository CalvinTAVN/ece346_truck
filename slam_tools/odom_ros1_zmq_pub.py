#!/usr/bin/env python3
import zmq
import msgpack
import rospy

from nav_msgs.msg import Odometry

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

class OdomZMQPublisher:
    def __init__(self,
                 odom_topic="/SLAM/Pose",
                 bind_host="0.0.0.0",
                 bind_port=5560,
                 snd_hwm=20):

        rospy.init_node("odom_ros1_to_zmq_pub", anonymous=False)

        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.SNDHWM, int(snd_hwm))
        self._sock.setsockopt(zmq.LINGER, 0)

        endpoint = f"tcp://{bind_host}:{bind_port}"
        self._sock.bind(endpoint)
        rospy.loginfo(f"[ZMQ PUB] Bound to {endpoint}")

        self._sub = rospy.Subscriber(odom_topic, Odometry, self._cb, queue_size=20)
        rospy.loginfo(f"[ROS1] Subscribed to {odom_topic}")

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

    def spin(self):
        rospy.loginfo("[ROS1→ZMQ] Odometry bridge running")
        rospy.spin()

    def shutdown(self):
        try:
            self._sock.close(0)
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