#!/usr/bin/env python3
"""
Control gate node: replaces ackermann_mux with explicit button-state gating.

State machine:
  Neither L2 nor R2  → publish zero velocity (IDLE)
  L2 only            → forward teleop RC commands (MANUAL)
  R2 only            → forward autonomous drive commands (AUTONOMOUS)
  Both L2 + R2       → publish zero velocity (CONFLICT/safe)

A watchdog timer publishes zero velocity if no joy message is received
within the timeout window (controller disconnect).
"""

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy


class ControlGate(Node):
    def __init__(self):
        super().__init__('control_gate')

        # Parameters
        self.declare_parameter('l2_button', 6)
        self.declare_parameter('r2_button', 5)
        self.declare_parameter('joy_timeout', 0.5)

        self.l2_button = self.get_parameter('l2_button').value
        self.r2_button = self.get_parameter('r2_button').value
        self.joy_timeout = self.get_parameter('joy_timeout').value

        # Button state
        self.l2_pressed = False
        self.r2_pressed = False
        self.joy_alive = False

        # Latest commands from each source
        self.last_teleop = None
        self.last_drive = None

        # Publisher
        self.pub = self.create_publisher(AckermannDriveStamped, 'ackermann_drive', 10)

        # Subscribers
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.create_subscription(AckermannDriveStamped, 'teleop', self.teleop_callback, 10)
        self.create_subscription(AckermannDriveStamped, 'drive', self.drive_callback, 10)

        # Watchdog timer
        self.last_joy_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)

    def _make_zero(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        return msg

    def joy_callback(self, msg: Joy):
        self.last_joy_time = self.get_clock().now()
        self.joy_alive = True

        prev_l2 = self.l2_pressed
        prev_r2 = self.r2_pressed

        if self.l2_button < len(msg.buttons):
            self.l2_pressed = bool(msg.buttons[self.l2_button])
        if self.r2_button < len(msg.buttons):
            self.r2_pressed = bool(msg.buttons[self.r2_button])

        # If we just left an active mode, send zero to stop motors
        was_active = (prev_l2 and not prev_r2) or (prev_r2 and not prev_l2)
        is_active = (self.l2_pressed and not self.r2_pressed) or (self.r2_pressed and not self.l2_pressed)
        if was_active and not is_active:
            self.pub.publish(self._make_zero())

    def teleop_callback(self, msg: AckermannDriveStamped):
        self.last_teleop = msg
        if self.joy_alive and self.l2_pressed and not self.r2_pressed:
            self.pub.publish(msg)

    def drive_callback(self, msg: AckermannDriveStamped):
        self.last_drive = msg
        if self.joy_alive and self.r2_pressed and not self.l2_pressed:
            self.pub.publish(msg)

    def watchdog_callback(self):
        dt = (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9
        if dt > self.joy_timeout:
            if self.joy_alive:
                self.get_logger().warn('Joy timeout — publishing zero velocity')
            self.joy_alive = False
            self.l2_pressed = False
            self.r2_pressed = False
            self.pub.publish(self._make_zero())


def main(args=None):
    rclpy.init(args=args)
    node = ControlGate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
