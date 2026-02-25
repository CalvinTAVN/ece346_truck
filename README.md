# ece346_truck

Modified f1tenth driver stack for ECE346 trucks. Uses ROS2 Foxy with a PS4 DualShock controller over Bluetooth.

## New Truck Setup

### 1. Flash JetPack on the Jetson

### 2. Set up Bluetooth (PS4 Controller)

Unblock Bluetooth and make it persistent across reboots:
```bash
sudo rfkill unblock bluetooth
sudo systemctl daemon-reload
sudo systemctl enable bluetooth
sudo systemctl restart bluetooth
```

Create a systemd service so Bluetooth is always unblocked on boot:
```bash
sudo bash -c 'cat > /etc/systemd/system/bluetooth-unblock.service << EOF
[Unit]
Description=Unblock Bluetooth
Before=bluetooth.service

[Service]
Type=oneshot
ExecStart=/usr/sbin/rfkill unblock bluetooth

[Install]
WantedBy=multi-user.target
EOF'
sudo systemctl enable bluetooth-unblock.service
sudo systemctl start bluetooth-unblock.service
```

Pair the PS4 controller:
```bash
sudo bluetoothctl
power on
agent on
scan on
# Put PS4 controller in pairing mode: hold Share + PS button until light bar flashes rapidly
# Find "Wireless Controller" in the scan results and note its MAC address
pair XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
exit
```

Verify it shows up:
```bash
ls /dev/input/js0
```

After trusting the controller, it will auto-reconnect on future boots when you press the PS button.

### 3. Clone and build
```bash
cd ~
git clone --recurse-submodules https://github.com/CalvinTAVN/ece346_truck.git
cd ece346_truck
./setup.sh
```

### 4. Cold boot
Required for the Logitech F710 kernel module to load. Reboot the Jetson after setup completes.

### 5. Test
```bash
source ~/ece346_truck/install/setup.bash
sudo apt install -y jstest-gtk
jstest /dev/input/js0
```

## Deadman's switch
On the PS4 controller, L2 is the deadman's switch for teleop, and R2 is the deadman's switch for autonomous navigation. You can remap buttons in `f1tenth_stack/config/joy_teleop.yaml`.

## Topics

### Topics that the driver stack subscribe to
- `/drive`: Topic for autonomous navigation, uses `AckermannDriveStamped` messages.

### Sensor topics published by the driver stack
- `/scan`: Topic for `LaserScan` messages.
- `/odom`: Topic for `Odometry` messages.
- `/sensors/imu/raw`: Topic for `Imu` messages.
- `/sensors/core`: Topic for telemetry data from the VESC

## External Dependencies

1. ackermann_msgs [https://index.ros.org/r/ackermann_msgs/#foxy](https://index.ros.org/r/ackermann_msgs/#foxy).
2. urg_node [https://index.ros.org/p/urg_node/#foxy](https://index.ros.org/p/urg_node/#foxy). This is the driver for Hokuyo LiDARs.
3. joy [https://index.ros.org/p/joy/#foxy](https://index.ros.org/p/joy/#foxy). This is the driver for joysticks in ROS 2.
4. teleop_tools  [https://index.ros.org/p/teleop_tools/#foxy](https://index.ros.org/p/teleop_tools/#foxy). This is the package for teleop with joysticks in ROS 2.
5. vesc [GitHub - f1tenth/vesc at ros2](https://github.com/f1tenth/vesc/tree/ros2). This is the driver for VESCs in ROS 2.
6. ackermann_mux [GitHub - f1tenth/ackermann_mux: Twist multiplexer](https://github.com/f1tenth/ackermann_mux). This is a package for multiplexing ackermann messages in ROS 2.
<!-- 7. rosbridge_suite [https://index.ros.org/p/rosbridge_suite/#foxy-overview](https://index.ros.org/p/rosbridge_suite/#foxy-overview) This is a package that allows for websocket connection in ROS 2. -->

## Package in this repo

1. f1tenth_stack: maintains the bringup launch and all parameter files

## Nodes launched in bringup

1. joy
2. joy_teleop
3. ackermann_to_vesc_node
4. vesc_to_odom_node
5. vesc_driver_node
6. urg_node
7. ackermann_mux

## Parameters and topics for dependencies

### vesc_driver

1. Parameters:
   - duty_cycle_min, duty_cycle_max
   - current_min, current_max
   - brake_min, brake_max
   - speed_min, speed_max
   - position_min, position_max
   - servo_min, servo_max
2. Publishes to:
   - sensors/core
   - sensors/servo_position_command
   - sensors/imu
   - sensors/imu/raw
3. Subscribes to:
   - commands/motor/duty_cycle
   - commands/motor/current
   - commands/motor/brake
   - commands/motor/speed
   - commands/motor/position
   - commands/servo/position

### ackermann_to_vesc

1. Parameters:
   - speed_to_erpm_gain
   - speed_to_erpm_offset
   - steering_angle_to_servo_gain
   - steering_angle_to_servo_offset
2. Publishes to:
   - ackermann_cmd
3. Subscribes to:
   - commands/motor/speed
   - commands/servo/position

### vesc_to_odom

1. Parameters:
   - odom_frame
   - base_frame
   - use_servo_cmd_to_calc_angular_velocity
   - speed_to_erpm_gain
   - speed_to_erpm_offset
   - steering_angle_to_servo_gain
   - steering_angle_to_servo_offset
   - wheelbase
   - publish_tf
2. Publishes to:
   - odom
3. Subscribes to:
   - sensors/core
   - sensors/servo_position_command

### throttle_interpolator

1. Parameters:
   - rpm_input_topic
   - rpm_output_topic
   - servo_input_topic
   - servo_output_topic
   - max_acceleration
   - speed_max
   - speed_min
   - throttle_smoother_rate
   - speed_to_erpm_gain
   - max_servo_speed
   - steering_angle_to_servo_gain
   - servo_smoother_rate
   - servo_max
   - servo_min
   - steering_angle_to_servo_offset
2. Publishes to:
   - topic described in rpm_output_topic
   - topic described in servo_output_topic
3. Subscribes to:
   - topic described in rpm_input_topic
   - topic described in servo_input_topic
