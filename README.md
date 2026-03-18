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

## Control Pipeline

### Overview

The truck uses a **control_gate** node instead of `ackermann_mux`. The control_gate checks DS4 button state every cycle and only forwards the appropriate command source. No buttons pressed = nothing moves.

### State Machine

| Button State     | Action                                    |
|------------------|-------------------------------------------|
| Neither L2 nor R2| Publish zero velocity (IDLE)              |
| L2 only (btn 6)  | Forward `/teleop` RC commands (MANUAL)    |
| R2 only (btn 5)  | Forward `/drive` autonomous commands (AUTO)|
| Both L2 + R2     | Publish zero velocity (CONFLICT/safe)     |
| Controller disconnect (>0.5s no joy) | Publish zero velocity |

### Node & Topic Graph

```
                    ┌─────────────┐
  DS4 Controller ──>│  joy_linux   │──> /joy (sensor_msgs/Joy)
                    └─────────────┘         │
                           │                │
                           v                v
                    ┌─────────────┐   ┌──────────────┐
                    │ joy_teleop  │   │ control_gate │
                    └─────────────┘   └──────────────┘
                           │                │
               /teleop ────┘                │ reads /joy button state
          (AckermannDriveStamped)           │ reads /teleop (manual)
                           │                │ reads /drive  (autonomous)
                           └───────>────────┤
                                            │
  traj_planner ──> /drive ─────────>────────┤
          (AckermannDriveStamped)           │
                                            v
                                   /ackermann_drive
                                  (AckermannDriveStamped)
                                            │
                                            v
                                 ┌─────────────────────┐
                                 │ ackermann_to_vesc    │
                                 └─────────────────────┘
                                      │           │
                  /commands/motor/     │           │  /commands/servo/
                  unsmoothed_speed     │           │  unsmoothed_position
                                      v           v
                                 ┌─────────────────────┐
                                 │throttle_interpolator │
                                 └─────────────────────┘
                                      │           │
                  /commands/motor/     │           │  /commands/servo/
                  speed                │           │  position
                                      v           v
                                 ┌─────────────────────┐
                                 │    vesc_driver       │──> /odom
                                 └─────────────────────┘    /sensors/core
                                                            /sensors/imu/raw
```

### Nodes launched in bringup

| Node | Package | Purpose |
|------|---------|---------|
| joy | joy_linux | Reads DS4 via `/dev/input/js0`, publishes `/joy` |
| joy_teleop | joy_teleop | Maps stick axes to `/teleop` (L2 deadman) |
| control_gate | f1tenth_stack | Gates `/teleop` and `/drive` based on button state |
| ackermann_to_vesc | vesc_ackermann | Converts AckermannDrive to VESC motor/servo commands |
| throttle_interpolator | f1tenth_stack | Smooths motor speed and servo position |
| vesc_driver | vesc_driver | Talks to VESC hardware over USB serial |
| vesc_to_odom | vesc_ackermann | Converts VESC sensor data to `/odom` |
| static_tf_publisher | tf2_ros | Publishes `base_link` → `laser` transform |
| odom_zmq_ros2_bridge | (script) | Receives SLAM odometry from ROS1 via ZMQ |

### Key Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/joy` | sensor_msgs/Joy | DS4 → control_gate | Raw button/axis state (~20Hz) |
| `/teleop` | AckermannDriveStamped | joy_teleop → control_gate | Manual RC commands (L2 held) |
| `/drive` | AckermannDriveStamped | traj_planner → control_gate | Autonomous commands |
| `/ackermann_drive` | AckermannDriveStamped | control_gate → vesc chain | Gated output to motors |
| `/odom` | Odometry | vesc_to_odom → anyone | Wheel odometry |
| `/SLAM/Pose` | Odometry | zmq_bridge → traj_planner | SLAM-corrected pose |
| `/scan` | LaserScan | urg_node → anyone | LiDAR (currently disabled) |

### Configuration Files

| File | Purpose |
|------|---------|
| `config/joy_teleop.yaml` | Joystick device, deadzone, axis mappings, deadman buttons |
| `config/vesc.yaml` | VESC serial port, ERPM gains, servo offsets, speed limits |
| `config/sensors.yaml` | LiDAR parameters (urg_node) |

## External Dependencies

1. ackermann_msgs [https://index.ros.org/r/ackermann_msgs/#foxy](https://index.ros.org/r/ackermann_msgs/#foxy).
2. urg_node [https://index.ros.org/p/urg_node/#foxy](https://index.ros.org/p/urg_node/#foxy). This is the driver for Hokuyo LiDARs.
3. joy [https://index.ros.org/p/joy/#foxy](https://index.ros.org/p/joy/#foxy). This is the driver for joysticks in ROS 2.
4. teleop_tools  [https://index.ros.org/p/teleop_tools/#foxy](https://index.ros.org/p/teleop_tools/#foxy). This is the package for teleop with joysticks in ROS 2.
5. vesc [GitHub - f1tenth/vesc at ros2](https://github.com/f1tenth/vesc/tree/ros2). This is the driver for VESCs in ROS 2.

## Package in this repo

1. f1tenth_stack: maintains the bringup launch, control_gate node, throttle_interpolator, and all parameter files
