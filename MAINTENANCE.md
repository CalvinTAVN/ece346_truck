# ECE346 Truck Fleet — Maintenance & Update Guide

Step-by-step instructions for updating trucks, configuring SLAM, and running
the full ECE346 stack. Covers all trucks in the fleet.

---

## 1. Updating an Existing Truck

### Quick update (code changes only)

```bash
# SSH into the Jetson
ssh nvidia@192.168.1.2XX   # password: nvidia

cd ~/ece346_truck
git pull origin foxy-devel

# Rebuild only if new packages were added (e.g. racecar_msgs)
source /opt/ros/foxy/setup.bash
colcon build --packages-select racecar_msgs   # skip if no new packages
source install/setup.bash
```

Then restart SLAM and bringup (see §4).

### Full rebuild (after major changes)

```bash
cd ~/ece346_truck
git pull origin foxy-devel
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 2. SLAM Configuration (per truck)

The SLAM config lives on the Jetson at:
```
/home/nvidia/StartUp/src/AprilTagSLAM_ROS/config/config.yaml
```

This file is **not in the ece346_truck repo** — it lives in the SLAM repo
(`~/StartUp`) and must be edited directly on each Jetson.

### 2a. CPU vs GPU detector

```yaml
frontend:
    type: "CPU"    # ← ALWAYS use CPU on JetPack 5.x (R35) trucks
    # type: "GPU"  # ← Only safe on JetPack 4.x (R32) trucks
```

**Check the JetPack version before touching this setting:**
```bash
cat /etc/nv_tegra_release
# R35 → JetPack 5.x → must use CPU
# R32 → JetPack 4.x → GPU is safe
```

> **Why**: The `nvAprilTags` GPU library bundled in the SLAM repo was compiled
> for JetPack 4.4 / CUDA 10.2. Running it on JetPack 5 (CUDA 11.4) causes a
> kernel panic and hard reboot. CPU mode bypasses it entirely.

### 2b. Camera pose offset

This tells SLAM where the ZED camera is relative to the **rear axle** (the
vehicle's reference point for state estimation):

```yaml
backend:
    pose_offset:    [1.0, 0.0, 0.0, -0.38, # camera is 0.38 m forward of rear axle
                    0.0, 1.0, 0.0,  0.0,    # centered on vehicle (no lateral offset)
                    0.0, 0.0, 1.0,  0.0,
                    0.0, 0.0, 0.0,  1.0]
```

| value | meaning |
|---|---|
| `-0.38` (x) | camera is 38 cm forward of the rear axle |
| `0.0` (y) | camera is on the vehicle centerline |
| `0.0` (z) | no height offset (ground vehicle assumption) |

> This value must match the physical camera mount. If the camera is at a
> different position on a specific truck, measure and update accordingly.

### 2c. Prior map path

```yaml
backend:
    prior_map: true
    load_path: ""   # overridden by the launch file — leave blank
```

The launch file (`zed_sdk.launch`) overrides `load_path` with
`$(find tagslam_ros)/config/track_landmark.g2o`. You do not normally need to
change this.

---

## 3. ECE346 Host Laptop Stack (SAFE_ROS2 / ECE346)

The host laptop runs inside Docker and communicates with the truck via
CycloneDDS over WiFi.

### 3a. First-time setup (host)

```bash
# Clone the repo (or use the existing one)
git clone https://github.com/CalvinTAVN/SAFE_ROS2.git
cd SAFE_ROS2

# Build the Docker image
docker compose build

# Start the container
docker compose up -d
docker compose exec ros bash
```

Inside the container:
```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3b. Updating the host stack

```bash
# On the host (outside Docker)
cd /path/to/SAFE_ROS2
git pull origin main

# Inside the container
docker compose exec ros bash
cd /ros2_ws
colcon build --packages-select racecar_ece346 racecar_msgs
source install/setup.bash
```

---

## 4. Running the Full Stack

You need **three terminals** for a complete bring-up.

### Prerequisites

- Truck powered on, Jetson booted.
- Host laptop on the same WiFi as the truck.
- Know your host IP: `hostname -I` (e.g. `192.168.1.100`)
- Know the truck IP: `192.168.1.2XX` (XX = Jetson ID on sticker)
- Know your group's `DOMAIN_ID` (0–101, assigned by TAs; use a unique value
  per group to avoid cross-talk between trucks)

### Terminal 1 — SLAM (on truck via SSH)

```bash
ssh nvidia@192.168.1.2XX
cd ~/ece346_truck
./start_slam.sh
```

Wait until you see:
```
SLAM started.
[ROS1→ZMQ] Odometry bridge running
```

The ZED camera must see AprilTag landmarks to localize. Point the camera at
a wall tag during startup.

### Terminal 2 — F1Tenth stack + ZMQ bridge (on truck via SSH)

```bash
ssh nvidia@192.168.1.2XX
cd ~/ece346_truck
source setup_cyclone.sh <HOST_IP> <DOMAIN_ID>
ros2 launch f1tenth_stack bringup_launch.py
```

**Quick sanity check**: hold L2 and use the left stick — the truck should
respond with manual RC control. If it doesn't, fix the truck side before
proceeding.

### Terminal 3 — ECE346 student stack (host laptop, inside Docker)

```bash
# Start / enter the container
docker compose up -d
docker compose exec ros bash

# Source CycloneDDS (use truck IP and same DOMAIN_ID as Terminal 2)
source /ros2_ws/setup_cyclone.sh 192.168.1.2XX <DOMAIN_ID>

# Verify truck topics are visible
ros2 topic list | grep -E "SLAM|drive|joy"
# expect: /SLAM/Pose, /SLAM/Tag_Detections_Dynamic, /joy, /drive

# Launch the FinalProject truck stack
ros2 launch racecar_ece346 final_project_truck_launch.py
```

---

## 5. AprilTag Obstacle Detection

The obstacle detection pipeline bridges AprilTag detections from ROS1 SLAM
to ROS2 `/Obstacles/Static` for the student's safety filter.

```
TagSLAM (ROS1)
  → /SLAM/Tag_Detections_Dynamic (tagslam_ros/AprilTagDetectionArray)
  → odom_ros1_zmq_pub.py (ZMQ port 5561)
  → [WiFi]
  → odom_ros2_zmq_sub.py (ZMQ port 5561)
  → /SLAM/Tag_Detections_Dynamic (racecar_msgs/AprilTagDetectionArray)
  → obstacle_detection_node.py (host, in final_project_truck_launch.py)
  → /Obstacles/Static (visualization_msgs/MarkerArray)
  → safety_filter_node.py
```

**Obstacle tag IDs**: tags 27–99 (the `ignore_tags` range in `config.yaml`).
These are not used for localization but ARE detected and forwarded as
obstacles.

**Verify the pipeline:**
```bash
# On host (after sourcing cyclone):
ros2 topic hz /SLAM/Tag_Detections_Dynamic   # ~15 Hz when camera sees a tag
ros2 topic echo /Obstacles/Static --once      # cube at the tag's world position
```

---

## 6. Per-Truck Configuration Checklist

Use this checklist when setting up or troubleshooting a specific truck.

| Step | Command / File | Expected result |
|---|---|---|
| Check JetPack | `cat /etc/nv_tegra_release` | R35 → CPU mode; R32 → GPU ok |
| SLAM detector mode | `grep "type:" ~/StartUp/.../config.yaml` | `"CPU"` on R35 trucks |
| pose_offset | `grep -A4 pose_offset ~/StartUp/.../config.yaml` | `-0.38, 0.0` |
| racecar_msgs built | `python3 -c "from racecar_msgs.msg import AprilTagDetectionArray; print('OK')"` | OK |
| SLAM running | `ros2 topic hz /SLAM/Pose` | ~15 Hz |
| Tag bridge running | `ros2 topic hz /SLAM/Tag_Detections_Dynamic` | ~15 Hz (with tags in view) |
| Obstacles flowing | `ros2 topic hz /Obstacles/Static` | ~15 Hz (with obstacle tag in view) |
| Manual RC | L2 + left stick | Truck drives |
| Autonomous gate | R2 held | `/drive` forwarded to motors |

---

## 7. Troubleshooting

### Jetson reboots when starting SLAM

**Cause**: GPU nvAprilTags library (JetPack 4.4) is incompatible with this
truck's CUDA version (JetPack 5.x).

**Fix**: Set `type: "CPU"` in
`/home/nvidia/StartUp/src/AprilTagSLAM_ROS/config/config.yaml`.

---

### `ModuleNotFoundError: No module named 'racecar_msgs'`

**Cause**: `racecar_msgs` not built in the truck's ROS2 workspace.

**Fix**:
```bash
cd ~/ece346_truck
git pull origin foxy-devel   # racecar_msgs is now in the repo
source /opt/ros/foxy/setup.bash
colcon build --packages-select racecar_msgs
source install/setup.bash
```

---

### `/SLAM/Pose` not visible on host

1. Check both machines use the **same** `DOMAIN_ID`.
2. Check both machines are on the same WiFi subnet.
3. Verify the bridge is running: `ps aux | grep zmq` on the Jetson.
4. Re-run `source setup_cyclone.sh <CORRECT_HOST_IP> <DOMAIN_ID>`.

---

### `/SLAM/Tag_Detections_Dynamic` missing

1. `git pull && git checkout HEAD -- slam_tools/odom_ros1_zmq_pub.py slam_tools/odom_ros2_zmq_sub.py` — in case files were manually reverted.
2. Restart `start_slam.sh` and bringup.
3. Hold a tag (ID 27–99) in front of the camera and check again.

---

### SLAM never gets past initialization

**Symptom**: Output stops at `CUDA Apriltag Detector Initialized.` or SLAM
waits forever without publishing `/SLAM/Pose`.

**Fixes**:
- Ensure `type: "CPU"` if on JetPack 5.x.
- Point the camera at a known landmark tag so SLAM can localize against the
  prior map.
- Check that `track_landmark.g2o` exists:
  `ls ~/StartUp/src/AprilTagSLAM_ROS/config/track_landmark.g2o`

---

### Truck doesn't move in teleop

1. Verify PS4 controller is connected: `ls /dev/input/js0`
2. Verify bringup is running: `ros2 node list | grep control_gate`
3. Hold **L2** (not R2) for teleop mode.
4. Check VESC USB cable is connected.
