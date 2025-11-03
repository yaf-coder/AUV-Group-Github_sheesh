# Autonomous Underwater Vehicle (AUV) Navigation System

A ROS 2 package for autonomous underwater vehicle navigation and control, designed for the BlueROV2 platform. This system implements computer vision-based tracking, PID control, and autonomous behaviors for submarine competition scenarios.

## Overview

This project implements an autonomous navigation system for underwater vehicles with the following capabilities:
- **Vision-based tracking** using YOLOv8 for object detection
- **AprilTag detection** for precise distance estimation
- **PID control** for depth and heading stabilization
- **State machine** for autonomous scanning and chasing behaviors
- **MAVROS integration** for BlueROV2 communication
- **Real-time camera processing** for underwater computer vision

## Features

### Control Systems
- **PID Depth Controller**: Maintains precise depth control with tunable PID parameters
- **PID Heading Controller**: Accurate yaw control for navigation
- **Manual Control Override**: Direct control via MAVROS ManualControl messages

### Computer Vision
- **YOLOv8 Integration**: Custom-trained model for underwater object detection
- **AprilTag Detection**: Precise distance and pose estimation
- **Real-time Processing**: Camera subscriber with image processing pipeline

### Autonomous Behaviors
- **Scanning Mode**: Search pattern with controlled depth and heading
- **Chase Mode**: Vision-guided pursuit of detected targets
- **Light Flashing**: Automated light control for signaling

## System Requirements

### Hardware
- BlueROV2 or compatible ROV
- Onboard computer (Raspberry Pi 4, NVIDIA Jetson, or equivalent)
- Camera system
- Pressure sensor for depth measurement

### Software
- Ubuntu 20.04 or later
- ROS 2 (Foxy, Humble, or later)
- Python 3.8+
- ArduSub firmware

## Installation

### 1. Install Dependencies

```bash
# Install ROS 2 (if not already installed)
# Follow official ROS 2 installation guide for your distribution

# Install MAVROS
sudo apt install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras

# Install Python dependencies
pip install ultralytics opencv-python matplotlib numpy
```

### 2. Clone and Build

```bash
# Create workspace
mkdir -p ~/auvc_ws/src
cd ~/auvc_ws/src

# Clone repository
git clone https://github.com/YOUR_USERNAME/AUV-Group-Github.git

# Build workspace
cd ~/auvc_ws
colcon build --symlink-install

# Source workspace
source ~/auvc_ws/install/setup.bash
```

### 3. Setup ArduSub SITL (for simulation)

```bash
# Clone ArduPilot
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive

# Run SITL
cd ~/ardupilot
Tools/autotest/sim_vehicle.py --vehicle=ArduSub --aircraft="bwsibot" -L RATBeach --out=udp:YOUR_COMPUTER_IP:14550
```

## Usage

### Launch Files

The package includes several launch configurations:

#### Full System Launch
```bash
ros2 launch intro_to_ros testitall.yaml
```
Launches the complete autonomous system with PID controllers, YOLO detection, and state machine.

#### SITL Simulation
```bash
ros2 launch intro_to_ros sitl.yaml
```
Runs the system in Software-In-The-Loop simulation mode.

#### PID Depth Test
```bash
ros2 launch intro_to_ros PID_depth_test.yaml
```
Tests depth control independently.

#### PID Heading Test
```bash
ros2 launch intro_to_ros PID_heading_test.yaml
```
Tests heading control independently.

#### YOLO Detection Only
```bash
ros2 launch intro_to_ros YOLO.yaml
```
Runs computer vision system only for testing.

### Manual Control

#### Set Desired Depth
```bash
ros2 topic pub /PID/desired_depth mavros_msgs/msg/Altitude "{relative: 0.8}"
```

#### Set Desired Heading
```bash
ros2 topic pub /img/desired_heading std_msgs/msg/Int16 "{data: 45}"
```

#### Target Detection Override
```bash
ros2 topic pub /img/targetted std_msgs/msg/Bool "{data: true}"
```

## ROS 2 Topics

### Subscribed Topics
- `/bluerov2/depth` (mavros_msgs/Altitude) - Current depth measurement
- `/bluerov2/heading` (std_msgs/Int16) - Current heading in degrees
- `/bluerov2/camera/image_raw` (sensor_msgs/Image) - Camera feed
- `/img/desired_heading` (std_msgs/Int16) - Target heading from vision
- `/img/distance` (std_msgs/Float32) - Distance to detected target
- `/img/targetted` (std_msgs/Bool) - Target acquisition status

### Published Topics
- `/bluerov2/manual_control` (mavros_msgs/ManualControl) - Movement commands
- `/bluerov2/override_rc` (mavros_msgs/OverrideRCIn) - Direct RC override (lights)
- `/PID/desired_depth` (mavros_msgs/Altitude) - Depth setpoint for PID
- `/PID/desired_heading` (std_msgs/Int16) - Heading setpoint for PID

## Node Descriptions

### PID Controllers

#### `pid_depth`
PID controller for depth stabilization with anti-windup and output clamping.

**Tunable Parameters** (in `intro_to_ros/pid_depth.py`):
```python
self.kp = 47    # Proportional gain
self.ki = 7     # Integral gain
self.kd = 18    # Derivative gain
```

#### `pid_heading`
PID controller for heading/yaw stabilization.

### Movement and State Machine

#### `movement`
Implements autonomous behaviors with state machine:
- **SCANNING**: Search pattern at minimum depth
- **CHASE**: Pursuit mode when target detected
- Automatic state transitions based on vision feedback

### Vision Processing

#### `camera_subscriber`
Processes camera feed for computer vision pipeline.

#### `YOLOmovement`
Integrates YOLO detection with movement control.

#### `YOLO_subscriber`
Subscribes to camera feed and runs YOLO inference.

### Utilities

#### `armdisarm`
Arms/disarms the vehicle for safety.

#### `pressure_to_depth`
Converts pressure sensor readings to depth measurements.

## Project Structure

```
AUV-Group-Github/
├── intro_to_ros/              # Main package directory
│   ├── armdisarm.py          # Arm/disarm control
│   ├── camera_subscriber.py  # Camera processing
│   ├── movement.py           # State machine and behaviors
│   ├── pid_depth.py          # Depth PID controller
│   ├── pid_heading.py        # Heading PID controller
│   ├── pressure_to_depth.py  # Sensor conversion
│   ├── yolo.py               # YOLO detection utilities
│   ├── YOLOmovement.py       # YOLO-based movement
│   └── YOLO_subscriber.py    # YOLO ROS integration
├── launch/                    # Launch configurations
│   ├── testitall.yaml        # Full system launch
│   ├── sitl.yaml             # Simulation launch
│   ├── PID_depth_test.yaml   # Depth control test
│   ├── PID_heading_test.yaml # Heading control test
│   └── YOLO.yaml             # Vision-only test
├── test/                      # Unit tests
├── package.xml               # ROS 2 package manifest
├── setup.py                  # Python package setup
└── README.md                 # This file
```

## Configuration and Tuning

### PID Tuning

The PID controllers have been tuned for specific vehicle configurations. If using different hardware:

1. **Start Conservative**: Begin with low gains
2. **Tune Proportional**: Increase Kp until oscillation, then reduce by 50%
3. **Add Integral**: Slowly increase Ki to eliminate steady-state error
4. **Add Derivative**: Increase Kd to reduce overshoot and improve response

### Depth Control Limits
```python
MIN_DEPTH = 0.4  # meters
MAX_DEPTH = 0.8  # meters
```

### YOLO Model
The system uses a custom-trained YOLOv8 model (`best_ncnn_model`). To retrain:
1. Collect underwater imagery
2. Annotate with your target classes
3. Train using Ultralytics YOLO
4. Replace model in `intro_to_ros/` directory

## Development Workflow

### Building After Changes
```bash
cd ~/auvc_ws
colcon build --symlink-install
source install/setup.bash
```

### Running Tests
```bash
cd ~/auvc_ws
colcon test
colcon test-result --verbose
```

### Debugging
```bash
# List all active topics
ros2 topic list

# Echo specific topic
ros2 topic echo /bluerov2/depth

# Check node info
ros2 node info /pid_depth

# View topic data rate
ros2 topic hz /bluerov2/camera/image_raw
```

## Safety Considerations

1. **Always test in simulation first** using SITL before deploying to hardware
2. **Set depth limits** appropriate for your operating environment
3. **Monitor battery levels** during operation
4. **Have a kill switch** or emergency surface procedure
5. **Test in controlled environment** before competition/deployment
6. **Verify sensor calibration** before each mission

## Troubleshooting

### MAVROS Connection Issues
```bash
# Check MAVROS status
ros2 topic echo /mavros/state

# Verify connection string in launch file
# Default: udp://:14550@YOUR_IP:14551
```

### Camera Not Publishing
```bash
# List camera topics
ros2 topic list | grep camera

# Check camera permissions
sudo usermod -a -G video $USER
```

### PID Oscillation
- Reduce Kp gain by 20-30%
- Ensure derivative term is not too high
- Check for sensor noise/filtering

## Contributing

Team Members:
- Aidan Gao (agao3019@gmail.com)
- Andy
- Ken
- Roneet
- Shamak

## License

TODO: Add license information

## Acknowledgments

- Built with ROS 2 and MAVROS
- Computer vision powered by Ultralytics YOLOv8
- Designed for BlueROV2 platform
- ArduSub flight controller

## Resources

- [ROS 2 Documentation](https://docs.ros.org)
- [MAVROS Documentation](http://wiki.ros.org/mavros)
- [ArduSub Documentation](https://www.ardusub.com/)
- [BlueROV2 Technical Details](https://bluerobotics.com/store/rov/bluerov2/)
- [YOLOv8 Documentation](https://docs.ultralytics.com/)

---

**For questions or issues, please contact the maintainer at agao3019@gmail.com**
