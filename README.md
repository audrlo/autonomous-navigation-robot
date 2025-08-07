# Autonomous Navigation Robot

A complete autonomous navigation robot system using ROS2 Humble, Intel RealSense D455, RTAB-Map SLAM, and RoboClaw 2x30A motor controller. Designed for easy deployment to Raspberry Pi 5 via Docker.

## Features

- **Autonomous Navigation** - Full Nav2 stack with path planning and following
- **SLAM Mapping** - Real-time mapping using RTAB-Map with Intel RealSense D455
- **Obstacle Avoidance** - Real-time obstacle detection within 2 feet using depth data
- **Motor Control** - RoboClaw 2x30A integration for differential drive
- **Containerized** - Complete Docker setup for easy deployment
- **Raspberry Pi 5 Ready** - ARM64 support with optimized build

## Hardware Requirements

- **Raspberry Pi 5** (8GB recommended)
- **Intel RealSense D455** camera
- **RoboClaw 2x30A** motor controller
- **Differential drive robot platform**
- **USB cables** for RealSense and RoboClaw connections

## Quick Start

### 1. Development Setup

```bash
# Clone and build locally
git clone <your-repo>
cd playground

# Build the Docker image
docker-compose build

# Run for development (with RViz)
docker-compose --profile dev up
```

### 2. Deploy to Raspberry Pi 5

```bash
# Deploy to your Raspberry Pi (replace with your Pi's address)
./deploy.sh robot@raspberrypi.local
```

### 3. Hardware Connections

- Connect RealSense D455 to USB 3.0 port
- Connect RoboClaw to `/dev/ttyACM0` (usually automatic)
- Ensure proper power supply for motors

### 4. Launch Robot

On the Raspberry Pi:
```bash
# Start the robot system
docker-compose up -d robot

# Monitor logs
docker-compose logs -f robot
```

## System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Intel RealSense│───▶│   RTAB-Map SLAM  │───▶│  Nav2 Planning  │
│      D455       │    │    + Mapping     │    │   + Following   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                │                        │
┌─────────────────┐    ┌──────────────────┐             │
│ Obstacle        │◀───│  Depth Processing│◀────────────┘
│ Avoidance       │    │   + Detection    │
└─────────────────┘    └──────────────────┘
         │
         ▼
┌─────────────────┐    ┌──────────────────┐
│  Velocity       │───▶│    RoboClaw      │
│  Commands       │    │  Motor Driver    │
└─────────────────┘    └──────────────────┘
```

## Configuration

### Robot Parameters

Edit in `src/robot_bringup/launch/robot.launch.py`:

```python
# RoboClaw settings
'wheel_separation': 0.5,    # Distance between wheels (meters)
'wheel_radius': 0.1,        # Wheel radius (meters)
'max_speed': 1.0,           # Maximum speed (m/s)
'ticks_per_meter': 1000,    # Encoder ticks per meter
```

### Navigation Parameters

Tune navigation in `src/robot_bringup/config/nav2_params.yaml`:

- **Robot dimensions**: `robot_radius`
- **Speed limits**: `max_vel_x`, `max_vel_theta`
- **Obstacle detection**: `obstacle_max_range`

### Obstacle Avoidance

Adjust in obstacle avoidance node:

```python
'min_distance': 0.6,          # 2 feet obstacle threshold
'max_linear_velocity': 0.5,   # Speed limit when avoiding
'safety_factor': 1.2,         # Safety margin multiplier
```

## ROS2 Topics

### Published
- `/odom` - Robot odometry
- `/map` - SLAM-generated map
- `/cmd_vel` - Final velocity commands
- `/joint_states` - Wheel joint states

### Subscribed
- `/camera/camera/color/image_raw` - RGB camera feed
- `/camera/camera/aligned_depth_to_color/image_raw` - Aligned depth
- `/camera/camera/depth/color/points` - Point cloud
- `/cmd_vel_nav` - Navigation velocity commands

## Package Structure

```
src/
├── robot_bringup/          # Main launch and configuration
│   ├── launch/
│   │   └── robot.launch.py # Main launch file
│   ├── config/
│   │   └── nav2_params.yaml # Navigation parameters
│   └── robot_bringup/
│       └── obstacle_avoidance.py # Obstacle avoidance node
└── roboclaw_driver/        # RoboClaw motor controller
    ├── roboclaw_driver/
    │   └── roboclaw_node.py # Motor driver node
    └── package.xml
```

## Troubleshooting

### Common Issues

1. **RealSense not detected**
   ```bash
   # Check USB connection
   lsusb | grep Intel
   
   # Verify permissions
   ls -la /dev/video*
   ```

2. **RoboClaw communication failure**
   ```bash
   # Check serial connection
   ls -la /dev/ttyACM*
   
   # Test baud rate (default: 38400)
   ```

3. **Navigation not working**
   - Ensure odometry is publishing: `ros2 topic echo /odom`
   - Check TF tree: `ros2 run tf2_tools view_frames`
   - Verify costmaps: `ros2 topic echo /local_costmap/costmap`

### Debugging

```bash
# Enter container for debugging
docker exec -it robot_container bash

# Check ROS2 nodes
ros2 node list

# Monitor specific topics
ros2 topic echo /odom
ros2 topic echo /scan

# Check transforms
ros2 run tf2_ros tf2_echo base_link camera_link
```

## Development

### Adding New Features

1. **Create new package**:
   ```bash
   cd src
   ros2 pkg create --build-type ament_cmake my_package
   ```

2. **Rebuild container**:
   ```bash
   docker-compose build
   ```

3. **Test locally**:
   ```bash
   docker-compose --profile dev up
   ```

### Debugging with RViz

1. **Start with visualization**:
   ```bash
   docker-compose --profile dev up
   ```

2. **Key RViz displays**:
   - `/map` - Current SLAM map
   - `/local_costmap/costmap` - Local planning costmap
   - `/global_costmap/costmap` - Global planning costmap
   - `/camera/camera/depth/color/points` - Depth point cloud

## Performance Optimization

### Raspberry Pi 5 Tuning

The system is optimized for Raspberry Pi 5:

- **CPU frequency**: Set to performance mode
- **Memory**: Reduced arena allocation (`MALLOC_ARENA_MAX=2`)
- **Parallel builds**: Optimized for 4 cores
- **Buffer sizes**: Tuned for real-time performance

### Runtime Optimization

```bash
# Set CPU governor to performance
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Increase GPU memory split
echo "gpu_mem=128" | sudo tee -a /boot/config.txt
```

## License

MIT License - feel free to modify and distribute!

## Contributing

1. Fork the repository
2. Create a feature branch
3. Test on hardware
4. Submit a pull request

For questions or issues, please open a GitHub issue.
