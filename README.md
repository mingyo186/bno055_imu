# bno055_imu

ROS 2 Jazzy driver for the Bosch BNO055 9-axis absolute orientation IMU over I2C.

## Features

- Publishes `sensor_msgs/Imu` on the `imu/data` topic (includes fused quaternion orientation)
- Publishes `sensor_msgs/MagneticField` on the `imu/mag` topic
- Publishes `sensor_msgs/Temperature` on the `imu/temp` topic
- **Fake mode** — generates random 9-axis IMU data without physical hardware
- Built-in sensor fusion: absolute orientation as quaternion (NDOF mode)
- Configurable operation mode: NDOF, IMU, Compass, M4G
- Calibration status readable from the BNO055

## Prerequisites

- ROS 2 Jazzy
- Python 3
- `smbus2` (only required when `fake_mode` is `false`)

```bash
pip3 install smbus2
```

## Installation

```bash
cd ~/ros2_ws
colcon build --packages-select bno055_imu
source install/setup.bash
```

## Usage

### Launch with default parameters (fake mode)

```bash
ros2 launch bno055_imu bno055_launch.py
```

### Run the node directly

```bash
ros2 run bno055_imu bno055_node.py --ros-args -p fake_mode:=true
```

### Run with real hardware

```bash
ros2 run bno055_imu bno055_node.py --ros-args -p fake_mode:=false
```

### Override parameters via YAML

```bash
ros2 launch bno055_imu bno055_launch.py params_file:=/path/to/your_params.yaml
```

### Verify output

```bash
ros2 topic echo /imu/data
ros2 topic echo /imu/mag
ros2 topic echo /imu/temp
```

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `fake_mode` | bool | `true` | `true`: generate random data, `false`: read from real I2C device |
| `i2c_bus` | int | `1` | I2C bus number (`/dev/i2c-N`) |
| `device_address` | int | `0x28` | BNO055 I2C address (`0x28` or `0x29`) |
| `publish_rate` | double | `100.0` | Publishing rate in Hz |
| `frame_id` | string | `imu_link` | TF frame ID in message headers |
| `operation_mode` | string | `ndof` | Fusion mode: `ndof`, `imu`, `compass`, `m4g` |
| `orientation_covariance` | double | `0.01` | Diagonal orientation covariance (rad²) |
| `angular_velocity_covariance` | double | `0.02` | Diagonal angular velocity covariance (rad²/s²) |
| `linear_acceleration_covariance` | double | `0.04` | Diagonal acceleration covariance (m²/s⁴) |
| `magnetic_field_covariance` | double | `0.0` | Diagonal magnetic field covariance (T²), 0 = unknown |

## Operation Modes

| Mode | Sensors Used | Description |
|---|---|---|
| `ndof` | Accel + Gyro + Mag | 9-axis absolute orientation (default) |
| `imu` | Accel + Gyro | 6-axis relative orientation (no magnetometer) |
| `compass` | Accel + Mag | Heading only |
| `m4g` | Accel + Mag | Rotation from magnetometer, tilt from accelerometer |

## Package Structure

```
bno055_imu/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── bno055_params.yaml
├── launch/
│   └── bno055_launch.py
├── bno055_imu/
│   ├── __init__.py
│   └── bno055_driver.py
└── nodes/
    └── bno055_node.py
```

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
