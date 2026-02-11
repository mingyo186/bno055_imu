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

## Why this package?

There is an existing ROS2 BNO055 driver ([flynneva/bno055](https://github.com/flynneva/bno055)). This package takes a different approach:

| Feature | flynneva/bno055 | This package |
|---|---|---|
| **fake_mode** | Not supported | Supported (default `true`) |
| **Calibrate / Reset services** | Read-only (returns current offsets) | Auto-calibrate (2 s sampling) + reset |
| **Runtime parameter change** | Not supported | `publish_rate` changeable via `ros2 param set` |
| **ROS2 Jazzy** | Not explicitly tested | Tested on Jazzy |
| **I2C library** | `smbus` | `smbus2` |
| **UART support** | Yes | No (I2C only) |
| **Axis remapping** | Yes (P0–P7) | No |
| **Topics** | 6 (imu, imu_raw, mag, grav, temp, calib_status) | 3 (imu/data, imu/mag, imu/temp) |
| **colcon test** | No launch tests | Full launch tests (28 tests, 0 failures) |

**Choose this package** if you need fake_mode for CI/testing, simple calibration services, or a lightweight Jazzy-tested driver.
**Choose flynneva/bno055** if you need UART support or axis remapping.

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
├── nodes/
│   └── bno055_node.py
└── test/
    └── test_bno055_node.py
```

## Test Results

Tested on Ubuntu 24.04 (WSL2) with `fake_mode: true`.

```
$ colcon test --packages-select bno055_imu
$ colcon test-result --verbose
Summary: 28 tests, 0 errors, 0 failures, 0 skipped
```

| Test Category | Test | Result |
|---|---|---|
| **Topics** | `imu/data` publishes `sensor_msgs/Imu` | PASS |
| **Topics** | `imu/mag` publishes `sensor_msgs/MagneticField` | PASS |
| **Topics** | `imu/temp` publishes `sensor_msgs/Temperature` | PASS |
| **Topics** | `frame_id == "imu_link"` | PASS |
| **Topics** | Temperature in range 10-40 °C | PASS |
| **Services** | `imu/calibrate` returns `success=True` | PASS |
| **Services** | `imu/reset` returns `success=True` | PASS |
| **Parameters** | `publish_rate` runtime change to 20.0 Hz | PASS |
| **Shutdown** | Clean exit (code 0, -2, or -15) | PASS |
| **Linting** | pep257, flake8, copyright, xmllint | PASS |

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
