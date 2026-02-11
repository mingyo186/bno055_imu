#!/usr/bin/env python3
"""ROS2 node that reads BNO055 over I2C and publishes Imu + MagneticField + Temperature."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature

from bno055_imu.bno055_driver import BNO055Driver, FakeBNO055Driver


class BNO055ImuNode(Node):
    def __init__(self):
        super().__init__('bno055_imu_node')

        # ── Declare parameters ────────────────────────────────────
        self.declare_parameter('fake_mode', True)
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('device_address', 0x28)
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('operation_mode', 'ndof')
        self.declare_parameter('orientation_covariance', 0.01)
        self.declare_parameter('angular_velocity_covariance', 0.02)
        self.declare_parameter('linear_acceleration_covariance', 0.04)
        self.declare_parameter('magnetic_field_covariance', 0.0)

        # ── Read parameters ───────────────────────────────────────
        fake_mode = self.get_parameter('fake_mode').value
        bus       = self.get_parameter('i2c_bus').value
        address   = self.get_parameter('device_address').value
        rate      = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        op_mode   = self.get_parameter('operation_mode').value
        self.ori_cov   = self.get_parameter('orientation_covariance').value
        self.gyro_cov  = self.get_parameter('angular_velocity_covariance').value
        self.accel_cov = self.get_parameter('linear_acceleration_covariance').value
        self.mag_cov   = self.get_parameter('magnetic_field_covariance').value

        # ── Initialise driver ─────────────────────────────────────
        if fake_mode:
            self.driver = FakeBNO055Driver()
            self.get_logger().info(
                'FAKE MODE enabled — generating random 9-axis IMU data')
        else:
            try:
                self.driver = BNO055Driver(bus, address, op_mode)
                cid = self.driver.chip_id()
                self.get_logger().info(
                    f'BNO055 initialised  bus={bus}  addr=0x{address:02X}  '
                    f'CHIP_ID=0x{cid:02X}  mode={op_mode}')
            except Exception as e:
                self.get_logger().fatal(f'Failed to open BNO055: {e}')
                raise

        # ── Publishers + timer ────────────────────────────────────
        self.pub_imu  = self.create_publisher(Imu, 'imu/data', 10)
        self.pub_mag  = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.pub_temp = self.create_publisher(Temperature, 'imu/temp', 10)
        self.timer = self.create_timer(1.0 / rate, self._timer_cb)
        self.get_logger().info(
            f'Publishing on "imu/data", "imu/mag", "imu/temp" @ {rate} Hz')

    # ──────────────────────────────────────────────────────────────
    def _timer_cb(self):
        try:
            (qw, qx, qy, qz), (gx, gy, gz), (ax, ay, az), \
                (mx, my, mz), temp = self.driver.read_all()
        except OSError as e:
            self.get_logger().warn(
                f'I2C read error: {e}', throttle_duration_sec=2.0)
            return

        stamp = self.get_clock().now().to_msg()

        # ── sensor_msgs/Imu ──────────────────────────────────────
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.frame_id

        imu_msg.orientation.w = qw
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        oc = self.ori_cov
        imu_msg.orientation_covariance = [
            oc,  0.0, 0.0,
            0.0, oc,  0.0,
            0.0, 0.0, oc,
        ]

        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz
        gc = self.gyro_cov
        imu_msg.angular_velocity_covariance = [
            gc,  0.0, 0.0,
            0.0, gc,  0.0,
            0.0, 0.0, gc,
        ]

        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        ac = self.accel_cov
        imu_msg.linear_acceleration_covariance = [
            ac,  0.0, 0.0,
            0.0, ac,  0.0,
            0.0, 0.0, ac,
        ]
        self.pub_imu.publish(imu_msg)

        # ── sensor_msgs/MagneticField ────────────────────────────
        mag_msg = MagneticField()
        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = self.frame_id
        mag_msg.magnetic_field.x = mx
        mag_msg.magnetic_field.y = my
        mag_msg.magnetic_field.z = mz
        mc = self.mag_cov
        mag_msg.magnetic_field_covariance = [
            mc,  0.0, 0.0,
            0.0, mc,  0.0,
            0.0, 0.0, mc,
        ]
        self.pub_mag.publish(mag_msg)

        # ── sensor_msgs/Temperature ──────────────────────────────
        temp_msg = Temperature()
        temp_msg.header.stamp = stamp
        temp_msg.header.frame_id = self.frame_id
        temp_msg.temperature = temp
        temp_msg.variance = 0.0
        self.pub_temp.publish(temp_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BNO055ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.driver.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
