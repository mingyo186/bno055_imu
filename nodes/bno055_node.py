#!/usr/bin/env python3
"""ROS2 node that reads BNO055 over I2C and publishes Imu + MagneticField + Temperature."""

import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_srvs.srv import Trigger

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
        self.fake_mode = self.get_parameter('fake_mode').value
        self.bus_num   = self.get_parameter('i2c_bus').value
        self.address   = self.get_parameter('device_address').value
        rate           = self.get_parameter('publish_rate').value
        self.frame_id  = self.get_parameter('frame_id').value
        self.op_mode   = self.get_parameter('operation_mode').value
        self.ori_cov   = self.get_parameter('orientation_covariance').value
        self.gyro_cov  = self.get_parameter('angular_velocity_covariance').value
        self.accel_cov = self.get_parameter('linear_acceleration_covariance').value
        self.mag_cov   = self.get_parameter('magnetic_field_covariance').value

        # ── Gyro bias (set by calibration) ────────────────────────
        self.gyro_bias = [0.0, 0.0, 0.0]

        # ── Initialise driver ─────────────────────────────────────
        self._init_driver()

        # ── Publishers + timer ────────────────────────────────────
        self.pub_imu  = self.create_publisher(Imu, 'imu/data', 10)
        self.pub_mag  = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.pub_temp = self.create_publisher(Temperature, 'imu/temp', 10)
        self.timer = self.create_timer(1.0 / rate, self._timer_cb)
        self.get_logger().info(
            f'Publishing on "imu/data", "imu/mag", "imu/temp" @ {rate} Hz')

        # ── Services ──────────────────────────────────────────────
        self.create_service(Trigger, 'imu/calibrate', self._calibrate_cb)
        self.create_service(Trigger, 'imu/reset', self._reset_cb)
        self.get_logger().info(
            'Services: "imu/calibrate", "imu/reset"')

        # ── Parameter change callback ─────────────────────────────
        self.add_on_set_parameters_callback(self._on_param_change)

    # ── Driver init helper ───────────────────────────────────────
    def _init_driver(self):
        if self.fake_mode:
            self.driver = FakeBNO055Driver()
            self.get_logger().info(
                'FAKE MODE enabled — generating random 9-axis IMU data')
        else:
            try:
                self.driver = BNO055Driver(
                    self.bus_num, self.address, self.op_mode)
                cid = self.driver.chip_id()
                self.get_logger().info(
                    f'BNO055 initialised  bus={self.bus_num}  '
                    f'addr=0x{self.address:02X}  CHIP_ID=0x{cid:02X}  '
                    f'mode={self.op_mode}')
            except Exception as e:
                self.get_logger().fatal(f'Failed to open BNO055: {e}')
                raise

    # ── Timer callback ───────────────────────────────────────────
    def _timer_cb(self):
        try:
            (qw, qx, qy, qz), (gx, gy, gz), (ax, ay, az), \
                (mx, my, mz), temp = self.driver.read_all()
        except OSError as e:
            self.get_logger().warn(
                f'I2C read error: {e}', throttle_duration_sec=2.0)
            return

        # Apply gyro bias correction
        gx -= self.gyro_bias[0]
        gy -= self.gyro_bias[1]
        gz -= self.gyro_bias[2]

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

    # ── Service: /imu/calibrate ──────────────────────────────────
    def _calibrate_cb(self, request, response):
        if self.fake_mode:
            response.success = True
            response.message = 'Calibration complete (fake)'
            self.get_logger().info('Calibration requested in fake mode — skipped')
            return response

        self.get_logger().info('Calibrating gyro — collecting data for 2 seconds...')
        samples = []
        end_time = time.monotonic() + 2.0
        while time.monotonic() < end_time:
            try:
                _, (gx, gy, gz), _, _, _ = self.driver.read_all()
                samples.append((gx, gy, gz))
            except OSError:
                pass
            time.sleep(0.01)

        if not samples:
            response.success = False
            response.message = 'Calibration failed — no samples collected'
            return response

        n = len(samples)
        self.gyro_bias[0] = sum(s[0] for s in samples) / n
        self.gyro_bias[1] = sum(s[1] for s in samples) / n
        self.gyro_bias[2] = sum(s[2] for s in samples) / n

        response.success = True
        response.message = (
            f'Calibration complete — {n} samples, '
            f'bias=({self.gyro_bias[0]:.6f}, '
            f'{self.gyro_bias[1]:.6f}, {self.gyro_bias[2]:.6f}) rad/s')
        self.get_logger().info(response.message)
        return response

    # ── Service: /imu/reset ──────────────────────────────────────
    def _reset_cb(self, request, response):
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.driver.close()
        self._init_driver()

        response.success = True
        response.message = 'Sensor reset complete'
        self.get_logger().info(response.message)
        return response

    # ── Runtime parameter change ─────────────────────────────────
    def _on_param_change(self, params):
        for param in params:
            if param.name == 'publish_rate':
                new_rate = param.value
                if new_rate <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='publish_rate must be > 0')
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / new_rate, self._timer_cb)
                self.get_logger().info(f'publish_rate changed to {new_rate} Hz')
        return SetParametersResult(successful=True)


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
