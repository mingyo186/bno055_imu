# Copyright 2025 The bno055_imu Authors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""BNO055 I2C Driver - 9-axis absolute orientation IMU with sensor fusion."""

import math
import random
import struct
import time


class FakeBNO055Driver:
    """Fake driver that generates random orientation/IMU data without I2C hardware."""

    def __init__(self, **kwargs):
        self._yaw = 0.0

    def read_all(self):
        """Return (quaternion, gyro, accel, mag, temp) with fake data."""
        # Slowly rotating yaw for a realistic fake quaternion
        self._yaw += random.gauss(0.01, 0.005)
        half = self._yaw / 2.0
        qw = math.cos(half)
        qx = random.gauss(0.0, 0.01)
        qy = random.gauss(0.0, 0.01)
        qz = math.sin(half)
        norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm

        gx = random.gauss(0.0, 0.01)
        gy = random.gauss(0.0, 0.01)
        gz = random.gauss(0.0, 0.01)

        ax = random.gauss(0.0, 0.3)
        ay = random.gauss(0.0, 0.3)
        az = 9.80665 + random.gauss(0.0, 0.3)

        mx = random.gauss(25.0e-6, 1.0e-6)
        my = random.gauss(0.0, 1.0e-6)
        mz = random.gauss(-45.0e-6, 1.0e-6)

        temp = 25.0 + random.gauss(0.0, 0.5)

        return (qw, qx, qy, qz), (gx, gy, gz), (ax, ay, az), (mx, my, mz), temp

    def chip_id(self) -> int:
        """Return fake chip ID 0xA0."""
        return 0xA0

    def calibration_status(self):
        """Return fully-calibrated status for all subsystems."""
        return {'sys': 3, 'gyro': 3, 'accel': 3, 'mag': 3}

    def close(self):
        """Close the fake driver (no-op)."""
        pass


class BNO055Driver:
    """Low-level I2C driver for Bosch BNO055."""

    # -- Register Map -------------------------------------------------------
    REG_CHIP_ID = 0x00
    REG_PAGE_ID = 0x07
    REG_TEMP = 0x34
    REG_CALIB_STAT = 0x35
    REG_OPR_MODE = 0x3D
    REG_PWR_MODE = 0x3E
    REG_SYS_TRIGGER = 0x3F
    REG_UNIT_SEL = 0x3B

    # Fusion data registers (page 0)
    REG_QUA_DATA_W_LSB = 0x20
    REG_GYR_DATA_X_LSB = 0x14
    REG_ACC_DATA_X_LSB = 0x08
    REG_MAG_DATA_X_LSB = 0x0E

    # -- Operation Modes ----------------------------------------------------
    MODE_CONFIG = 0x00
    MODE_NDOF = 0x0C
    MODE_IMU = 0x08
    MODE_COMPASS = 0x09
    MODE_M4G = 0x0A

    MODE_MAP = {
        'ndof': 0x0C,
        'imu': 0x08,
        'compass': 0x09,
        'm4g': 0x0A,
    }

    # -- Unit scales --------------------------------------------------------
    # With default unit selection (SI):
    #   Accel: m/s^2, 1 LSB = 0.01 m/s^2
    #   Gyro:  rad/s, 1 LSB = 1/900 rad/s  (= 1/16 dps -> rad)
    #   Mag:   uT, 1 LSB = 1/16 uT
    #   Quaternion: 1 LSB = 1/16384 (dimensionless, unit quaternion)
    #   Temp:  C, 1 LSB = 1 C
    ACCEL_SCALE = 0.01
    GYRO_SCALE = 1.0 / 900.0
    MAG_SCALE = 1.0 / 16.0e6
    QUAT_SCALE = 1.0 / 16384.0

    def __init__(self, bus: int = 1, address: int = 0x28,
                 operation_mode: str = 'ndof'):
        from smbus2 import SMBus

        self.address = address
        self.bus = SMBus(bus)

        self._init_device(operation_mode)

    # -- Initialisation -----------------------------------------------------
    def _init_device(self, operation_mode: str):
        """Initialize the BNO055 device with the given operation mode."""
        # Switch to config mode
        self.bus.write_byte_data(self.address, self.REG_OPR_MODE, self.MODE_CONFIG)
        time.sleep(0.025)

        # Reset
        self.bus.write_byte_data(self.address, self.REG_SYS_TRIGGER, 0x20)
        time.sleep(0.7)

        # Wait for chip ID to be readable after reset
        for _ in range(10):
            try:
                cid = self.bus.read_byte_data(self.address, self.REG_CHIP_ID)
                if cid == 0xA0:
                    break
            except OSError:
                pass
            time.sleep(0.1)

        # Normal power mode
        self.bus.write_byte_data(self.address, self.REG_PWR_MODE, 0x00)
        time.sleep(0.01)

        # Page 0
        self.bus.write_byte_data(self.address, self.REG_PAGE_ID, 0x00)

        # Unit selection: SI units, Celsius, rad/s, m/s^2
        #   bit7=ORI_Android(0), bit4=TEMP_C(0), bit2=EUL_Rad(1),
        #   bit1=GYR_Rps(1), bit0=ACC_ms2(0)
        self.bus.write_byte_data(self.address, self.REG_UNIT_SEL, 0x06)
        time.sleep(0.01)

        # Switch to requested fusion mode
        mode_val = self.MODE_MAP.get(operation_mode, self.MODE_NDOF)
        self.bus.write_byte_data(self.address, self.REG_OPR_MODE, mode_val)
        time.sleep(0.025)

    # -- Burst read helpers -------------------------------------------------
    def _read_block(self, reg: int, length: int) -> bytes:
        """Read a block of bytes from the given register."""
        return bytes(self.bus.read_i2c_block_data(self.address, reg, length))

    @staticmethod
    def _unpack_int16(data: bytes, offset: int) -> int:
        """Unpack a little-endian signed 16-bit integer from data at offset."""
        return struct.unpack_from('<h', data, offset)[0]

    # -- Public API ---------------------------------------------------------
    def read_all(self):
        """Read quaternion, gyro, accel, mag, and temperature."""
        # Quaternion (8 bytes)
        qbuf = self._read_block(self.REG_QUA_DATA_W_LSB, 8)
        qw = self._unpack_int16(qbuf, 0) * self.QUAT_SCALE
        qx = self._unpack_int16(qbuf, 2) * self.QUAT_SCALE
        qy = self._unpack_int16(qbuf, 4) * self.QUAT_SCALE
        qz = self._unpack_int16(qbuf, 6) * self.QUAT_SCALE

        # Gyroscope (6 bytes)
        gbuf = self._read_block(self.REG_GYR_DATA_X_LSB, 6)
        gx = self._unpack_int16(gbuf, 0) * self.GYRO_SCALE
        gy = self._unpack_int16(gbuf, 2) * self.GYRO_SCALE
        gz = self._unpack_int16(gbuf, 4) * self.GYRO_SCALE

        # Accelerometer (6 bytes)
        abuf = self._read_block(self.REG_ACC_DATA_X_LSB, 6)
        ax = self._unpack_int16(abuf, 0) * self.ACCEL_SCALE
        ay = self._unpack_int16(abuf, 2) * self.ACCEL_SCALE
        az = self._unpack_int16(abuf, 4) * self.ACCEL_SCALE

        # Magnetometer (6 bytes)
        mbuf = self._read_block(self.REG_MAG_DATA_X_LSB, 6)
        mx = self._unpack_int16(mbuf, 0) * self.MAG_SCALE
        my = self._unpack_int16(mbuf, 2) * self.MAG_SCALE
        mz = self._unpack_int16(mbuf, 4) * self.MAG_SCALE

        # Temperature (1 byte, signed)
        temp = struct.unpack('b', self._read_block(self.REG_TEMP, 1))[0]

        return (qw, qx, qy, qz), (gx, gy, gz), (ax, ay, az), (mx, my, mz), float(temp)

    def calibration_status(self) -> dict:
        """Read calibration status for sys, gyro, accel, mag (0-3 each)."""
        val = self.bus.read_byte_data(self.address, self.REG_CALIB_STAT)
        return {
            'sys': (val >> 6) & 0x03,
            'gyro': (val >> 4) & 0x03,
            'accel': (val >> 2) & 0x03,
            'mag': val & 0x03,
        }

    def chip_id(self) -> int:
        """Read CHIP_ID register (should return 0xA0 for genuine BNO055)."""
        return self.bus.read_byte_data(self.address, self.REG_CHIP_ID)

    def close(self):
        """Close the underlying SMBus connection."""
        self.bus.close()
