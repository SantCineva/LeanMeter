from machine import I2C, Pin
import math

class MPU6050():
    # MPU6050 address and registers
    MPU6050_ADDR = 0x68
    MPU6050_PWR_MGMT_1 = 0x6B

    # Accelerometer Full-Scale Range:
    # ±2g:  0
    # ±4g:  1
    # ±8g:  2
    # ±16g: 3
    MPU6050_ACCEL_RANGE_CONFIG = 0x1C

    # Gyroscope Full-Scale Range:
    # ±250  degrees per second (dps): 0
    # ±500  dps: 1
    # ±1000 dps: 2
    # ±2000 dps: 3
    MPU6050_GYRO_RANGE_CONFIG = 0x1B

    # Sensitivity values
    # Accelerometer Sensitivity:
    # ±2g: 16384 LSB/g
    # ±4g: 8192 LSB/g
    # ±8g: 4096 LSB/g
    # ±16g: 2048 LSB/g
    MPU6050_ACC_LSBG = 16384.0

    # Gyroscope Sensitivity:
    # ±250 degrees per second (dps): 131 LSB/dps
    # ±500 dps: 65.5 LSB/dps
    # ±1000 dps: 32.8 LSB/dps
    # ±2000 dps: 16.4 LSB/dps
    MPU6050_GYRO_LSBDPS = 131.0

    # Addresses for the Accelerometer value composition which are split into a high and a low bit
    ACCEL_X_H = 0x3B
    ACCEL_X_L = 0x3C

    ACCEL_Y_H = 0x3D
    ACCEL_Y_L = 0x3E

    ACCEL_Z_H = 0x3F
    ACCEL_Z_L = 0x40

    # Addresses for the Gyroscope value composition which are split into a high and a low bit
    GYRO_X_H = 0x43
    GYRO_X_L = 0x44

    GYRO_Y_H = 0x45
    GYRO_Y_L = 0x46

    GYRO_Z_H = 0x47
    GYRO_Z_L = 0x48


    def __init__(self, i2c=None, scl=Pin(1), sda=Pin(0)):
        self.i2c = i2c if i2c else I2C(0, scl=scl, sda=sda)
        self.i2c.writeto_mem(self.MPU6050_ADDR, self.MPU6050_PWR_MGMT_1, bytes([0]))
        self.i2c.writeto_mem(self.MPU6050_ADDR, self.MPU6050_ACCEL_RANGE_CONFIG, bytes([0 << 3]))
        self.i2c.writeto_mem(self.MPU6050_ADDR, self.MPU6050_GYRO_RANGE_CONFIG, bytes([0 << 3]))

    @classmethod
    def __resolve_values(cls, highByte, lowByte):
        if not highByte[0] & 0x80:
            return highByte[0] << 8 | lowByte[0]
        return -((highByte[0] ^ 255) << 8) | (lowByte[0] ^ 255) + 1

    def get_accelerations(self):
        accel_x_h = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.ACCEL_X_H, 1)
        accel_x_l = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.ACCEL_X_L, 1)
        accel_y_h = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.ACCEL_Y_H, 1)
        accel_y_l = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.ACCEL_Y_L, 1)
        accel_z_h = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.ACCEL_Z_H, 1)
        accel_z_l = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.ACCEL_Z_L, 1)
        return (self.__resolve_values(accel_x_h, accel_x_l) / self.MPU6050_ACC_LSBG,
                self.__resolve_values(accel_y_h, accel_y_l) / self.MPU6050_ACC_LSBG,
                self.__resolve_values(accel_z_h, accel_z_l) / self.MPU6050_ACC_LSBG)
    
    def get_acceleration_x(self):
        accel_x_h = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.ACCEL_X_H, 1)
        accel_x_l = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.ACCEL_X_L, 1)
        return self.__resolve_values(accel_x_h, accel_x_l) / self.MPU6050_ACC_LSBG
    
    def get_acceleration_y(self):
        accel_y_h = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.ACCEL_Y_H, 1)
        accel_y_l = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.ACCEL_Y_L, 1)
        return self.__resolve_values(accel_y_h, accel_y_l) / self.MPU6050_ACC_LSBG
    
    def get_acceleration_z(self):
        accel_z_h = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.ACCEL_Z_H, 1)
        accel_z_l = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.ACCEL_Z_L, 1)
        return self.__resolve_values(accel_z_h, accel_z_l) / self.MPU6050_ACC_LSBG

    def get_angular_rates(self):
        gyro_x_h = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.GYRO_X_H, 1)
        gyro_x_l = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.GYRO_X_L, 1)
        gyro_y_h = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.GYRO_Y_H, 1)
        gyro_y_l = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.GYRO_Y_L, 1)
        gyro_z_h = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.GYRO_Z_H, 1)
        gyro_z_l = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.GYRO_Z_L, 1)
        return (self.__resolve_values(gyro_x_h, gyro_x_l) / self.MPU6050_GYRO_LSBDPS,
                self.__resolve_values(gyro_y_h, gyro_y_l) / self.MPU6050_GYRO_LSBDPS,
                self.__resolve_values(gyro_z_h, gyro_z_l) / self.MPU6050_GYRO_LSBDPS)
    # Roll rate
    def get_roll_rate(self):
        gyro_x_h = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.GYRO_X_H, 1)
        gyro_x_l = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.GYRO_X_L, 1)
        return self.__resolve_values(gyro_x_h, gyro_x_l) / self.MPU6050_GYRO_LSBDPS
    
    # Pitch rate
    def get_pitch_rate(self):
        gyro_y_h = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.GYRO_Y_H, 1)
        gyro_y_l = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.GYRO_Y_L, 1)
        return self.__resolve_values(gyro_y_h, gyro_y_l) / self.MPU6050_GYRO_LSBDPS
    
    # Yaw rate
    def get_yaw_rate(self):
        gyro_z_h = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.GYRO_Z_H, 1)
        gyro_z_l = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.GYRO_Z_L, 1)
        return self.__resolve_values(gyro_z_h, gyro_z_l) / self.MPU6050_GYRO_LSBDPS
    
    def get_roll_angle(self):
        accel_x, accel_y, accel_z = self.get_accelerations()
        angle_roll_radians = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2))
        angle_roll_degrees = angle_roll_radians * (180 / math.pi)
        return angle_roll_degrees
    
    def get_pitch_angle(self):
        accel_x, accel_y, accel_z = self.get_accelerations()
        angle_pitch_radians = -math.atan2(accel_x, math.sqrt(accel_y**2 + accel_z**2))
        angle_pitch_degrees = angle_pitch_radians * (180 / math.pi)
        return angle_pitch_degrees

