###################################################
#        Lean angle meter for motorcycles         #
#                                                 #
# This project uses the MPU6050 IMU to measure    #
# the acceleration and angular rates, then passes #
# the values to a Kalman filter to accurately     #
# determine the roll angle of the motorcycle.     #
# Result is displayed live onto a 128x32px OLED.  #
#                                                 #
###################################################
#                                                 #
# Written by @SantCineva                          #
#                                                 #
###################################################



# Used libs
from machine import I2C, Pin
import KalmanFilter as kf
import ssd1306_gfx
import mpu6050
import writer
import time
import math

# Used fonts
import AbelRegular26
import UAVSansMono32


if __name__ == "__main__":

    # Initialize I2C connection, MPU6050 IMU, OLED Display 128x32px
    i2c = I2C(0, scl=Pin(1), sda=Pin(0))
    imu = mpu6050.MPU6050(i2c)
    display = ssd1306_gfx.SSD1306_I2C_SETUP(1, 0, 128, 32)

    # Image and font
    mainFont = writer.Writer(display, UAVSansMono32)
    secFont = writer.Writer(display, AbelRegular26)

    # Initialize Kalman Filter
    kalman_filter = kf.KalmanFilter(Q_angle=0.001, Q_gyro=0.003, R_angle=0.03)

    # Timing variables
    previous_time = time.ticks_ms()

    # Clear display
    display.fill(0)
    display.show()

    # Max lean angles
    max_right_angle = 0
    max_left_angle = 0

    roll_zero_start_time = None

    reset_duration = 5000  # Duration in milliseconds
    
    inputFolder = '/introGif'
    display.play_animation(inputFolder, 0.05)

    # Main loop
    while True:
        current_time = time.ticks_ms()
        dt = (current_time - previous_time) / 1000.0  # Convert ms to seconds
        previous_time = current_time

        # Get accelerometer roll angle
        roll_acc = imu.get_roll_angle()
        # Get gyroscope roll rate
        roll_rate = imu.get_roll_rate()

        # Calculate roll angle using Kalman filter
        roll = kalman_filter.get_angle(roll_acc, roll_rate, dt)

        # Update max angles and timing
        max_right_angle = max(max_right_angle, roll)
        max_left_angle = min(max_left_angle, roll)  

        # Check if roll angle is 0
        if int(roll) == 0:
            if roll_zero_start_time is None:
                roll_zero_start_time = current_time
            elif time.ticks_diff(current_time, roll_zero_start_time) > reset_duration:
                max_right_angle = 0
                max_left_angle = 0
                roll_zero_start_time = None
        else:
            roll_zero_start_time = None

        # Display clear
        display.fill(0)
        
        secFont.set_textpos(0, 0)
        secFont.printstring(f"{abs(int(max_right_angle))}")
        
        # Branchless way to set text position based on the value of roll
        offset = (abs(int(roll)) >= 10) * 14  # If roll >= 10, offset is 10; otherwise, it's 0
        text_pos = 52 - offset
        mainFont.set_textpos(text_pos, 0)   
        mainFont.printstring(f"{abs(int(roll))}")
        
        secFont.set_textpos(94, 0)
        secFont.printstring(f"{abs(int(max_left_angle))}")
        
        display.show()

        time.sleep(0.01)

