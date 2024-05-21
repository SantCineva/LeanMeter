###########################################################
#  Simple class to facilitate the use of a Kalman filter  #
#  in the context of a IMU like the MPU6050               #
#                                                         #
#  Written by @SandCineva                                 #
###########################################################

class KalmanFilter:
    def __init__(self, Q_angle, Q_gyro, R_angle):
        self.Q_angle = Q_angle  # Process noise variance for the accelerometer
        self.Q_gyro = Q_gyro    # Process noise variance for the gyroscope
        self.R_angle = R_angle  # Measurement noise variance

        self.angle = 0.0  # Reset the angle
        self.bias = 0.0   # Reset bias
        self.rate = 0.0   # Unbiased rate

        self.P = [[0.0, 0.0], [0.0, 0.0]]  # Error covariance matrix

    def get_angle(self, new_angle, new_rate, dt):
        # Prediction
        self.rate = new_rate - self.bias
        self.angle += dt * self.rate

        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_gyro * dt

        # Update
        S = self.P[0][0] + self.R_angle             # Estimate error
        K = [self.P[0][0] / S, self.P[1][0] / S]    # Kalman gain

        y = new_angle - self.angle                  # Angle difference
        self.angle += K[0] * y
        self.bias += K[1] * y

        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return self.angle

