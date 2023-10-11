import numpy as np
from geometry_msgs.msg import Twist

class Robot:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, velocity=0.1):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.velocity = velocity
        self.path_x = []
        self.path_y = []

    def move_to_point(self, target_x, target_y):
        a = target_x - self.x
        b = target_y - self.y
        target_yaw = np.arctan2(b, a)
        delta_yaw = target_yaw - self.yaw
        distance = np.sqrt(a**2 + b**2)

        if abs(delta_yaw) > np.pi:
            delta_yaw = -(2 * np.pi - delta_yaw)

        steering = delta_yaw
        return distance, steering

    def move(self, steering, distance):
        self.yaw += steering
        self.x += self.velocity * np.cos(self.yaw)
        self.y += self.velocity * np.sin(self.yaw)
        return self.x, self.y, self.yaw

    def pure_pursuit(self, cx, cy, lookahead):

        lookahead_point = (cx, cy)

        if lookahead_point is None:
            return None, None

        distance, steering = self.move_to_point(lookahead_point[0], lookahead_point[1])
        return distance, steering


        return cmd
class PID:
    def __init__(self, P=1.0, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.previous_error = 0.0
        self.integral = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output



if __name__ == "__main__":
    cx = np.arange(0, 50, 1)
    cy = [np.sin(ix / 5.0) * ix / 2.0 for ix in cx]

    robot = Robot(x=0.0, y=-3.0, yaw=0.0)

    lookahead = 5.0
    while True:
        distance, steering = pure_pursuit(robot, cx, cy, lookahead)
        if distance is None:
            break
        robot.move(steering, distance)
        robot.path_x.append(robot.x)
        robot.path_y.append(robot.y)

    # 결과를 표시하려면 matplotlib을 사용하십시오.
    # plt.plot(cx, cy, "-r", label="Course")
    # plt.plot(robot.path_x, robot.path_y, "-b", label="Path")
    # plt.legend()
    # plt.show()


