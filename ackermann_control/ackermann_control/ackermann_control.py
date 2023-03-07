import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
import math
import sys
import os

if sys.platform == 'win32':
    import msvcrt
else:
    import tty
    import termios

    
def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

message = """
Ackermann Control Node
---------------------------
Moving around:
          ^
          w 
   < a    s    d >
---------------------------
w : increase speed
s : decrease speed
a : turn left
d : turn right
r : steering angle to 0
q : to stop and quit
"""

class ackermannControl(Node):
    def __init__(self):
        super().__init__('Ackermann_Control')
        self.publisher_ = self.create_publisher(AckermannDrive, '/cmd_ackermann', 10) 
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.speed = 0.0
        self.steering_angle = 0.0
        self.lines = []
        print(message)


    def timer_callback(self):
        msg = AckermannDrive()
        msg.steering_angle_velocity = math.pi/72
        msg.acceleration = 0.5
        key = getKey(settings=termios.tcgetattr(sys.stdin.fileno()))
        
        if key.lower() == 'w':
            self.speed = self.speed + 1.0
        elif key.lower() == 'a':
            self.steering_angle = self.steering_angle - math.pi/36
        elif key.lower() == 's':
            self.speed = self.speed - 1.0
        elif key.lower() == 'd':
            self.steering_angle = self.steering_angle + math.pi/36
        elif key.lower() == 'r':
            self.steering_angle = 0.0
        elif key.lower() == 'q':
            self.speed = 0.0
            self.steering_angle = 0.0
            msg.steering_angle = self.steering_angle
            msg.speed = self.speed
            self.publisher_.publish(msg)
            messages = "speed: " + str(self.speed) + " steering_angle: " + str(self.steering_angle)
            print(messages)
            sys.exit(0)
        messages = "speed: " + str(self.speed) + " steering_angle: " + str(self.steering_angle)
        print(messages)
        self.lines.append(messages)
        msg.steering_angle = self.steering_angle
        msg.speed = self.speed
        self.publisher_.publish(msg)
        
        if len(self.lines) > 8:
            self.lines.clear()
            clear_screen()
            print(message)
            print(messages)


def main(args=None):
    rclpy.init(args=args)
    ackermann_control = ackermannControl()
    rclpy.spin(ackermann_control)
    ackermann_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
