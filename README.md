# Description


This node provides Ackermann steering control for a vehicle. The vehicle can be driven using the W-A-S-D keys on a keyboard.


# Usage


To move the vehicle, use the following keys:

|      |      |      |
|:----:|:----:|:----:|
|      |   ^  |      |
|      |   w  |      |
| <  a |   s  |  d > |
|      |   v  |      |


w : increase speed


s : decrease speed


a : turn left


d : turn right


r : set steering angle to 0


q : stop and quit


# Control


The node sends Ackermann messages to the /cmd_ackermann topic to control the vehicle's steering and speed.

1. Copy the folder to your workspace directory, for example `ros2_ws/src`.
2. Build your workspace using `colcon build`.
3. Source your workspace by running `source install/setup.bash`.
4. Run the ackermann_control node using the command `ros2 run ackermann_control ackermann_control`.
