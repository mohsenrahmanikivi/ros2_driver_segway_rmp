# Segway-ROS2-driver-libsegwayrmp-based
ROS2 Segway Drivers for BWIBots V2. It's derived from <code>libsegwayrmp</code> (https://github.com/utexas-bwi/libsegwayrmp.git)

## Dependencies
### serial
- put the serial folder from the following repo in parallel to your colcon workspace
- <code>git clone https://github.com/utexas-bwi/serial_for_ros2.git</code>

### libsegwayrmp-ros2
- <code>cd [ROS_WS]/src</code>
- <code>git clone https://github.com/utexas-bwi/libsegwayrmp_ros2.git</code>
- use <code>libsegwayrmp</code> as a main package of ros2 humble and not <code>libsegwayrmp_ros2</code>

## Launch
- <code>ros2 launch segway_rmp_ros2 segway_rmp_ros2.launch.py</code>
  
## Test
- <code>ros2 run teleop_twist_keyboard teleop_twist_keyboard</code>
- You should be able to operate the segway with teleop commands now
