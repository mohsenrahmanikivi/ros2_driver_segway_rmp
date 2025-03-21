# Segway-ROS2-driver-libsegwayrmp-based
ROS2 Segway Drivers for BWIBots V2. This package is derived from <code>segway_rmp</code> (https://github.com/utexas-bwi/segway_rmp.git)

## Dependencies
### serial
- put the serial folder from the following repo in parallel to your colcon workspace
- <code>git clone https://github.com/mohsenrahmanikivi/serial_for_ros2.git</code>

### libsegwayrmp-ros2
- <code>cd [ROS_WS]/src</code>
- <code>git clone https://github.com/mohsenrahmanikivi/libsegwayrmp_ros2.git</code>
- use <code>libsegwayrmp</code> as a main package of ros2 humble and not <code>libsegwayrmp_ros2</code>

### build
- <code>[ROS_WS]/colcon build</code>
- in case of " internal compiler error: Segmentation fault 477 | enable current_exception( T const & x )"
<code> How I resolved my g++ segmentation fault issue
sudo apt-get install build-essential software-properties-common -y
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y 
sudo apt-get update
sudo apt-get install gcc-snapshot -y
  </code>
  reference https://stackoverflow.com/questions/44286265/g-internal-compiler-error-segmentation-fault-program-cc1plus-where-do-i

## Launch
- <code>ros2 launch segway_rmp_ros2 segway_rmp_ros2.launch.py</code>
  
## Test
- <code>ros2 run teleop_twist_keyboard teleop_twist_keyboard</code>
- You should be able to operate the segway with teleop commands now
