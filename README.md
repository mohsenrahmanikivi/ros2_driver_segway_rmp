# ros2_driver_segway_rmp based libsegwayrmp, the official library
This package is derived from <code>segway_rmp</code> (https://github.com/utexas-bwi/segway_rmp.git)
- If not define then run <code>export ROS_WS= your full path of the ros working space</code>


## Dependencies
### 1. serial
- Keep parallel in your workspace's src folder
- <code>cd ${ROS_WS}/src</code>
- <code>git clone https://github.com/mohsenrahmanikivi/serial_for_ros2.git</code>
- NEED BUILDING BEFORE PROCEED *Enter the folder, then follow the instructions to build the serial library*.

  
### 2. libsegwayrmp-ros2
- Keep this repository parallel in your workspace's src folder
- <code>cd ${ROS_WS}/src</code>
- <code>git clone https://github.com/mohsenrahmanikivi/libsegwayrmp_ros2.git</code>
- use <code>libsegwayrmp</code> as a main package of ros2 humble and not <code>libsegwayrmp_ros2</code>

### 3. segway_rmp_ros2
- Keep this repository parallel in your workspace's src folder
- <code>cd ${ROS_WS}/src</code>
- <code>git clone https://github.com/mohsenrahmanikivi/segway_rmp_ros2.git </code>


## Final build
- <code>cd ${ROS_WS} && colcon build</code>
- in case of " internal compiler error: Segmentation fault 477 | enable current_exception( T const & x )"
<code> How I resolved my g++ segmentation fault issue
sudo apt-get install build-essential software-properties-common -y
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y 
sudo apt-get update
sudo apt-get install gcc-snapshot -y
  </code>
  reference https://stackoverflow.com/questions/44286265/g-internal-compiler-error-segmentation-fault-program-cc1plus-where-do-i

## Launch
- <code>source ${ROS_WS}/install/setup.bash</code>
- <code>ros2 launch segway_rmp_ros2 segway_rmp_ros2.launch.py</code>
  
## Test
- <code>ros2 run teleop_twist_keyboard teleop_twist_keyboard</code>
- You should be able to operate the segway with teleop commands now
