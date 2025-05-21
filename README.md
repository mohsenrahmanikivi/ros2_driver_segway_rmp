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

### 1. Verify the serial communication channel
#### PINs Connection
Users need to connect the serial communication cable to actively achieve hardware communication. Specifically,
1. Connect the TX (PIN3) of the 8PIN port and the RX of the serial port module.
2. Connect the RX (PIN4) of the 8PIN port and the TX of the serial port module.
3. Connect the GND (PIN5) of the 8PIN port and the GND of the serial port module.

#### 8PIN pinout
1. CAN, CANH AWG26, Red
2. CAN, CANL AWG26, Gray
3. Serial Port, TX AWG26, Blue
4. Serial Port, RX AWG26, Green
5. Serial Port, GND AWG26, White
6. RC-R, 5V AWG26 Remote, Brown
7. RC-R, GND AWG26, Black
8. RC-R, S.B PPM AWG26, Yellow

ref: https://github.com/SegwayRoboticsSamples/RMP220-SDK/wiki/HardWare_Connection

#### Verify
1. Check whether it is recognised or not
- <code> sudo dmesg | grep tty </code> The response should be something like <code> usb 1-2.4: cp210x converter now attached to ttyUSB0 </code>
- <code>  lsusb </code>  The response should be something like <code> Bus 001 Device 006: ID 10c4:ea60 Silicon Labs CP210x UART Bridge </code>
2. Set the baud rate (921600) using the stty command (Replace /dev/ttyUSB0 with your actual device path.)
- <code> sudo stty -F /dev/ttyUSB0 921600 </code>
3. Run the cat command to read from the serial port:
-  <code> sudo cat /dev/ttyUSB0 </code> This will start showing the data being received from the connected hardware. If the serial communication is working, you'll see continuous data output.


### 2. Run

- if using isaac docker run the ros2 container with
    -  <code> cd $ISAAC_ROS_WS && ./src/isaac_ros_common/scripts/run_dev.sh</code>
    -  <code> export ROS_WS=$ISAAC_ROS_WS </code>
- <code>source ${ROS_WS}/install/setup.bash</code>
- <code>ros2 launch segway_rmp_ros2 segway_rmp_ros2.launch.py</code>
  
## Test
- if not installed, <code> sudo apt update && sudo apt install ros-${ROS_DISTRO}-teleop-twist-keyboard </code>
- <code>ros2 run teleop_twist_keyboard teleop_twist_keyboard</code>
- You should be able to operate the segway with teleop commands now
