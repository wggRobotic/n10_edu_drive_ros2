# edu_drive_ros2
This package comprises a ROS2 interface for EduArt's generic drive concept. It covers several kinematic concepts: Differential drive, Mecanum steering and Skid steering. All three can be used in dependency of the mounted wheels and the configuration of YAML parameters.

<p align="center">
  <img src="/images/edu_drive.png"/>
</p>

## Launching the Robot
In order to run the robot, you need to launch the appropriate launch file. In the launch folder, there is a prepared template.
Configure the correct kinematic concept and motor parameters. A description of the YAML file can be found below.
```console
ros2 launch edu_drive_ros2 edu_drive.launch.py
```
When everthing is initialized well, one should see the following output:
```console
# Listener start
CAN Interface: CAN2
[ INFO] [1651663592.994224328]: Instanciated robot with vMax: 0.366519 m/s and omegaMax: 0.733038 rad/s
...
```
After startup, the drive system is in the deactivated state.

Please notice also, that the ROS variable ROS_DOMAIN_ID should be set properly.

## YAML file parameters

| tag    | description      |
| ------ |:--|
| usingPowerManagementBoard | Define if a Power Management Board is used, default is "true"
| verbosity      | Display status messages for debugging, default is "false"
| canInterface   | SocketCAN interface, e.g. CAN2       |
| frequencyScale | Divider of PWM frequency, Base frequency: 500kHz (Divider=1)     |
| inputWeight    | Low pass filtering, setpoint = inputWeight*setpoint_new + (1-inputWeight)*setpoint_old  |
| maxPulseWidth  | Limitation of pulse width, meaningful values [0; 100]      |
| timeout        | Dead time monitoring of communication (in milliseconds) |
| kp, ki, kd     | Controller coefficients    |
| antiWindup     | Enable anti windup monitoring of closed-loop controller     |
| invertEnc      | Invert encoder signal |
| responseMode   | Activate transmission of RPM or position measurements of motor controller |
| controllers    | Number of motor controllers used |
| canID          | Motorcontroller ID (set by DIP switch on motor controller PCB) |
| gearRatio      | Gear ratio of connected geared motors |
| encoderRatio   | Encoder pulses per motor revolution (rising and falling edge evaluation) |
| rpmMax         | Maximum revolutions per minute of geared motor pinion |
| channel        | Used channel of motor controller. There are single-channel motorshields and dual-channel motorshields. Meaningful values are 0 or 1. |
| kinematics     | Three-dimensional vector describing the conversion from Twist messages in motor revolutions. See explanation below. |

## Calculation of the kinematic parameters
The kinematic concept uses a conversion matrix for the conversion of twist parameters and wheel speeds.

<img src="https://latex.codecogs.com/svg.image?\begin{pmatrix}&space;\omega_0&space;\\&space;\omega_1&space;\\&space;\omega_2&space;\\&space;\omega_3\end{pmatrix}&space;=&space;\mathbf{T}\begin{pmatrix}v_x&space;\\v_y&space;\\\omega\end{pmatrix}&space;" title="https://latex.codecogs.com/svg.image?\begin{pmatrix} \omega_0 \\ \omega_1 \\ \omega_2 \\ \omega_3\end{pmatrix} = \mathbf{T}\begin{pmatrix}v_x \\v_y \\\omega\end{pmatrix} " />

, where &omega;<sub>i</sub> are the wheel's angular velocities and v<sub>x</sub>, v<sub>y</sub> and &omega; are Twist values. The matrix **T** can be calculated as follows:
<img src="https://latex.codecogs.com/svg.image?\mathbf{T}&space;=&space;\begin{pmatrix}&space;kx_0&space;&&space;ky_0&space;&&space;k\omega{}_0\\&space;kx_1&space;&&space;ky_1&space;&&space;k\omega{}_1\\&space;kx_2&space;&&space;ky_2&space;&&space;k\omega{}_2\\&space;kx_3&space;&&space;ky_3&space;&&space;k\omega{}_3\end{pmatrix}" title="https://latex.codecogs.com/svg.image?\mathbf{T} = \begin{pmatrix} kx_0 & ky_0 & k\omega{}_0\\ kx_1 & ky_1 & k\omega{}_1\\ kx_2 & ky_2 & k\omega{}_2\\ kx_3 & ky_3 & k\omega{}_3\end{pmatrix}" />

for a four-wheeled robot. kx<sub>i</sub>, ky<sub>i</sub> and k&omega;<sub>i</sub> are the translation parameters from one space to the other. These parameters include the wheel radius r as well as the robot length l<sub>x</sub> and robot width l<sub>y</sub>.

Depending on the polarity of the motor wiring, the kinematic parameters may have to negated.

### Example for a Differential drive
<img src="https://latex.codecogs.com/svg.image?\mathbf{T}&space;=&space;\begin{pmatrix}&space;\frac{1}{r}&space;&&space;0&space;&&space;-\frac{l_y}{2&space;\cdot&space;r}\\&space;-\frac{1}{r}&space;&&space;0&space;&&space;-\frac{l_y}{2&space;\cdot&space;r}\\\end{pmatrix}" title="https://latex.codecogs.com/svg.image?\mathbf{T} = \begin{pmatrix} \frac{1}{r} & 0 & -\frac{l_y}{2 \cdot r}\\ -\frac{1}{r} & 0 & -\frac{l_y}{2 \cdot r}\\\end{pmatrix}" />

### Example for a Mecanum drive
<img src="https://latex.codecogs.com/svg.image?\mathbf{T}&space;=&space;\frac{1}{r}\begin{pmatrix}&space;&space;1&space;&&space;-1&space;&&space;-\frac{l_x&plus;l_y}{2}\\&space;-1&space;&&space;-1&space;&&space;-\frac{l_x&plus;l_y}{2}\\&space;&space;1&space;&&space;&space;1&space;&&space;-\frac{l_x&plus;l_y}{2}\\&space;-1&space;&&space;&space;1&space;&&space;-\frac{l_x&plus;l_y}{2}\end{pmatrix}" title="https://latex.codecogs.com/svg.image?\mathbf{T} = \frac{1}{r}\begin{pmatrix} 1 & -1 & -\frac{l_x+l_y}{2}\\ -1 & -1 & -\frac{l_x+l_y}{2}\\ 1 & 1 & -\frac{l_x+l_y}{2}\\ -1 & 1 & -\frac{l_x+l_y}{2}\end{pmatrix}" />

# Setting up a Raspberry PI4 from scratch
1. Install Raspberry Pi OS or Ubuntu, Ubuntu 22.04.3 server (jammy jellyfish) has been tested
2. Update and install packages
```console
sudo apt update
sudo apt upgrade
sudo apt install can-utils build-essential git
```
3. Add udev rules for CAN interfaces
The extension board for the Raspberry Pi provides three CANFD interfaces. To ensure that the naming of the interfaces is the same after each boot process, a udev rule must be created in the /etc/udev/rules.d directory. Create the file /etc/udev/rules.d/42-mcp251xfd.rules with the following content:
```console
KERNELS=="spi0.0", SUBSYSTEMS=="spi", DRIVERS=="mcp251xfd", ACTION=="add", NAME="CAN0"
KERNELS=="spi0.1", SUBSYSTEMS=="spi", DRIVERS=="mcp251xfd", ACTION=="add", NAME="CAN1"
KERNELS=="spi1.0", SUBSYSTEMS=="spi", DRIVERS=="mcp251xfd", ACTION=="add", NAME="CAN2"
```
In this way, the CAN interface for the motor controller is always named CAN2. The CAN0 and CAN1 interfaces can be accessed via the sockets on the expansion board (see labeling on the board) and are intended for connecting the flexible sensor ring from EduArt.

4. Add the following entries in /etc/rc.local. On newer Ubuntu systems /etc/rc.local may not be loaded by default. See [this description](https://www.linuxbabe.com/linux-server/how-to-enable-etcrc-local-with-systemd) on how to enable it.
```console
ip link set CAN2 up type can bitrate 500000
```
5. Install ROS. Read the [official documentation](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html) for more detail
```console
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt install ros-humble-ros-base
sudo apt install python3-colcon-common-extensions
sudo apt install ros-dev-tools
sudo reboot
```
6. Get and build the edu_drive_ros2 software
```console
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/EduArt-Robotik/edu_drive_ros2.git
cd ..
colcon build --packages-select edu_drive_ros2 --symlink-install
source install/setup.bash
```

Now, the launch file should be started. Please adjust the parameters in edu_drive.yaml before.
```console
ros2 launch edu_drive_ros2 edu_drive.launch.py
```
