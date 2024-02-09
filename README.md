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
6. Optional: Static IP address
By default, the configuration of the Ubuntu 22.04. server edition is set to DHCP. If you would like to set a static IP address, you can do this by making the following adjustment:
```console
sudo echo "network: {config: disabled}" > /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg
```
This switches off the automatism with which the network configuration file is generated. The network configuration file /etc/netplan/50-cloud-init.yaml must now be adapted:
```console
network:
    renderer: networkd
    ethernets:
        eth0:
          addresses:
            - 192.168.178.111/24
          nameservers:
            addresses: [4.2.2.2, 8.8.8.8]
          routes:
            - to: default
              via: 192.168.178.1
    version: 2
```
Replace the IP addresses above with your desired configuration. Then reboot your system.

7. Get and build the edu_drive_ros2 software
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

## Free Kinematics Kit
The Free Kinematics Kit from EduArt gives you all the freedom you need to build your own robot with few restrictions on the mechanical design.

<p align="center">
  <img src="/images/Free_Kinematics_Kit_1920.jpg" width="800"/>
</p>

It consists of the following components, which can all be plugged together:
- *Adapter Board* either directly pluggable onto a Raspberry Pi 4/5 or as a standalone EduArt Ethernet controller board, with which you can connect any computer with an Ethernet interface. This gives you the freedom to build the computing power into your robot that you need for your desired application.
- *Motorcontrollers* can be plugged in directly. This allows you to control the speed of 2 to 8 motors.
- The *Power Management Module* takes over the charge control of a 19.2V NiMH battery pack and also offers an on/off logic.
- The *Auxiliary Power Supply Module* provides additional voltage levels with which you can supply additional devices.

### Electrical Interface
Below you can see the electrical interfaces of the Free Kinematics Kit. There are off-the-shelf cables for the white Molex socket. Depending on the desired cable length, you can obtain the following part numbers from the usual distributors: 151360800 (50 mm), 151360801 (100 mm), 151360802 (150 mm), 151360803 (300 mm), 151360805 (450 mm) or 151360806 (600 mm).

<p align="center">
  <img src="/images/Free_Kinematics_Kit_Electrical_Interface_Desc_1920.jpg" width="800"/>
</p>

Integrated voltage monitoring of the power management module protects your robot in the event of incorrect operation:
* If the voltage is too low (< 17.5 V), the drives are deactivated after 10 seconds.
* If the voltage remains below 17.5 V for longer than 120 seconds, the system switches off automatically. This protects your battery from deep discharge.
* If the voltage is above 24 V, the drives are deactivated immediately. This prevents you from driving off with a connected power supply unit.

The following diagram shows an example of the logic functions.

<p align="center">
  <img src="/images/simulateEnableLogic/simulateEnableLogic.png" width="800"/>
</p>

> **Warning:_**: Despite these protective functions, you must always ensure that the device is used as intended. Never charge the device unattended. Also make sure that no persons are in the immediate vicinity when operating your robot and that the robot cannot fall from a height difference.
