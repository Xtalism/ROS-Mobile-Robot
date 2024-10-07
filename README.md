# ROS-Mobile-Robot

Mobile Robot using ROS and ESP32 (rosserial).

## Table of Contents
- [ROS-Mobile-Robot](#ros-mobile-robot)
  - [Table of Contents](#table-of-contents)
  - [Hardware Requirements](#hardware-requirements)
  - [Software Requirements](#software-requirements)
  - [Installation](#installation)
  - [Controller Setup](#controller-setup)
  - [Initialize services.](#initialize-services)
  - [License](#license)

## Hardware Requirements
- ESP32.
- Motors.
- L298N.
- Xbox Controller.
- 18650 batteries.

## Software Requirements
In order for the project to work we need to have installed [Ubuntu 20.04 LTS Focal Fossa](https://releases.ubuntu.com/focal/) alocated
in a partition with a Windows dual boot or as the main Operating System of your machine. 

![Ubuntu Focal Fossa](ubuntu.png)

You can try to virtualize it but in my experience, networking and driver problems arouse.

## Installation
We need to install [ROS Noetic Ninjemys](https://wiki.ros.org/noetic), you can follow the [documentation](https://wiki.ros.org/noetic/Installation/Ubuntu) on how to install it or you can follow and run these few commands inside Ubuntu's terminal:

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

```shell
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

```shell
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Controller Setup
You can use any controller listed in the [documentation](http://wiki.ros.org/joy). I'm personally
using an 8bitdo Ultimate C 2.4GHz. It is not a Xbox Controller per se but I managed to find some drivers to 
trick Linux into thinking it is a generic Xbox Controller ([you can find the post here](https://gist.github.com/ammuench/0dcf14faf4e3b000020992612a2711e2)):

```shell
touch /etc/udev/rules.d/99-8bitdo-xinput.rules
sudo nano /etc/udev/rules.d/99-8bitdo-xinput.rules
```
Inside 99-8bitdo-xinput.rules you type the following and save:
```shell
ACTION=="add", ATTRS{idVendor}=="2dc8", ATTRS{idProduct}=="3106", RUN+="/sbin/modprobe xpad", RUN+="/bin/sh -c 'echo 2dc8 3106 > /sys/bus/usb/drivers/xpad/new_id'"
```

Reload udevadm service
```shell
sudo udevadm control --reload
```

You also need to make your [joystick device accesible](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick):

```shell
ls -l /dev/input/jsX
```

You will see something similar to:
```shell
crw-rw-XX- 1 root dialout 188, 0 2009-08-14 12:04 /dev/input/jsX
```

We need to change XX to rw:
```shell
sudo chmod a+rw /dev/input/jsX
```

## Initialize services.

We initialize our master:
```shell
roscore
```

To initialize our serial node using tcp protocol:
```shell
rosrun rosserial_python serial_node.py tcp
```

To initialize our joy node using joy package:
```shell
rosrun joy joy_node
```

If we want to see our joy node response:
```shell
rostopic echo joy
```

## License
[MIT License](https://opensource.org/osd)