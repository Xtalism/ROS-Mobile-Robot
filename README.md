# ROS-Mobile-Robot

Mobile Robot using ROS and ESP32 (rosserial).

## Table of Contents
- [ROS-Mobile-Robot](#ros-mobile-robot)
  - [Table of Contents](#table-of-contents)
  - [Hardware Requirements](#hardware-requirements)
  - [Software Requirements](#software-requirements)
  - [Installation](#installation)
  - [Usage](#usage)
  - [License](#license)

## Hardware Requirements
- ESP32.
- Motors.
- L298N.
- 18650 batteries (adds up to 14v).

## Software Requirements
In order for the project to work we need to have installed [Ubuntu 20.04 LTS Focal Fossa](https://releases.ubuntu.com/focal/) alocated
in a partition with a Windows dual boot or as the main Operating System of your machine. 

![Ubuntu Focal Fossa](https://ubuntucommunity.s3.us-east-2.amazonaws.com/original/2X/c/cc5769b9fb7c4c088876fbe477452a44e1c36a4d.jpeg)

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

## Usage
Explain how to use the project, including any important commands or configurations.

## License
Include the license information for the project.