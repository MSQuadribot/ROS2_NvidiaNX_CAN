# ROS2 CAN Package for the Nvidia NX Xavier

This package's goal is to use a CAN device provided with the nvidia NX Xavier in a ROS2 environment.
More precisely, the goal is to create a link betweenthe Nvidia NX Xavier and an electric golf car using a CAN device.

## How to build the package :

First, open a new terminal and go to the following directory : /Desktop/dev_ws
There you will be able to build all the packages that are in the src subdirectory

$ colcon build

This process can take some times (on my machine it could take up to 15s to run)
Remember that if the built was already completed, there is no need to run it again
But, be also aware that you have to rebuild the package each time you make a modification

## How to run the package :

For that, you will need to open a new terminal and go to the following directory : /Desktop/dev_ws
If you just built the package, the terminal will need to be different as this can cause issues otherwise

in order to launch a process for this package you will need to source your workspace

$ . install/setup.bash

Once done, you can process with using this wonderful package

$ ros2 run canbot {$processname}

## What are the available process ?

There are currently five available process :

reader will provide the information directly from the CAN, according that the emitted data are conform.
The data are retrieved directly from the CAN device. And then are sent in the form of a ROS2 message.

subscriber is set to get the data from the publisher.

keyboard is set to get the data from the keyboard. Thus by using 'WASD' keys, you can control the car.
Speed, steering, and braking are controlled by the keyboard and are adjusted dynamically by the user.

controller use this keyboard input and send it to the car using the CAN Bus. 
The bus is updated every 20ms even if the speed and other element remain unchanged.
Indeed, the car natively supports an "alive" counter that is incremented every 20ms.
If the counter is not incremented, the car will stop.

variables will use the can data provided by the reader and will send it to get the car variables.
This include speed, steering angle, battery status, and more.
This node was designed with the precise requirement of the CAN bus provided by a specific car.

## What are the topics provided by the package ?

The main topics is can_data, it is provided by the Node can_reader.
This topic is used to send the data from the CAN device to the other nodes.
The message is a BusCan custom message. It reproduce the CAN frame structure in a user-friendly way.

Another topic is provided by the Node can_input. 
This topic is used to send the data from the keyboard to the other nodes.
This idea was to use this topic to remotely control the car.


## Important note

In most Ubuntu devices, normal users will not be able to natively access devices.
Furthermore, the CAN device is not activated by default.

You will need to activate it in order to be able to use it. This requires root privileges.

$ sudo ip lint set can0 type can bitrate 500000

$ sudo ip link set up can0

Then the user can use the terminal to access the CAN device.

$ cansend can0 300#0FF0FF

$ candump can0

What is more, the package needs to use an home made ROS2 message type to send the data.
It is located in another package : interfaces and it is names BusCan.msg

Sometimes, the use of a virtual device can be more valuable.
For this, the user need to create a new virtual device and then use it.

$ sudo modprobe vcan

$ sudo ip link add dev vcan0 type vcan

$ sudo ip link set up vcan0

Then the user can use the terminal to access the virtual CAN device.
In case the user is willing to use the virtual device with this package, he will need to change the device name in the code.
