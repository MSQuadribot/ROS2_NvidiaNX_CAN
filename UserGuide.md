# How to use this ROS2 package in order to drive the golf car

## Start by sourcing the package

First, open a new terminal and go to the following directory : /Desktop/dev_ws
There you will be able to build all the packages that are in the src subdirectory.

$ colcon build

This process can take some times (on my machine it could take up to 15s to run).
Remember that if the built was already completed, there is no need to run it again.
But, be also aware that you have to rebuild the package each time you make a modification.

in order to launch a process for this package you will need to source your workspace.

$ . install/setup.bash

Once done, you can process with using this wonderful package.

$ ros2 run canbot {$processname}

## How do I run the golf car remotly ?

### Create a local network

For that, the user will need to create a LAN linking both the remote computer and the car's nvidia device.
The user can connect both device to the same router (be it a phone or a wifi device).
This, however, won't always work (for instance INRIA-Guest wifi network won't allow local communication somehow).

Another option can be to use ethernet link, through an ethernet cable.
Then the user will need to create a static ip address for each device :

$ sudo ifconfig eth0 (or enp2s0) {ipaddress}

For instance, in my case, the nvidia address was 192.168.9.2 while remote pc was 192.168.9.1

### Set up the CAN device on the nvidia device

In order to send data through the Controller Area Network device, the former must be initialised.
On the nvidia device, enter the following command : 

$ sudo ip link set can0 type can bitrate 500000

$ sudo ip link set up can0

Those commands use the socket-can API to initialise the device.
To aknowledge the proper functionning of the device, use the following command.

$ candump can0

It should return many number (most of them in hexadecimal) as long as it is connected to the car.

### Remote control of the car

In the nvidia device, the user will need to have at least two different terminal opened and sourced.
In the first one launch the remote control server :

$ ros2 run canbot remote

Then, on a sourced terminal of the remote computer use the following command :

$ ros2 run canbot input

The joystick's data are now sent from the remote computer to the nvidia device.
Now, the user will need to open a new node in order to send the data to the car through the CAN.
For that matter, use the following command on the nvidia device :

$ ros2 run canbot controller

Now, make sure that the key is in external position on the car.
If it is, then you are ready to go and to drive the car with the joystick.

### Getting car's situation

Most of the time, it might be useful for the user to get information about physical element such as speed.
It is possible to get thoses data from the CAN bus provided by the car, but they are not easily readed by a human.
This package provide a Node that will display the car status in an understandable format.

First, the user needs to open two new sourced terminals, one that will read the CAN, and one that will display the info.

$ ros2 run canbot reader

The launched node will read the can and print what is sent by the CAN bus and display it in a raw format.

$ ros2 run canbot status

This will print out the car status, it's speed, steering angle ... 
It will be updated every time the reader received a new and useful CAN bus information.