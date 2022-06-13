#README

The following section will walk the reader through the steps to setup the ROS workspace for this project.
This tutorial is intended for unix-based terminals.

It is assumed that ROS-melodic is installed on the Jetson.

SSH into the Jetson or connect keyboard, mouse and monitor to the Jetson and open a terminal in the the Linux GUI and follow these steps:


1. Clone the repository into a folder on the Jetson

cd ~/sandbox (or whatever folder you want to work in)
git clone https://github.com/jepp5220/f1tenth.git


2. Create workspace

mkdir -p f1tenth_ws/src
cp -r f1tenth_system f1tenth_ws/src/


3. Install additional ROS packages

sudo apt-get update
sudo apt-get install ros-melodic-driver-base


4. Make Python scripts executeable

cd f1tenth_ws
find . -name “*.py” -exec chmod +x {} \;


5. Move to workspace folder and compile the code 

catkin_make

6. Source directory

source devel/setup.bash

Now we are ready to launch the car.
