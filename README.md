# fogg

Fast Occupancy Grid Generation Using a Kinect

![fogg](docs/seg_with_og.png)

# Installation

This installation has been tested using Ubuntu 14.04 running ROS Indigo. First
you need to install the Freenect libraries used for accessing the Kinect

    sudo apt-get install ros-indigo-freenect-*

Then you can download and build the fogg ROS package

    cd $CATKIN_WS/src
    git clone https://github.com/mit-drl/fogg.git
    cd ..
    catkin_make

# Running

A launch file is provided for running fogg using a Kinect.

    roslaunch fogg fogg.launch
