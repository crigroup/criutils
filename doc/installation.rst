************
Installation
************

The following instructions have been tested in **Ubuntu 16.04** (Xenial), 64
bits.

ROS Kinetic
===========

Set-up your computer to accept software from *packages.ros.org* and set-up your
keys following the steps described at
http://wiki.ros.org/kinetic/Installation/Ubuntu

Now, install the ROS bare bones::

  # Installation
  sudo apt-get update
  sudo apt-get install python-wstool ros-kinetic-ros-base
  # Initialize rosdep
  sudo rosdep init
  rosdep update

ROS Package installation
========================

Go to your ROS working directory::

  cd ~/catkin_ws/src

Clone this repository::

  git clone https://github.com/crigroup/criutils.git

Install any missing dependencies using rosdep::

  rosdep update
  rosdep install --from-paths . --ignore-src -y

Now, compile your ROS workspace::

  cd ~/catkin_ws && catkin_make

Testing the Installation
========================

Make sure you always source the appropriate ROS setup file, e.g::

  source ~/catkin_ws/devel/setup.bash

The following will run the tests of the ``criutils`` package::

  roscd criutils/tests
  nosetests -v
