# elp-stereo-camera-ros-pkg

A ROS driver for the ELP Dual Lens stereo camera.


## Install from Source

These setup instructions assume that you have Ubuntu 14.04, have ROS Indigo or ROS Jade installed, and have a catkin workspace at `~/catkin_ws`. If you don't, follow the [Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) tutorial before proceeding. The following specific packages should be installed if they aren't already:

```bash
sudo apt-get update
sudo apt-get install ros-${ROS_DISTRO}-image-common ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-image-pipeline ros-${ROS_DISTRO}-usb-cam
```

Now you can install the elp_stereo_camera package:

```bash
git clone http://10.131.99.36/Other-projects/elp_stereo_camera.git  ~/catkin_ws/src/elp_stereo_camera

# setup udev rules
sudo cp ~/catkin_ws/src/elp_stereo_camera/debian/99-elp-stereo-camera.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
```

