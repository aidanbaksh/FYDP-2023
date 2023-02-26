# ROS Noetic
**This is a custom installation of ROS Noetic on Ubuntu 18.04**

ROS Noetic is not supported on Ubuntu 18.04, only Ubuntu 20.04. However, the NVIDIA Jetson Nano only officially supports Ubuntu 18.04. The Jetson image installer (Jetpack) only supports 20.04 in version 5, which is targetted at newer Jetsons, not the Nano.

**The installation of ROS Noetic is based on [this guide](https://vsbogd.github.io/coding/install-rospy-noetic-ubuntu-1804.html).**

## Adding New Noetic Packages
If you need a noetic package (`sensor_msgs`, `perception_pcl`, etc.) perform the following:

```
cd $HOME/noetic_install
rosinstall_generator <pkg> --rosdistro noetic --deps --tar > <pkg>.rosinstall
vcs import --input <pkg>.rosinstall ./src
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y
./src/catkin/bin/catkin_make_isolated --install -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
```

For changes to take effect, source `setup.bash` or restart your shell.
```
source $HOME/noetic_install/install_isolated/setup.bash
```