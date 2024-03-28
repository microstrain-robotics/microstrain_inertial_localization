## Description

Collection of launch files and configuration files to allow for easy localization using [`microstrain_inertial`](https://github.com/LORD-MicroStrain/microstrain_inertial/tree/ros)

## Install Instructions

### Buildfarm

If you do not need to modify the source, it is recommended to install directly from the buildfarm by running the following commands where `ROS_DISTRO` is the version of ROS you are using such as `noetic`.

```bash
sudo apt-get update && sudo apt-get install ros-ROS_DISTRO-microstrain-inertial-localization
```

For more information on the ROS distros and platforms we support, please see [index.ros.org](https://index.ros.org/r/microstrain_inertial_navigation/github-microstrain-robotics-microstrain_inertial_navigation/#noetic)


### Source

If you need to modify the source of this repository, or are running on a platform that we do not support, you can build from source by following the [Building From Source](#building-from-source) guide below.

#### Building from source

1. Install ROS and create a workspace: [Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

2. Clone the repository into your workspace:
    ```bash
    git clone --branch ros https://github.com/microstrain-robotics/microstrain_inertial_localization.git ~/your_workspace/src/microstrain_inertial_localization
    ```

3. Install rosdeps for this package: `rosdep install --from-paths ~/your_workspace/src -i -r -y`

4. Build your workspace:

    ```bash        
    cd ~/your_workspace
    catkin_make
    source ~/your_workspace/devel/setup.bash
    ```
   The source command will need to be run in each terminal prior to launching a ROS node.

## Usage

Each use case is seperated into it's own launch file under the `launch` directory. These launch files each have their own set of configuration files under the `config` directory.
for more information on how to use these launch files please see the [ROS wiki](https://wiki.ros.org/microstrain_inertial_localization)