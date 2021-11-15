## Introduction

This package hosts the configuration files, models, and programs for simulating 
and use the Nachi mz04 robot. We provide:


## Install

1. Create a symlink from this package's `nachi_ros` folder to the `src` folder of your
   ROS workspace like: `ln -s /home/$USER/nachi_dual_arm/nachi_ros /home/$USER/catkin_ws/src/nachi_ros`.
4. Create a symlink from the `dual_arm_api` to the `controllers` folder under `nachi_webots`:
   `ln -s /home/$USER/nachi_dual_arm/nachi_ros/dual_arm_api/ /home/$USER/nachi_dual_arm/nachi_webots/controllers/dual_arm_api`
5. Add the line `export CURIOSITY_HOME=/home/$USER/nachi_dual_arm` into your `.bashrc` or `.zshrc` file. 
   This assumes that you have put this repository to your home folder. You can change the path accordingly.

## Usage

### Start the simulation

You can start a simulation scenario for the Curiosity robot with the command:

```shell script
roslaunch dual_arm_bringup dual_arm_webots.launch
```

