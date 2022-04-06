# FYP_ws

This is nachi robot's webots simulation package for FYP.

 ## Introduction
 
This package hosts the configuration files, models, and programs for simulating and use the nachi robot. We provide:

- Webots proto files in [nachi_webots](nachi_webots)
- ROS control API in [nachi_ros](nachi_ros)

## Dependencies

1. Install the latest version of [Webots](https://cyberbotics.com/)

2. Install ROS dependencies:

   ```shell script
   sudo apt-get install ros-$ROS_DISTRO-webots-ros
   ```
3. Install RoTools-v2 dependencies:
- please follow readme in RoTools-v2 package.

## Install

1. Download the source code from .
2. Download the nachi_webots from the [link](https://pan.baidu.com/s/1dnXgXcDh4rI3FDsxc8uu2w), password: s67k
3. Add the line `export DUAL_ARM_HOME=/home/$USER/nachi_dual_arm` into your `.bashrc` or `.zshrc` file. 
   This assumes that you have put this repository to your home folder. You can change the path accordingly.
   
## Usage
Firstly, create and build a catkin workspace:
```shell script
$ mkdir -p ~/FYP_ws/src
$ cd ~/FYP_ws/
# put codes in this folder then build
$ catkin_make
```
then  source your workspace, add the following code to your .bashrc file:
```shell script
source ~/FYP_ws/devel/setup.bash
```
## 从这里开始：

搭建自己的webots虚拟环境。另存dual_arm_demo_v1.wbt文件进行自己虚拟环境的搭建，然后命名保存，如dual_arm_demo_(your name).wbt
保存的文件会在/home/hp/nachi_dual_arm/nachi_webots/worlds文件夹下。

webots的使用请参考：
[webots Tutorial 1, 2, 3](https://cyberbotics.com/doc/guide/tutorial-1-your-first-simulation-in-webots)

**注意：** 当修改world文件时，必须确保仿真是暂停的，并且虚拟时间为0（the simulation is paused and is at a virtual time of 0.） 
，如果不为0，需要点击reset simulation按钮，然后修改。

当想要保存修改的world时，可以点击File / Save World， 
- 导入自己的工件

从solidworks中导出的stl文件，单位必须为**m**。
再导入webots中，保持默认选项，![导入](./src/doc/solid.png)

然后webots scene tree中会出现导入的物体，

![solod](./src/doc/import.png)

将children中的solid复制出来，paste到scene tree的最后，将原有的solid删除，这样可以避免定义两次bounding object 和physics。

- 定义bounding object和physics

![bounding object](./src/doc/bounding%20box.png)

可以点击 view/optional rendering/show all bounding box, 查看 bounding box与物体模型的重合程度，近似就行，不用那么精确。
- 拍照位置

为了能够得到合适的物体图片，需要确定工件和相机的位置。
相机的位置可以图中所示的translation和rotation改变。



设置好之后机器人就能抓取物体了。

![camera](./src/doc/camera.png)

### 1. Start the Webots simulation

如果想启动自己的虚拟环境，需要修改dual_arm_webots,launch文件，

```shell script
<arg name="world" default="dual_arm_demo_v1"/> 更改为你的.wbt文件名称
```
然后重新运行以下命令。
You can start a simulation scenario for the nachi robot with the command:

```shell script
roslaunch dual_arm_bringup dual_arm_webots.launch
```
This command takes a `world` argument, by default, it will be `default`.

**then press start button in webots. 点击开始按钮。** 
如果不开始运行仿真的话，后面三个终端的程序无法启动。


### 2. Start MoveIt! package
```shell script
roslaunch dual_arm_bringup dual_arm_moveit.launch

当界面出现 you can start planning now! 说明启动成功!

否则说明webots开始按钮没有按下。
```
### 3. Start roport functional package
```shell script
roslaunch dual_arm_bringup dual_arm_roport.launch

当界面出现 Roport: MoveIt sever ready! 说明启动成功!
```
### 4. Start python control demo program
```shell script
# you can create a virtual environment to run this command
# 现在可以直接使用已经建好的虚拟环境， 输入：
dp2.7
# move to dual_arm_bringup/scripts folder
roscd dual_arm_bringup/scripts
# 启动python脚本
python dual_arm_grasp_test.py
```

## 注意：

四条运行命令必须依次在四个终端中运行，只有上一个命令运行成功之后，才能运行下一个命令。如果前三个终端中出现错误，请重启所有前三个终端中。

在dual_arm_grasp_test.py的simple_test的函数中：

### 1.
```shell script
pose0 = Transform.from_list([0.35, 0.1, 0.05, 1, 0, 0, 0])
该函数共有七个参数, 分别为在机器人基坐标系下的位置(x,y,z)和姿态(qx,qy,qz,qw)
```
具体请参考:

![机器人坐标系](./src/doc/robot_frame.png)

### 2
```shell script
trajectory1 = [pose1, pose2] # trajectory1 包含pose1, pose2两个位置点
self.execute_trajectory(group_name, trajectory1) #让机器人执行trajectory1运动
```

### 3
关于使用角点检测，来计算判断距离，请参考detect_test1.py
``` python
import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

filename = './doc/image4.png'
img = cv.imread(filename)
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
corners = cv.goodFeaturesToTrack(gray, 25, 0.01, 10)
corners = np.int0(corners)
corners = np.squeeze(corners)
# 按照x坐标排序
sort_corners = corners[np.argsort(corners[:, 0])]
# print(sort_corners.shape, sort_corners)
for i in sort_corners:
    x, y = i.ravel()
    cv.circle(img, (x, y), 9, (0, 0, 255), -1)
plt.imshow(img[:, :, ::-1])
plt.savefig('./doc/demo.png')
plt.show()

print(f'第一条边和第四条边之间的距离为{sort_corners[0, 1]- sort_corners[-1, 1]} pixel')

```
最终效果如下：
![detect_demo](./src/doc/demo.png)