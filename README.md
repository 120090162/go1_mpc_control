refer to A1-QP-MPC-Controller
https://github.com/ShuoYangRobotics/A1-QP-MPC-Controller.git

robot frame is target on relative movement

world frame is target on whole movement

状态估计部分参考mit源码
https://blog.csdn.net/Everlasting_Aa/article/details/120920668

pd控制与轨迹规划部分参考
https://blog.csdn.net/weixin_45728705/article/details/120817881


x -> roll
y -> pitch
z -> yaw

dependence
- ubuntu 18.04
- unitree_legged_sdk v3.8.0
- unitree_ros 
- ros melodic

```bash
# ros install
wget http://fishros.com/install -O fishros && . fishros
```
详细配置见[setup](./环境配置.md)

配置完以上依赖后在`~/.bashrc`后添加
```bash
# Go1 gazebo
source /usr/share/gazebo-9/setup.sh
export UNITREE_LEGGED_SDK_PATH=path-to/unitree_legged_sdk
#amd64, arm32, arm64
export UNITREE_PLATFORM="amd64"
```

安装
假设workspace是在~/go1_ws下
```bash
sudo apt-get install ros-melodic-controller-interface  ros-melodic-gazebo-ros-control ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-joint-trajectory-controller
```
```bash
cd go1_ws/src
git clone --recursive https://github.com/unitreerobotics/unitree_ros.git
git clone https://github.com/120090162/go1_mpc_control.git
git clone https://github.com/120090162/keyboard_input.git
```
And open the file `unitree_gazebo/worlds/stairs.world`. At the end of the file:
```bash
<include>
    <uri>model:///home/unitree/catkin_ws/src/unitree_ros/unitree_gazebo/worlds/building_editor_models/stairs</uri>
</include>
```
Please change the path of `building_editor_models/stairs` to the real path on your PC.

并在编译之前需要去`path-to/unitree_ros/unitree_gazebo/plugin/draw_force_plugin.cc`文件的47，48行将`common::Color`替换为`ignition::math::Color`。
```bash
# compile
cd ~/go1_ws
catkin_make --only-pkg-with-deps unitree_legged_msgs # 避免unitree_legged_msgs报错
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```
如果你只想编译一个包
```bash
catkin_make --only-pkg-with-deps go1_mpc_control
```
如果你想编译整个工作空间
```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```
运行
```bash
# run gazebo
roslaunch unitree_gazebo normal.launch rname:=go1 wname:=earth
# reset robot
rosrun unitree_controller unitree_move_kinetic # place the robot back to origin
rosrun unitree_controller unitree_servo  # let the robot stretch legs
# 启动键盘控制
rosrun keyboard_input keyboard_input_node
# run mpc control
roslaunch go1_mpc_control go1_ctrl.launch type:=gazebo solver_type:=mpc
```
其中键盘的 `f` 键用来切换 stand/tort 步态，详细的控制看 `keyboard_input` 的代码
