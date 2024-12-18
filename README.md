refer to A1-QP-MPC-Controller
https://github.com/ShuoYangRobotics/A1-QP-MPC-Controller.git
robot frame is target on relative movement
world frame is target on whole movement
状态估计部分参考mit源码https://blog.csdn.net/Everlasting_Aa/article/details/120920668
pd控制与轨迹规划部分参考https://blog.csdn.net/weixin_45728705/article/details/120817881


x -> roll
y -> pitch
z -> yaw

dependence
- ubuntu 18.04
- unitree_legged_sdk v3.8.0
- unitree_ros 
- ros

```bash
# ros install
wget http://fishros.com/install -O fishros && . fishros
```

配置完以上依赖后在~/.bashrc后添加，假设workspace是在~/go1_ws下
```bash
# Go1 gazebo
source ~/go1_ws/devel/setup.bash
source /usr/share/gazebo-8/setup.sh
export ROS_PACKAGE_PATH=~/go1_ws:${ROS_PACKAGE_PATH}
export GAZEBO_PLUGIN_PATH=~/go1_ws/devel/lib:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=~/go1_ws/devel/lib:${LD_LIBRARY_PATH}
export UNITREE_LEGGED_SDK_PATH=path-to/unitree_legged_sdk
#amd64, arm32, arm64
export UNITREE_PLATFORM="amd64"
```

```bash
# compile
catkin_make --only-pkg-with-deps go1_mpc_control
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
# run gazebo
roslaunch unitree_gazebo normal.launch rname:=go1 wname:=earth
# reset robot
rosrun unitree_controller unitree_move_kinetic
rosrun unitree_controller unitree_servo
# run mpc control
roslaunch go1_mpc_control go1_ctrl.launch type:=gazebo solver_type:=mpc
```