refer to A1-QP-MPC-Controller

robot frame is target on relative movement
world frame is target on whole movement
状态估计部分参考mit源码https://blog.csdn.net/Everlasting_Aa/article/details/120920668
pd控制与轨迹规划部分参考https://blog.csdn.net/weixin_45728705/article/details/120817881


x -> roll
y -> pitch
z -> yaw

dependence
- unitree_legged_sdk v3.8.0
- unitree_ros 
- ros noetic

```bash
fishrc
```

配置完以上依赖后在~/.bashrc后添加
```bash
# Go1 gazebo
source ~/go1_ws/devel/setup.zsh
source /usr/share/gazebo/setup.sh
export UNITREE_LEGGED_SDK_PATH=path-to/unitree_legged_sdk
#amd64, arm32, arm64
export UNITREE_PLATFORM="amd64"
```

```bash
# compile
catkin_make --only-pkg-with-deps go1_mpc_control
# run gazebo
roslaunch unitree_gazebo normal.launch rname:=go1 wname:=earth
# run mpc control
roslaunch go1_mpc_control go1_ctrl.launch type:=gazebo solver_type:=mpc
```