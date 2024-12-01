refer to A1-QP-MPC-Controller

robot frame is target on relative movement
world frame is target on whole movement
状态估计部分参考mit源码https://blog.csdn.net/Everlasting_Aa/article/details/120920668

x -> roll
y -> pitch
z -> yaw

```bash
# compile
catkin_make --only-pkg-with-deps go1_mpc_control
# run gazebo
roslaunch unitree_gazebo normal.launch rname:=go1 wname:=earth
# run mpc control
roslaunch go1_mpc_control go1_ctrl.launch type:=gazebo solver_type:=mpc
```