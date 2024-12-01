#ifndef GAZEBO_GO1_ROS_H
#define GAZEBO_GO1_ROS_H

// calculation
#include <Eigen/Dense>

// std
#include <memory>
#include <set>
#include <chrono>
#include <map>
#include <fstream>

// threading
#include <mutex>
#include <thread>
#include <condition_variable>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
// #include <sensor_msgs/JointState.h>
// #include <nav_msgs/Odometry.h> // 包含里程计消息
// #include <geometry_msgs/TwistStamped.h>              // 包含速度消息
// #include <geometry_msgs/PoseWithCovarianceStamped.h> // 包含带协方差的位置消息
// #include <geometry_msgs/PoseArray.h>                 // 包含位置数组消息
// #include <geometry_msgs/Vector3Stamped.h>            // 包含三维向量消息
#include <geometry_msgs/WrenchStamped.h> // 包含力矩消息
// #include <geometry_msgs/PointStamped.h>  // 包含位置消息
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/LowCmd.h>

// control parameters
#include "go1_robot_control.h"
#include "go1_kinematics.h"
#include "utils/go1_params.h"
#include "utils/go1_ctrl_states.h"
#include "utils/go1_basicEKF.h"
#include "utils/utils.h"

#include "utils/filter.hpp"

class GazeboGo1ROS
{
public:
    GazeboGo1ROS(ros::NodeHandle &_nh);

    bool update_foot_forces_grf(double dt);

    bool main_update(double t, double dt);

    bool send_cmd(); // send control (joint) command to robot

    // callback functions
    void imu_callback(const sensor_msgs::Imu::ConstPtr &imu);

    void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg);

    void FL_hip_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state);

    void FL_thigh_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state);

    void FL_calf_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state);

    void FR_hip_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state);

    void FR_thigh_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state);

    void FR_calf_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state);

    void RL_hip_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state);

    void RL_thigh_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state);

    void RL_calf_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state);

    void RR_hip_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state);

    void RR_thigh_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state);

    void RR_calf_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state);

    void FL_foot_contact_callback(const geometry_msgs::WrenchStamped &force);

    void FR_foot_contact_callback(const geometry_msgs::WrenchStamped &force);

    void RL_foot_contact_callback(const geometry_msgs::WrenchStamped &force);

    void RR_foot_contact_callback(const geometry_msgs::WrenchStamped &force);

private:
    ros::NodeHandle nh;
    // Front Left, Front Right, Rear Left, Rear Right
    // 0,  1,  2: FL_hip, FL_thigh, FL_calf
    // 3,  4,  5: FR_hip, FR_thigh, FR_calf
    // 6,  7,  8: RL_hip, RL_thigh, RL_calf
    // 9, 10, 11: RR_hip, RR_thigh, RR_calf
    ros::Publisher pub_joint_cmd[12];
    ros::Subscriber sub_joint_msg[12];

    // 0, 1, 2, 3: FL, FR, RL, RR
    ros::Subscriber sub_foot_contact_msg[4];
    ros::Subscriber sub_gt_pose_msg;
    ros::Subscriber sub_imu_msg;
    ros::Subscriber sub_joy_msg;

    // joystick command
    double joy_cmd_velx = 0.0;
    double joy_cmd_velx_forward = 0.0;
    double joy_cmd_velx_backward = 0.0;
    double joy_cmd_vely = 0.0;
    double joy_cmd_velz = 0.0;

    double joy_cmd_pitch_rate = 0.0;
    double joy_cmd_roll_rate = 0.0;
    double joy_cmd_yaw_rate = 0.0;

    double joy_cmd_pitch_ang = 0.0;
    double joy_cmd_roll_ang = 0.0;
    double joy_cmd_body_height = 0.3;

    int joy_cmd_ctrl_state = 0; // 0 is standing, 1 is walking
    bool joy_cmd_ctrl_state_change_request = false;
    int prev_joy_cmd_ctrl_state = 0;
    bool joy_cmd_exit = false;

    // add leg kinematics
    // the leg kinematics is relative to body frame, which is the center of the robot
    // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
    double leg_offset_x[4] = {};
    double leg_offset_y[4] = {};
    double motor_offset[4] = {};
    // for each leg, there is a default foot position in the body frame
    double upper_leg_length[4] = {};
    double lower_leg_length[4] = {};
    std::vector<Eigen::VectorXd> rho_fix_list;
    Go1Kinematics go1_kin;

    // variables related to control
    Go1CtrlStates go1_ctrl_states;
    Go1RobotControl robot_control;
    Go1BasicEKF go1_estimate;

    // filters
    MovingWindowFilter acc_x;
    MovingWindowFilter acc_y;
    MovingWindowFilter acc_z;
    MovingWindowFilter gyro_x;
    MovingWindowFilter gyro_y;
    MovingWindowFilter gyro_z;
    MovingWindowFilter quat_w;
    MovingWindowFilter quat_x;
    MovingWindowFilter quat_y;
    MovingWindowFilter quat_z;
};

#endif // GAZEBO_GO1_ROS_H
