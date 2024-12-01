// stl
#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>

// control parameters
#include "utils/go1_params.h"
// Go1 control
#include "robot/gazebo_go1_ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_go1_mpc_ctrl");
    ros::NodeHandle nh;

    // change ros logger
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    // make sure the ROS infra using sim time, otherwise the controller cannot run with correct time steps
    std::string use_sim_time;
    if (ros::param::get("/use_sim_time", use_sim_time))
    {
        if (use_sim_time != "true")
        {
            std::cout << "ROS must set use_sim_time in order to use this program!" << std::endl;
            return -1;
        }
    }

    // create go1 controller
    std::unique_ptr<GazeboGo1ROS> go1 = std::make_unique<GazeboGo1ROS>(nh); // 智能指针自动释放内存

    std::atomic<bool> control_execute{}; // 原子变量，用于线程间同步
    control_execute.store(true, std::memory_order_release);

    // Thread 1: compute desired ground reaction forces
    std::cout << "Enter thread 1: compute desired ground forces" << std::endl;
    std::thread compute_foot_forces_grf_thread([&]()
                                               {
        // prepare variables to monitor time and control the while loop
        ros::Time prev = ros::Time::now();
        ros::Time now = ros::Time::now();
        ros::Duration dt(0);

        while (control_execute.load(std::memory_order_acquire) && ros::ok()) {
            ros::Duration(GRF_UPDATE_FREQUENCY / 1000).sleep(); // 固定时间间隔

            // get t and dt
            now = ros::Time::now();
            dt = now - prev;
            prev = now;

            auto t1 = std::chrono::high_resolution_clock::now();

            // compute desired ground forces
            bool running = go1->update_foot_forces_grf(dt.toSec());

            auto t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> ms_double = t2 - t1; // 测量的是实际计算任务的执行时间
            std::cout << "MPC solution is updated in " << ms_double.count() << "ms" << std::endl;

            if (!running) {
                std::cout << "Thread 1 loop is terminated because of errors." << std::endl;
                ros::shutdown();
                std::terminate();
                break;
            }
        } });

    // Thread 2: update robot states, compute desired swing legs forces, compute desired joint torques, and send commands
    std::cout << "Enter thread 2: update robot states, compute desired swing legs forces, compute desired joint torques, and send commands"
              << std::endl;
    std::thread main_thread([&]()
                            {
        // prepare variables to monitor time and control the while loop
        ros::Time start = ros::Time::now();
        ros::Time prev = ros::Time::now();
        ros::Time now = ros::Time::now();
        ros::Duration dt(0);

        while (control_execute.load(std::memory_order_acquire) && ros::ok()) {
            auto t3 = std::chrono::high_resolution_clock::now();

            ros::Duration(MAIN_UPDATE_FREQUENCY / 1000).sleep();

            // get t and dt
            now = ros::Time::now();
            dt = now - prev;
            prev = now;
            ros::Duration elapsed = now - start;

            // compute desired ground forces
            bool main_update_running = go1->main_update(elapsed.toSec(), dt.toSec());
            bool send_cmd_running = go1->send_cmd();

            auto t4 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> ms_double = t4 - t3; // 测量的是整个更新循环的总时间
            // std::cout << "Thread 2 is updated in " << ms_double.count() << "ms" << std::endl;

            if (!main_update_running || !send_cmd_running) {
                std::cout << "Thread 2 loop is terminated because of errors." << std::endl;
                ros::shutdown();
                std::terminate();
                break;
            }
        } });

    ros::AsyncSpinner spinner(16); // 创建异步spinner，用于并行处理回调
    spinner.start();

    compute_foot_forces_grf_thread.join();
    main_thread.join();

    return 0;
}
