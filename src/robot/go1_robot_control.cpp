#include "robot/go1_robot_control.h"

Go1RobotControl::Go1RobotControl()
{
    std::cout << "init Go1RobotControl" << std::endl;
    // init some parameters
    // init QP solver
    Q.diagonal() << 1.0, 1.0, 1.0, 400.0, 400.0, 100.0;
    R = 1e-3;
    hessian.resize(3 * NUM_LEG, 3 * NUM_LEG);
    gradient.resize(3 * NUM_LEG);
    linearMatrix.resize(NUM_LEG + 4 * NUM_LEG, 3 * NUM_LEG);
    lowerBound.resize(NUM_LEG + 4 * NUM_LEG);
    lowerBound.setZero();
    upperBound.resize(NUM_LEG + 4 * NUM_LEG);
    upperBound.setZero();

    spline_time.setZero();

    // init mpc skip counter
    mpc_init_counter = 0;

    // init friction coefficient
    mu = 0.7;
    F_min = 0;
    F_max = 180;

    // constraint matrix fixed [F_zi(4), F_xi(4x2), F_yi(4x2)]
    for (int i = 0; i < NUM_LEG; ++i)
    {
        // extract F_zi
        linearMatrix.insert(i, 2 + i * 3) = 1;
        // friction pyramid
        // 1. F_xi < uF_zi ===> F_xi -uF_zi < 0
        linearMatrix.insert(NUM_LEG + i * 4, i * 3) = 1;
        linearMatrix.insert(NUM_LEG + i * 4, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4) = -OsqpEigen::INFTY; // 避免足部力Fz为负
        // 2. F_xi > -uF_zi ===> -F_xi -uF_zi < 0
        linearMatrix.insert(NUM_LEG + i * 4 + 1, i * 3) = -1;
        linearMatrix.insert(NUM_LEG + i * 4 + 1, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4 + 1) = -OsqpEigen::INFTY;
        // 3. F_yi < uF_zi ===> F_yi -uF_zi < 0
        linearMatrix.insert(NUM_LEG + i * 4 + 2, 1 + i * 3) = 1;
        linearMatrix.insert(NUM_LEG + i * 4 + 2, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4 + 2) = -OsqpEigen::INFTY;
        // 4. F_yi > -uF_zi ===> -F_yi -uF_zi < 0
        linearMatrix.insert(NUM_LEG + i * 4 + 3, 1 + i * 3) = -1;
        linearMatrix.insert(NUM_LEG + i * 4 + 3, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4 + 3) = -OsqpEigen::INFTY;
    }

    terrain_angle_filter = MovingWindowFilter(100);
    for (int i = 0; i < NUM_LEG; ++i)
    {
        recent_contact_x_filter[i] = MovingWindowFilter(60);
        recent_contact_y_filter[i] = MovingWindowFilter(60);
        recent_contact_z_filter[i] = MovingWindowFilter(60);
    }
}

Go1RobotControl::Go1RobotControl(ros::NodeHandle &_nh) : Go1RobotControl()
{
    std::cout << "init nh" << std::endl;
    nh = _nh;
    _nh.param("use_sim_time", use_sim_time);
    // initial debug publisher
    for (int i = 0; i < NUM_LEG; ++i)
    {
        std::string id = std::to_string(i);
        std::string start_topic = "/gazebo_go1/foot" + id + "/start_pos";
        std::string end_topic = "/gazebo_go1/foot" + id + "/end_pos";
        std::string path_topic = "/gazebo_go1/foot" + id + "/swing_path";

        pub_foot_start[i] = nh.advertise<visualization_msgs::Marker>(start_topic, 100);
        pub_foot_end[i] = nh.advertise<visualization_msgs::Marker>(end_topic, 100);
        pub_foot_path[i] = nh.advertise<visualization_msgs::Marker>(path_topic, 100);

        // set basic info of markers
        foot_start_marker[i].header.frame_id = "go1_world";
        foot_start_marker[i].ns = "basic_shapes";
        foot_start_marker[i].id = 10 + i;
        foot_start_marker[i].type = visualization_msgs::Marker::CYLINDER;
        foot_start_marker[i].action = visualization_msgs::Marker::ADD;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        foot_start_marker[i].scale.x = 0.08;
        foot_start_marker[i].scale.y = 0.08;
        foot_start_marker[i].scale.z = 0.02;
        foot_start_marker[i].pose.orientation.x = 0.0;
        foot_start_marker[i].pose.orientation.y = 0.0;
        foot_start_marker[i].pose.orientation.z = 0.0;
        foot_start_marker[i].pose.orientation.w = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        foot_start_marker[i].color.r = 1.0f;
        foot_start_marker[i].color.g = 0.0f;
        foot_start_marker[i].color.b = 0.0f;
        foot_start_marker[i].color.a = 1.0;

        foot_end_marker[i].lifetime = ros::Duration();

        foot_end_marker[i].header.frame_id = "go1_world";
        foot_end_marker[i].ns = "basic_shapes";
        foot_end_marker[i].id = 20 + i;
        foot_end_marker[i].type = visualization_msgs::Marker::CYLINDER;
        foot_end_marker[i].action = visualization_msgs::Marker::ADD;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        foot_end_marker[i].scale.x = 0.08;
        foot_end_marker[i].scale.y = 0.08;
        foot_end_marker[i].scale.z = 0.02;
        foot_end_marker[i].pose.orientation.x = 0.0;
        foot_end_marker[i].pose.orientation.y = 0.0;
        foot_end_marker[i].pose.orientation.z = 0.0;
        foot_end_marker[i].pose.orientation.w = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        foot_end_marker[i].color.r = 0.0f;
        foot_end_marker[i].color.g = 0.0f;
        foot_end_marker[i].color.b = 1.0f;
        foot_end_marker[i].color.a = 1.0;

        foot_end_marker[i].lifetime = ros::Duration();

        foot_path_marker[i].header.frame_id = "go1_world";
        foot_path_marker[i].ns = "basic_shapes";
        foot_path_marker[i].id = 30 + i;
        foot_path_marker[i].type = visualization_msgs::Marker::LINE_STRIP;
        foot_path_marker[i].action = visualization_msgs::Marker::ADD;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        foot_path_marker[i].scale.x = 0.02;
        foot_path_marker[i].pose.position.x = 0.0;
        foot_path_marker[i].pose.position.y = 0.0;
        foot_path_marker[i].pose.position.z = 0.0;
        foot_path_marker[i].pose.orientation.w = 1.0;
        foot_path_marker[i].pose.orientation.x = 0.0;
        foot_path_marker[i].pose.orientation.y = 0.0;
        foot_path_marker[i].pose.orientation.z = 0.0;
        foot_path_marker[i].points.resize(10); // fix to be 10 points
        foot_path_marker[i].colors.resize(10); // fix to be 10 points
        for (int k = 0; k < 10; k++)
        {
            foot_path_marker[i].colors[k].r = 0.0f;
            foot_path_marker[i].colors[k].g = 1.0f;
            foot_path_marker[i].colors[k].b = 0.0f;
            foot_path_marker[i].colors[k].a = 1.0f;
        }

        foot_path_marker[i].lifetime = ros::Duration();
    }
    pub_terrain_angle = nh.advertise<std_msgs::Float64>("go1_debug/terrain_angle", 100);
}

void Go1RobotControl::update_plan(Go1CtrlStates &state, double dt)
{
    state.counter += 1;
    if (!state.movement_mode)
    {
        // movement_mode == 0, standstill with all feet in contact with ground
        for (bool &plan_contact : state.plan_contacts)
            plan_contact = true;
        state.gait_counter_reset();
    }
    else
    {
        // movement_mode == 1, walk
        for (int i = 0; i < NUM_LEG; ++i)
        {
            state.gait_counter(i) += state.gait_counter_speed(i);
            state.gait_counter(i) = std::fmod(state.gait_counter(i), state.counter_per_gait); // %
            if (state.gait_counter(i) <= state.counter_per_swing)                             // stand
            {
                state.plan_contacts[i] = true;
            }
            else
            {
                state.plan_contacts[i] = false;
            }
        }
    }

    // update foot plan: state.foot_pos_target_world
    Eigen::Vector3d lin_vel_rel = state.root_rot_mat_z.transpose() * state.root_lin_vel; // robot body frame linear velocity, 一般只考虑水平旋转即yaw轴旋转

    // Raibert Heuristic, calculate foothold position
    // delta_x = sqrt(abs(z) / g) * (vx - vx_d) + 1/2 * T_swing * vx_d
    // delta_y = sqrt(abs(z) / g) * (vy - vy_d) + 1/2 * T_swing * vy_d
    state.foot_pos_target_rel = state.default_foot_pos;
    for (int i = 0; i < NUM_LEG; ++i)
    {
        double delta_x =
            std::sqrt(std::abs(state.default_foot_pos(2)) / ROBOT_GRAVITY) * (lin_vel_rel(0) - state.root_lin_vel_d(0)) +
            ((state.counter_per_swing / state.gait_counter_speed(i)) * state.control_dt) / 2.0 *
                state.root_lin_vel_d(0);
        double delta_y =
            std::sqrt(std::abs(state.default_foot_pos(2)) / ROBOT_GRAVITY) * (lin_vel_rel(1) - state.root_lin_vel_d(1)) +
            ((state.counter_per_swing / state.gait_counter_speed(i)) * state.control_dt) / 2.0 *
                state.root_lin_vel_d(1);
        // constrain delta_x, delta_y
        if (delta_x < -FOOT_DELTA_X_LIMIT)
        {
            delta_x = -FOOT_DELTA_X_LIMIT;
        }
        else if (delta_x > FOOT_DELTA_X_LIMIT)
        {
            delta_x = FOOT_DELTA_X_LIMIT;
        }
        if (delta_y < -FOOT_DELTA_Y_LIMIT)
        {
            delta_y = -FOOT_DELTA_Y_LIMIT;
        }
        else if (delta_y > FOOT_DELTA_Y_LIMIT)
        {
            delta_y = FOOT_DELTA_Y_LIMIT;
        }

        state.foot_pos_target_rel(0, i) += delta_x;
        state.foot_pos_target_rel(1, i) += delta_y;

        state.foot_pos_target_abs.block<3, 1>(0, i) = state.root_rot_mat * state.foot_pos_target_rel.block<3, 1>(0, i);
        state.foot_pos_target_world.block<3, 1>(0, i) = state.foot_pos_target_abs.block<3, 1>(0, i) + state.root_pos;
    }
}

void Go1RobotControl::generate_swing_legs_ctrl(Go1CtrlStates &state, double dt)
{
    state.joint_torques.setZero();
    foot_pos_target.setZero();
    foot_vel_target.setZero();

    for (int i = 0; i < NUM_LEG; ++i)
    {
        // TODO: make sure use root_rot_mat_z instead of root_rot_mat
        foot_pos_cur.block<3, 1>(0, i) = state.root_rot_mat_z.transpose() * state.foot_pos_abs.block<3, 1>(0, i);

        // from foot_pos_cur to foot_pos_final computes an intermediate point using BezierUtils
        if (state.gait_counter(i) <= state.counter_per_swing)
        {
            // stance foot
            spline_time(i) = 0.0;
            // in this case the foot should be stance
            // keep refreshing foot_pos_start in stance mode
            state.foot_pos_start.block<3, 1>(0, i) = foot_pos_cur.block<3, 1>(0, i);
        }
        else
        {
            // in this case the foot should be swing
            spline_time(i) = float(state.gait_counter(i) - state.counter_per_swing) / float(state.counter_per_swing);
        }

        foot_pos_target.block<3, 1>(0, i) = bezierUtils[i].get_foot_pos_curve(spline_time(i),
                                                                              state.foot_pos_start.block<3, 1>(0, i),
                                                                              state.foot_pos_target_rel.block<3, 1>(0, i),
                                                                              0.0);

        foot_vel_cur.block<3, 1>(0, i) = (foot_pos_cur.block<3, 1>(0, i) - state.foot_pos_rel_last_time.block<3, 1>(0, i)) / dt;
        state.foot_pos_rel_last_time.block<3, 1>(0, i) = foot_pos_cur.block<3, 1>(0, i);

        foot_vel_target.block<3, 1>(0, i) = (foot_pos_target.block<3, 1>(0, i) - state.foot_pos_target_last_time.block<3, 1>(0, i)) / dt;
        state.foot_pos_target_last_time.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i);

        foot_pos_error.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i) - foot_pos_cur.block<3, 1>(0, i);
        foot_vel_error.block<3, 1>(0, i) = foot_vel_target.block<3, 1>(0, i) - foot_vel_cur.block<3, 1>(0, i);
        // pd torque control, 修正力
        foot_forces_kin.block<3, 1>(0, i) = foot_pos_error.block<3, 1>(0, i).cwiseProduct(state.kp_foot.block<3, 1>(0, i)) +
                                            foot_vel_error.block<3, 1>(0, i).cwiseProduct(state.kd_foot.block<3, 1>(0, i));
    }
    state.foot_pos_cur = foot_pos_cur;

    // detect early contact
    for (int i = 0; i < NUM_LEG; ++i)
    {
        // 判断是否处于摆动后期阶段
        if (state.gait_counter(i) <= state.counter_per_swing * 1.5)
        {
            state.early_contacts[i] = false;
        }
        // 判断在摆动后期阶段是否有足部力大于阈值
        if (!state.plan_contacts[i] &&
            (state.gait_counter(i) > state.counter_per_swing * 1.5) &&
            (state.foot_force(i) > FOOT_FORCE_LOW))
        {
            state.early_contacts[i] = true;
        }

        // actual contact
        state.contacts[i] = state.plan_contacts[i] || state.early_contacts[i];

        // record recent contact position if the foot is in touch with the ground
        if (state.contacts[i])
        {
            // state.foot_pos_recent_contact.block<3, 1>(0, i) = state.root_rot_mat.transpose() * (state.foot_pos_world.block<3, 1>(0, i));
            // state.foot_pos_recent_contact.block<3, 1>(0, i) = state.foot_pos_abs.block<3, 1>(0, i);
            state.foot_pos_recent_contact.block<3, 1>(0, i)
                << recent_contact_x_filter[i].CalculateAverage(state.foot_pos_abs(0, i)),
                recent_contact_y_filter[i].CalculateAverage(state.foot_pos_abs(1, i)),
                recent_contact_z_filter[i].CalculateAverage(state.foot_pos_abs(2, i));
        }
    }

    std::cout << "foot_pos_recent_contact z: " << state.foot_pos_recent_contact.block<1, 4>(2, 0) << std::endl;

    state.foot_forces_kin = foot_forces_kin;
}

void Go1RobotControl::compute_joint_torques(Go1CtrlStates &state)
{
    joint_torques.setZero();
    mpc_init_counter++;
    mpc_init_counter = std::min(mpc_init_counter, 10000); // avoid overflow
    // for the first 10 ticks, just return zero torques.
    if (mpc_init_counter < 10)
    {
        state.joint_torques = joint_torques;
    }
    else
    {
        // for each leg, if it is a swing leg (contact[i] is false), use foot_force_kin to get joint_torque
        // for each leg, if it is a stance leg (contact[i] is true), use foot_forces_grf to get joint_torque
        for (int i = 0; i < NUM_LEG; ++i)
        {
            Eigen::Matrix3d jac = state.j_foot.block<3, 3>(3 * i, 3 * i);
            if (state.contacts[i])
            {
                // stance leg
                joint_torques.segment<3>(i * 3) = jac.transpose() * -state.foot_forces_grf.block<3, 1>(0, i);
            }
            else
            {
                // swing leg
                Eigen::Vector3d force_tgt = state.km_foot.cwiseProduct(state.foot_forces_kin.block<3, 1>(0, i)); // 考虑足部质量和惯性的参数矩阵
                joint_torques.segment<3>(i * 3) = jac.lu().solve(force_tgt);                                     // jac * tau = F
            }
        }
        // gravity compensation
        joint_torques += state.torques_gravity;

        // prevent nan
        for (int i = 0; i < 12; ++i)
        {
            if (!isnan(joint_torques[i]))
                state.joint_torques[i] = joint_torques[i];
        }
    }
}

Eigen::Matrix<double, 3, NUM_LEG> Go1RobotControl::compute_grf(Go1CtrlStates &state, double dt)
{
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;
    // first get parameters needed to construct the solver hessian and gradient
    // use euler angle to get desired angle
    Eigen::Vector3d euler_error = state.root_euler_d - state.root_euler;

    // limit euler error to pi/2
    // 限制偏航角误差在-pi/2到pi/2之间
    if (euler_error(2) > PI * 1.5)
    {
        euler_error(2) = state.root_euler_d(2) - PI * 2 - state.root_euler(2);
    }
    else if (euler_error(2) < -PI * 1.5)
    {
        euler_error(2) = state.root_euler_d(2) + PI * 2 - state.root_euler(2);
    }

    // only do terrain adaptation in MPC
    if (state.stance_leg_control_type == 1)
    {
        Eigen::Vector3d surf_coef = compute_walking_surface(state);
        Eigen::Vector3d flat_ground_coef;
        flat_ground_coef << 0, 0, 1;
        double terrain_angle = 0;
        // only record terrain angle when the body is high
        if (state.root_pos[2] > 0.1)
        {
            terrain_angle = terrain_angle_filter.CalculateAverage(Utils::cal_dihedral_angle(flat_ground_coef, surf_coef));
        }
        else
        {
            terrain_angle = 0;
        }
        // 限制地形角度在-0.5到0.5之间
        if (terrain_angle > 0.5)
        {
            terrain_angle = 0.5;
        }
        if (terrain_angle < -0.5)
        {
            terrain_angle = -0.5;
        }

        // FL, FR, RL, RR
        // 计算前后脚的z轴角度差
        double F_R_diff = state.foot_pos_recent_contact(2, 0) + state.foot_pos_recent_contact(2, 1) - state.foot_pos_recent_contact(2, 2) -
                          state.foot_pos_recent_contact(2, 3);

        // 默认启用地形适应
        if (state.use_terrain_adapt)
        {
            if (F_R_diff > 0.05)
            {
                state.root_euler_d[1] = -terrain_angle;
            }
            else
            {
                state.root_euler_d[1] = terrain_angle;
            }
        }

        std_msgs::Float64 terrain_angle_msg;
        terrain_angle_msg.data = terrain_angle * (180 / PI);
        pub_terrain_angle.publish(terrain_angle_msg); // publish in deg
        // 打印期望的俯仰角
        std::cout << "desire pitch in deg: " << state.root_euler_d[1] * (180 / PI) << std::endl;
        std::cout << "terrain angle: " << terrain_angle << std::endl;

        // save calculated terrain pitch angle
        // TODO: limit terrain pitch angle to -30 to 30?
        state.terrain_pitch_angle = terrain_angle;
    }
    // solve grf using QP or MPC
    if (state.stance_leg_control_type == 0)
    { // 0: QP
        // desired acc in world frame
        root_acc.setZero();
        root_acc.block<3, 1>(0, 0) = state.kp_linear.cwiseProduct(state.root_pos_d - state.root_pos);

        root_acc.block<3, 1>(0, 0) += state.root_rot_mat * state.kd_linear.cwiseProduct(
                                                               state.root_lin_vel_d - state.root_rot_mat.transpose() * state.root_lin_vel);

        root_acc.block<3, 1>(3, 0) = state.kp_angular.cwiseProduct(euler_error);

        root_acc.block<3, 1>(3, 0) += state.kd_angular.cwiseProduct(
            state.root_ang_vel_d - state.root_rot_mat.transpose() * state.root_ang_vel);

        // add gravity
        root_acc(2) += state.robot_mass * ROBOT_GRAVITY;

        // Create inverse inertia matrix
        Eigen::Matrix<double, 6, DIM_GRF> inertia_inv;
        for (int i = 0; i < NUM_LEG; ++i)
        {
            inertia_inv.block<3, 3>(0, i * 3).setIdentity();
            // TODO: confirm this should be root_rot_mat instead of root_rot_mat
            inertia_inv.block<3, 3>(3, i * 3) = state.root_rot_mat_z.transpose() * Utils::skew(state.foot_pos_abs.block<3, 1>(0, i));
        }
        Eigen::Matrix<double, DIM_GRF, DIM_GRF> dense_hessian;
        dense_hessian.setIdentity();
        dense_hessian *= R;
        dense_hessian += inertia_inv.transpose() * Q * inertia_inv;
        hessian = dense_hessian.sparseView();
        // accidentally wrote this as -2* before. Huge problem
        gradient.block<3 * NUM_LEG, 1>(0, 0) = -inertia_inv.transpose() * Q * root_acc;

        // adjust bounds according to contact flag
        for (int i = 0; i < NUM_LEG; ++i)
        {
            double c_flag = state.contacts[i] ? 1.0 : 0.0;
            lowerBound(i) = c_flag * F_min;
            upperBound(i) = c_flag * F_max;
        }

        // instantiate the solver
        OsqpEigen::Solver solver;
        // settings
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(false);
        solver.data()->setNumberOfVariables(3 * NUM_LEG);
        solver.data()->setNumberOfConstraints(NUM_LEG + 4 * NUM_LEG);
        solver.data()->setLinearConstraintsMatrix(linearMatrix);
        solver.data()->setHessianMatrix(hessian);
        solver.data()->setGradient(gradient);
        solver.data()->setLowerBound(lowerBound);
        solver.data()->setUpperBound(upperBound);

        auto t1 = std::chrono::high_resolution_clock::now();
        solver.initSolver();
        auto t2 = std::chrono::high_resolution_clock::now();
        // solver.solve();
        solver.solveProblem();
        auto t3 = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> ms_double_1 = t2 - t1;
        std::chrono::duration<double, std::milli> ms_double_2 = t3 - t2;

        std::cout << "qp solver init time: " << ms_double_1.count() << "ms; solve time: " << ms_double_2.count() << "ms" << std::endl;

        Eigen::VectorXd QPSolution = solver.getSolution(); // 12x1
        for (int i = 0; i < NUM_LEG; ++i)
        {
            // the QP solves for world frame force
            // here we convert the force into robot frame
            foot_forces_grf.block<3, 1>(0, i) = state.root_rot_mat.transpose() * QPSolution.segment<3>(i * 3);
        }
    }
    else if (state.stance_leg_control_type == 1)
    { // 1: MPC
        ConvexMpc mpc_solver = ConvexMpc(state.q_weights, state.r_weights);
        mpc_solver.reset();

        // initialize the mpc state at the first time step
        // state.mpc_states.resize(13);
        state.mpc_states << state.root_euler[0], state.root_euler[1], state.root_euler[2],
            state.root_pos[0], state.root_pos[1], state.root_pos[2],
            state.root_ang_vel[0], state.root_ang_vel[1], state.root_ang_vel[2],
            state.root_lin_vel[0], state.root_lin_vel[1], state.root_lin_vel[2],
            -ROBOT_GRAVITY;

        // previously we use dt passed by outer thread. It turns out that this dt is not stable on hardware.
        // if the thread is slowed down, dt becomes large, then MPC will output very large force and torque value
        // which will cause over current. Here we use a new mpc_dt, this should be roughly close to the average dt
        // of thread 1
        double mpc_dt = 0.0025;

        // in simulation, use dt has no problem
        if (use_sim_time == "true")
        {
            mpc_dt = dt;
        }

        // initialize the desired mpc states trajectory
        state.root_lin_vel_d_world = state.root_rot_mat * state.root_lin_vel_d;
        // state.mpc_states_d.resize(13 * PLAN_HORIZON);
        for (int i = 0; i < PLAN_HORIZON; ++i)
        {
            state.mpc_states_d.segment(i * 13, 13)
                << state.root_euler_d[0],
                state.root_euler_d[1],
                state.root_euler[2] + state.root_ang_vel_d[2] * mpc_dt * (i + 1),
                state.root_pos[0] + state.root_lin_vel_d_world[0] * mpc_dt * (i + 1),
                state.root_pos[1] + state.root_lin_vel_d_world[1] * mpc_dt * (i + 1),
                state.root_pos_d[2],
                state.root_ang_vel_d[0],
                state.root_ang_vel_d[1],
                state.root_ang_vel_d[2],
                state.root_lin_vel_d_world[0],
                state.root_lin_vel_d_world[1],
                0,
                -ROBOT_GRAVITY;
        }

        // a single A_c is computed for the entire reference trajectory
        auto t1 = std::chrono::high_resolution_clock::now();
        mpc_solver.calculate_A_mat_c(state.root_euler);

        // for each point in the reference trajectory, an approximate B_c matrix is computed using desired values of euler angles and feet positions
        // from the reference trajectory and foot placement controller
        // state.foot_pos_abs_mpc = state.foot_pos_abs;
        auto t2 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < PLAN_HORIZON; i++)
        {
            // calculate current B_c matrix
            mpc_solver.calculate_B_mat_c(state.robot_mass,
                                         state.go1_trunk_inertia,
                                         state.root_rot_mat,
                                         state.foot_pos_abs);
            // state.foot_pos_abs_mpc.block<3, 1>(0, 0) = state.foot_pos_abs_mpc.block<3, 1>(0, 0) - state.root_lin_vel_d * mpc_dt;
            // state.foot_pos_abs_mpc.block<3, 1>(0, 1) = state.foot_pos_abs_mpc.block<3, 1>(0, 1) - state.root_lin_vel_d * mpc_dt;
            // state.foot_pos_abs_mpc.block<3, 1>(0, 2) = state.foot_pos_abs_mpc.block<3, 1>(0, 2) - state.root_lin_vel_d * mpc_dt;
            // state.foot_pos_abs_mpc.block<3, 1>(0, 3) = state.foot_pos_abs_mpc.block<3, 1>(0, 3) - state.root_lin_vel_d * mpc_dt;

            // state space discretization, calculate A_d and current B_d
            mpc_solver.state_space_discretization(mpc_dt);

            // store current B_d matrix
            mpc_solver.B_mat_d_list.block<13, 12>(i * 13, 0) = mpc_solver.B_mat_d;
        }

        // calculate QP matrices
        auto t3 = std::chrono::high_resolution_clock::now();
        mpc_solver.calculate_qp_mats(state);

        // solve
        auto t4 = std::chrono::high_resolution_clock::now();
        if (!solver.isInitialized())
        {
            solver.settings()->setVerbosity(false);
            solver.settings()->setWarmStart(true);
            solver.data()->setNumberOfVariables(NUM_DOF * PLAN_HORIZON);
            solver.data()->setNumberOfConstraints(MPC_CONSTRAINT_DIM * PLAN_HORIZON);
            solver.data()->setLinearConstraintsMatrix(mpc_solver.linear_constraints);
            solver.data()->setHessianMatrix(mpc_solver.hessian);
            solver.data()->setGradient(mpc_solver.gradient);
            solver.data()->setLowerBound(mpc_solver.lb);
            solver.data()->setUpperBound(mpc_solver.ub);
            solver.initSolver();
        }
        else
        {
            solver.updateHessianMatrix(mpc_solver.hessian);
            solver.updateGradient(mpc_solver.gradient);
            solver.updateLowerBound(mpc_solver.lb);
            solver.updateUpperBound(mpc_solver.ub);
        }
        auto t5 = std::chrono::high_resolution_clock::now();
        // solver.solve();
        solver.solveProblem();
        auto t6 = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> ms_double_1 = t2 - t1;
        std::chrono::duration<double, std::milli> ms_double_2 = t3 - t2;
        std::chrono::duration<double, std::milli> ms_double_3 = t4 - t3;
        std::chrono::duration<double, std::milli> ms_double_4 = t5 - t4;
        std::chrono::duration<double, std::milli> ms_double_5 = t6 - t5;

        //        std::cout << "mpc cal A_mat_c: " << ms_double_1.count() << "ms" << std::endl;
        //        std::cout << "mpc cal B_mat_d_list: " << ms_double_2.count() << "ms" << std::endl;
        //        std::cout << "mpc cal qp mats: " << ms_double_3.count() << "ms" << std::endl;
        //        std::cout << "mpc init time: " << ms_double_4.count() << "ms" << std::endl;
        //        std::cout << "mpc solve time: " << ms_double_5.count() << "ms" << std::endl << std::endl;

        Eigen::VectorXd solution = solver.getSolution();
        // std::cout << solution.transpose() << std::endl;

        for (int i = 0; i < NUM_LEG; ++i)
        {
            if (!isnan(solution.segment<3>(i * 3).norm()))
                foot_forces_grf.block<3, 1>(0, i) = state.root_rot_mat.transpose() * solution.segment<3>(i * 3);
        }
    }
    return foot_forces_grf;
}

Eigen::Vector3d Go1RobotControl::compute_walking_surface(Go1CtrlStates &state)
{
    Eigen::Matrix<double, NUM_LEG, 3> W;
    Eigen::VectorXd foot_pos_z;
    Eigen::Vector3d a;
    Eigen::Vector3d surf_coef;

    W.block<NUM_LEG, 1>(0, 0).setOnes();
    W.block<NUM_LEG, 2>(0, 1) = state.foot_pos_recent_contact.block<2, NUM_LEG>(0, 0).transpose();

    foot_pos_z.resize(NUM_LEG);
    foot_pos_z = state.foot_pos_recent_contact.block<1, NUM_LEG>(2, 0).transpose();

    a = Utils::pseudo_inverse(W.transpose() * W) * W.transpose() * foot_pos_z;
    // surface: a1 * x + a2 * y - z + a0 = 0, coefficient vector: [a1, a2, -1]
    surf_coef << a[1], a[2], -1;
    return surf_coef;
}
