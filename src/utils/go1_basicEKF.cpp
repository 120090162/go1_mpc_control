#include "utils/go1_basicEKF.h"

Go1BasicEKF::Go1BasicEKF()
{
    // constructor
    eye3.setIdentity();
    // C is fixed
    C.setZero();
    for (int i = 0; i < NUM_LEG; ++i)
    {
        C.block<3, 3>(i * 3, 0) = -eye3;              //-pos
        C.block<3, 3>(i * 3, 6 + i * 3) = eye3;       // foot pos
        C.block<3, 3>(NUM_LEG * 3 + i * 3, 3) = eye3; // vel
        C(NUM_LEG * 6 + i, 6 + i * 3 + 2) = 1;        // height z of foot
    }

    // Q R are fixed
    Q.setIdentity();
    Q.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * eye3; // position transition
    Q.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * eye3; // velocity transition
    for (int i = 0; i < NUM_LEG; ++i)
    {
        Q.block<3, 3>(6 + i * 3, 6 + i * 3) = PROCESS_NOISE_PFOOT * eye3; // foot position transition
    }

    R.setIdentity();
    for (int i = 0; i < NUM_LEG; ++i)
    {
        R.block<3, 3>(i * 3, i * 3) = SENSOR_NOISE_PIMU_REL_FOOT * eye3;                             // fk estimation 前向运动学估计
        R.block<3, 3>(NUM_LEG * 3 + i * 3, NUM_LEG * 3 + i * 3) = SENSOR_NOISE_VIMU_REL_FOOT * eye3; // vel estimation
        R(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = SENSOR_NOISE_ZFOOT;                                    // height z estimation
    }

    // set A to identity
    A.setIdentity();

    // set B to zero
    B.setZero();

    assume_flat_ground = true;
}

// Go1BasicEKF::Go1BasicEKF(bool assume_flat_ground_) : Go1BasicEKF()
// {
//     // constructor
//     assume_flat_ground = assume_flat_ground_;
//     // change R according to this flag, if we do not assume the robot moves on flat ground,
//     // then we cannot infer height z using this way
//     if (assume_flat_ground == false)
//     {
//         for (int i = 0; i < NUM_LEG; ++i)
//         {
//             R(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = 1e5; // height z estimation not reliable
//         }
//     }
// }

void Go1BasicEKF::init_state(Go1CtrlStates &state)
{

    filter_initialized = true;
    // initialize P
    P.setIdentity();
    P = P * 3; // set initial variance

    // set initial estimator value of x
    x.setZero();
    x.segment<3>(0) = Eigen::Vector3d(0, 0, 0.09); // initial body position
    for (int i = 0; i < NUM_LEG; ++i)
    {
        Eigen::Vector3d fk_pos = state.foot_pos_rel.block<3, 1>(0, i);
        x.segment<3>(6 + i * 3) = state.root_rot_mat * fk_pos + x.segment<3>(0); // initial foot position
    }
}

void Go1BasicEKF::update_estimation(Go1CtrlStates &state, double dt)
{
    // update A B using latest dt
    A.block<3, 3>(0, 3) = dt * eye3;
    B.block<3, 3>(3, 0) = dt * eye3;

    // control input u is Ra + ag
    Eigen::Vector3d u = state.root_rot_mat * state.imu_acc + Eigen::Vector3d(0, 0, -ROBOT_GRAVITY);

    // contact estimation, do something very simple first
    if (state.movement_mode == 0)
    { // stand
        for (int i = 0; i < NUM_LEG; ++i)
            estimated_contacts[i] = 1.0;
    }
    else
    { // walk
        /* probability of the contact state
         * when the foot force surpass 80 N, the estimated state is contact */
        for (int i = 0; i < NUM_LEG; ++i)
        {
            // TODO: max_force and min_force should be set as parameters
            estimated_contacts[i] = std::min(std::max((state.foot_force(i) + 20.0) / (100.0 - 0.0), 0.0), 1.0);
            // estimated_contacts[i] = 1.0 / (1.0 + std::exp(-(state.foot_force(i) - 100)));
        }
    }
    // update Q
    Q.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * dt / 20.0 * eye3;                 // IMU P
    Q.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * dt * ROBOT_GRAVITY / 20.0 * eye3; // IMU V
    // update Q R for legs not in contact
    for (int i = 0; i < NUM_LEG; ++i)
    {
        Q.block<3, 3>(6 + i * 3, 6 + i * 3) =
            (1 + (1 - estimated_contacts[i]) * LARGEVARIANCE) * dt * PROCESS_NOISE_PFOOT * eye3; // foot position transition
        // for estimated_contacts[i] == 1, Q = 0.002
        // for estimated_contacts[i] == 0, Q = 1001*Q

        R.block<3, 3>(i * 3, i * 3) = (1 + (1 - estimated_contacts[i]) * LARGEVARIANCE) * SENSOR_NOISE_PIMU_REL_FOOT * eye3;                             // fk estimation
        R.block<3, 3>(NUM_LEG * 3 + i * 3, NUM_LEG * 3 + i * 3) = (1 + (1 - estimated_contacts[i]) * LARGEVARIANCE) * SENSOR_NOISE_VIMU_REL_FOOT * eye3; // vel estimation
        if (assume_flat_ground)
        {
            R(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = (1 + (1 - estimated_contacts[i]) * LARGEVARIANCE) * SENSOR_NOISE_ZFOOT; // height z estimation
        }
    }

    // process update
    /* A and B are translate matrixs */
    xbar = A * x + B * u;
    /* Pbar means state predict covariance martrix */
    Pbar = A * P * A.transpose() + Q;

    // measurement construction
    yhat = C * xbar;
    // leg_v = (-J_rf*av-skew(omega)*p_rf);
    // r((i-1)*3+1:(i-1)*3+3) = body_v - R_er*leg_v;
    // actual measurement
    /* update the observation variable in kalman filter */
    for (int i = 0; i < NUM_LEG; ++i)
    {
        Eigen::Vector3d fk_pos = state.foot_pos_rel.block<3, 1>(0, i);                                           // in base frame
        y.block<3, 1>(i * 3, 0) = state.root_rot_mat * fk_pos;                                                   // fk estimation
        Eigen::Vector3d leg_v = -state.foot_vel_rel.block<3, 1>(0, i) - Utils::skew(state.imu_ang_vel) * fk_pos; // -v_rf - omega x p_rf
        y.block<3, 1>(NUM_LEG * 3 + i * 3, 0) =
            (1.0 - estimated_contacts[i]) * x.segment<3>(3) + estimated_contacts[i] * state.root_rot_mat * leg_v; // vel estimation

        y(NUM_LEG * 6 + i) =
            (1.0 - estimated_contacts[i]) * (x(2) + fk_pos(2)) + estimated_contacts[i] * 0; // height z estimation
    }
    /* update kalman filter parameters */
    S = C * Pbar * C.transpose() + R;
    S = 0.5 * (S + S.transpose()); // make sure S is symmetric
    Sqr = S.fullPivHouseholderQr();
    Serror_y = Sqr.solve(y - yhat); // QR & lu decomposition

    x = xbar + Pbar * C.transpose() * Serror_y;

    SC = Sqr.solve(C);
    P = Pbar - Pbar * C.transpose() * SC * Pbar; // (I-KC)Pbar
    P = 0.5 * (P + P.transpose());

    // reduce position drift
    if (P.block<2, 2>(0, 0).determinant() > 1e-6)
    {
        P.block<2, 16>(0, 2).setZero();
        P.block<16, 2>(2, 0).setZero();
        P.block<2, 2>(0, 0) /= 10.0;
    }

    // final step
    // put estimated values back to Go1CtrlStates& state
    for (int i = 0; i < NUM_LEG; ++i)
    {
        if (estimated_contacts[i] < 0.5)
        {
            state.estimated_contacts[i] = false;
        }
        else
        {
            state.estimated_contacts[i] = true;
        }
    }
    // debug
    // state.estimated_root_pos = x.segment<3>(0);
    // state.estimated_root_vel = x.segment<3>(3);

    state.root_pos = x.segment<3>(0);
    state.root_lin_vel = x.segment<3>(3);
}