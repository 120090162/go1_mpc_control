#ifndef GO1_BASICEKF_H
#define GO1_BASICEKF_H

#include "go1_params.h"
#include "go1_ctrl_states.h"
#include "utils.h"

// implement a basic error state KF to estimate robot pose
// assume orientation is known from a IMU (state.root_rot_mat)
class Go1BasicEKF
{
public:
    // assume flat ground
    Go1BasicEKF();
    // Go1BasicEKF(bool assume_flat_ground_);
    void init_state(Go1CtrlStates &state);
    void update_estimation(Go1CtrlStates &state, double dt);
    bool is_inited() { return filter_initialized; }

private:
    bool filter_initialized = false;
    bool assume_flat_ground = false;

    // state
    // 0 1 2 pos 3 4 5 vel 6 7 8 foot pos FL 9 10 11 foot pos FR 12 13 14 foot pos RL 15 16 17 foot pos RR
    Eigen::Matrix<double, STATE_SIZE, 1> x;             // estimation state, position(3)+velocity(3)+feet position(3x4)
    Eigen::Matrix<double, STATE_SIZE, 1> xbar;          // estimation state after process update
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P;    // estimation state covariance
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Pbar; // estimation state covariance after process update
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> A;    // The transtion matrix of estimator
    Eigen::Matrix<double, STATE_SIZE, 3> B;             // The input matrix
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q;    // estimation state transition noise

    // observation
    // 0 1 2   FL pos residual
    // 3 4 5   FR pos residual
    // 6 7 8   RL pos residual
    // 9 10 11 RR pos residual
    // 12 13 14 vel residual from FL
    // 15 16 17 vel residual from FR
    // 18 19 20 vel residual from RL
    // 21 22 23 vel residual from RR
    // 24 25 26 27 foot height
    Eigen::Matrix<double, MEAS_SIZE, 1> y;           // observation, the measurement value of output y
    Eigen::Matrix<double, MEAS_SIZE, 1> yhat;        // estimated observation, the prediction of output y
    Eigen::Matrix<double, MEAS_SIZE, 1> Serror_y;    // S^-1*error_y
    Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> C;  // estimation state observation
    Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> SC; // S^-1*C
    Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> R;   // estimation state observation noise
    // helper matrices
    Eigen::Matrix<double, 3, 3> eye3;                                             // 3x3 identity
    Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> S;                                // Innovation (or pre-fit residual) covariance
    Eigen::FullPivHouseholderQR<Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE>> Sqr; // QR decomposition of S
    Eigen::Matrix<double, STATE_SIZE, MEAS_SIZE> K;                               // kalman gain

    // variables to process whether foot is in contact, 0 means not in contact, 1 means in contact
    double estimated_contacts[4];
};

#endif // GO1_BASICEKF_H
