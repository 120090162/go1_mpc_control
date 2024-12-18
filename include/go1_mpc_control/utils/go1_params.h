#ifndef GO1_PARAMS_H
#define GO1_PARAMS_H

// control time related
// #define CTRL_FREQUENCY 2.5  // ms
// GRF -> ground force (Fx,Fy,Fz)x4
#define GRF_UPDATE_FREQUENCY 2.5        // ms
#define MAIN_UPDATE_FREQUENCY 2.5       // ms
#define HARDWARE_FEEDBACK_FREQUENCY 2.0 // ms
#define PI 3.1415926
#define ROBOT_GRAVITY 9.81

// constant define
// joy stick command interprate
#define JOY_CMD_BODY_HEIGHT_MAX 0.32 // m
#define JOY_CMD_BODY_HEIGHT_MIN 0.1  // m
#define JOY_CMD_BODY_HEIGHT_VEL 0.04 // m/s
#define JOY_CMD_VELX_MAX 0.6         // m/s
#define JOY_CMD_VELY_MAX 0.3         // m/s
#define JOY_CMD_YAW_MAX 0.8          // rad
#define JOY_CMD_PITCH_MAX 0.4        // rad
#define JOY_CMD_ROLL_MAX 0.4         // rad

// mpc
#define PLAN_HORIZON 10
#define MPC_STATE_DIM 13
#define MPC_CONSTRAINT_DIM 20

// robot constant
#define NUM_LEG 4
#define NUM_DOF_PER_LEG 3
#define DIM_GRF 12
#define NUM_DOF 12

// #ifdef USE_GO1
#define LEG_OFFSET_X 0.1881
#define LEG_OFFSET_Y 0.04675
#define MOTOR_OFFSET 0.08

#define UPPER_LEG_LENGTH 0.213
#define LOWER_LEG_LENGTH 0.213
// #endif

// #ifdef USE_A1
// #define LEG_OFFSET_X 0.1805
// #define LEG_OFFSET_Y 0.047
// #define MOTOR_OFFSET 0.0838

// #define UPPER_LEG_LENGTH 0.2
// #define LOWER_LEG_LENGTH 0.2
// #endif

#define FOOT_FORCE_LOW 30.0
#define FOOT_FORCE_HIGH 80.0

#define FOOT_SWING_CLEARANCE1 0.0f
#define FOOT_SWING_CLEARANCE2 0.4f

#define FOOT_DELTA_X_LIMIT 0.1
#define FOOT_DELTA_Y_LIMIT 0.1

#define ERROR_CURVE_ALREADY_SET 184
#define ERROR_CURVE_NOT_SET 185

// state estimator parameters
#define STATE_SIZE 18
#define MEAS_SIZE 28
#define PROCESS_NOISE_PIMU 0.01
#define PROCESS_NOISE_VIMU 0.01
#define PROCESS_NOISE_PFOOT 0.01
#define SENSOR_NOISE_PIMU_REL_FOOT 0.001
#define SENSOR_NOISE_VIMU_REL_FOOT 0.1
#define SENSOR_NOISE_ZFOOT 0.001
#define LARGEVARIANCE 1e3

#endif // GO1_PARAMS_H
