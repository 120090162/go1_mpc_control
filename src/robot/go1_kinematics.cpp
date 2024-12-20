#include "robot/go1_kinematics.h"

Eigen::Vector3d Go1Kinematics::fk(Eigen::Vector3d q, Eigen::VectorXd rho_fix)
{
    Eigen::Vector3d out;
    autoFunc_fk_derive(q.data(), rho_fix.data(), out.data());
    return out;
}

Eigen::Matrix3d Go1Kinematics::jac(Eigen::Vector3d q, Eigen::VectorXd rho_fix)
{
    Eigen::Matrix3d mtx;
    autoFunc_d_fk_dq(q.data(), rho_fix.data(), mtx.data());
    return mtx;
}

// functions generated by matlab
void Go1Kinematics::autoFunc_fk_derive(const double in1[3], const double in2[5], double p_bf[3])
{
    // double p_bf_tmp;
    // double t2;
    // double t3;
    // double t4;
    // double t5;
    // double t6;
    // double t7;
    // double t8;
    // double t9;

    double l1 = in2[2];
    double l2 = -in2[3];
    double l3 = -in2[4];

    double s1 = std::sin(in1[0]);
    double s2 = std::sin(in1[1]);
    double s3 = std::sin(in1[2]);

    double c1 = std::cos(in1[0]);
    double c2 = std::cos(in1[1]);
    double c3 = std::cos(in1[2]);

    double c23 = c2 * c3 - s2 * s3;
    double s23 = s2 * c3 + c2 * s3;

    // t2 = std::cos(in1[0]);
    // t3 = std::cos(in1[1]);
    // t4 = std::cos(in1[2]);

    // t5 = std::sin(in1[0]);
    // t6 = std::sin(in1[1]);
    // t7 = std::sin(in1[2]);

    // t8 = in1[1] + in1[2];
    // t9 = std::sin(t8);

    // p_bf[0] = in2[0] - in2[4] * t9 - t6 * in2[3];
    p_bf[0] = in2[0] + l2 * s2 + l3 * s23;
    // p_bf[1] = in2[1] + in2[2] * t2 + t3 * t5 * in2[3] + in2[4] * t3 * t4 * t5 - in2[4] * t5 * t6 * t7;
    p_bf[1] = in2[1] + l1 * c1 - l2 * c2 * s1 - l3 * s1 * c23;
    // p_bf_tmp = in2[4] * t2;
    // p_bf[2] = in2[2] * t5 - t2 * t3 * in2[3] - p_bf_tmp * t3 * t4 + p_bf_tmp * t6 * t7;
    p_bf[2] = l1 * s1 + l2 * c1 * c2 + l3 * c1 * c23;
}

void Go1Kinematics::autoFunc_d_fk_dq(const double in1[3], const double in2[5], double jacobian[9])
{
    double l1 = in2[2];
    double l2 = -in2[3];
    double l3 = -in2[4];

    double s1 = std::sin(in1[0]);
    double s2 = std::sin(in1[1]);
    double s3 = std::sin(in1[2]);

    double c1 = std::cos(in1[0]);
    double c2 = std::cos(in1[1]);
    double c3 = std::cos(in1[2]);

    double c23 = c2 * c3 - s2 * s3;
    double s23 = s2 * c3 + c2 * s3;

    jacobian[0] = 0.0;
    jacobian[1] = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1;
    jacobian[2] = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1;
    jacobian[3] = l3 * c23 + l2 * c2;
    jacobian[4] = l3 * s1 * s23 + l2 * s1 * s2;
    jacobian[5] = -l3 * c1 * s23 - l2 * c1 * s2;
    jacobian[6] = l3 * c23;
    jacobian[7] = l3 * s1 * s23;
    jacobian[8] = -l3 * c1 * s23;
}