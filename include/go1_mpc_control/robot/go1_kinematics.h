#ifndef GO1_KINEMATICS_H
#define GO1_KINEMATICS_H

#include <eigen3/Eigen/Dense>

class Go1Kinematics
{
public:
    Go1Kinematics() = default;
    ~Go1Kinematics() = default;

    // const int RHO_FIX_SIZE = 5;
    // rho fix are body offset x& y, thigh motor offset, upper leg length, lower leg length
    // functions with eigen interface
    // forward kinematics 3x1
    Eigen::Vector3d fk(Eigen::Vector3d q, Eigen::VectorXd rho_fix);
    // jacobian   3x3
    Eigen::Matrix3d jac(Eigen::Vector3d q, Eigen::VectorXd rho_fix);

private:
    // functions with basic C++ interface
    void autoFunc_fk_derive(const double in1[3], const double in2[5], double p_bf[3]);
    void autoFunc_d_fk_dq(const double in1[3], const double in2[5], double jacobian[9]);
};

#endif // GO1_KINEMATICS_H
