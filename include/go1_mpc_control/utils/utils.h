#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "go1_params.h"

class Utils
{
public:
    // compare to Eigen's default eulerAngles
    // this function returns yaw angle within -pi to pi
    static Eigen::Vector3d quat_to_euler(Eigen::Quaterniond quat);
    // skew symmetric matrix
    static Eigen::Matrix3d skew(Eigen::Vector3d vec);
    static Eigen::Matrix3d pseudo_inverse(const Eigen::Matrix3d &mat);
    static double cal_dihedral_angle(Eigen::Vector3d surf_coef_1, Eigen::Vector3d surf_coef_2);
};

class BezierUtils
{
    // TODO: allow degree change? may not be necessary, we can stick to degree 4
public:
    BezierUtils()
    {
        curve_constructed = false;
        bezier_degree = 4;
    }
    // set of functions create bezier curves, get points, reset
    Eigen::Vector3d get_foot_pos_curve(float t,
                                       Eigen::Vector3d foot_pos_start,
                                       Eigen::Vector3d foot_pos_final,
                                       double terrain_pitch_angle); // 地形陡峭角度

    bool reset_foot_pos_curve() { curve_constructed = false; } // return true

private:
    double bezier_curve(double t, const std::vector<double> &P);

    bool curve_constructed;
    float bezier_degree;
};

#endif // UTILS_H
