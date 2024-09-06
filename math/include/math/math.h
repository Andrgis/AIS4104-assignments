//
// Created by andre on 05.09.2024.
//

#ifndef MATH_H
#define MATH_H
#include <Eigen/Dense>

namespace math {
    bool floatEquals(double a, double b);
    double deg_to_rad(double degrees);
    Eigen::Matrix3d rotate_x(double degrees);
    Eigen::Matrix3d rotate_y(double degrees);
    Eigen::Matrix3d rotate_z(double degrees);
    Eigen::Matrix3d skew_symmetric(Eigen::Vector3d v);
    Eigen::Matrix3d rotation_matrix_from_frame_axes(const Eigen::Vector3d &x,const Eigen::Vector3d &y,const Eigen::Vector3d &z);
    Eigen::Matrix3d rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double degrees);
    Eigen::Matrix3d rotation_matrix_from_euler_zyx(const Eigen::Vector3d &e);
    Eigen::Matrix4d transformation_matrix(const Eigen::Matrix3d &r, const Eigen::Vector3d &p);
    Eigen::Vector3d euler_zyx_from_rotation(const Eigen::Matrix3d &r);
    Eigen::VectorXd twist(const Eigen::Vector3d &w, const Eigen::Vector3d &v);
    Eigen::VectorXd screw_axis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h);
    Eigen::MatrixXd adjoint_matrix(const Eigen::Matrix4d &tf);
    double cot(double x);
    void wrench_in_s_and_w();
    constexpr double c_rad_to_deg{57.2957795};
    constexpr double c_deg_to_rad{0.01745329251};
}

#endif //MATH_H
