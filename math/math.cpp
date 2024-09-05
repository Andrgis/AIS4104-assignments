#include <iostream>
#include <Eigen/Dense>
#include "math/math.h"



bool math::floatEquals(double a, double b) {
    return std::abs(a - b) < 1e-6;
}

double math::deg_to_rad(double degrees)
{
    return degrees * M_PI / 180;
}

Eigen::Matrix3d math::rotate_x(double degrees)
{
    double radians = deg_to_rad(degrees);
    Eigen::Matrix3d matrix;
    matrix <<
        1, 0, 0,
        0, std::cos(radians), -std::sin(radians),
        0, std::sin(radians), std::cos(radians);
    return matrix;
}

Eigen::Matrix3d math::rotate_y(double degrees) {
    double radians = deg_to_rad(degrees);
    Eigen::Matrix3d matrix;
    matrix <<
         std::cos(radians), 0, std::sin(radians),
                         0, 1,                  0,
        -std::sin(radians), 0,  std::cos(radians);
    return matrix;
}

Eigen::Matrix3d math::rotate_z(double degrees) {
    double radians = deg_to_rad(degrees);
    Eigen::Matrix3d matrix;
    matrix <<
        std::cos(radians), -std::sin(radians), 0,
        std::sin(radians),  std::cos(radians), 0,
                        0,                  0, 1;
    return matrix;
}

Eigen::Matrix3d math::skew_symmetric(Eigen::Vector3d v) {
    Eigen::Matrix3d skewed_vector;
    skewed_vector <<
             0, -v.z(),  v.y(),
         v.z(),      0, -v.x(),
        -v.y(),  v.x(),      0;
    return skewed_vector;
}

Eigen::Matrix3d math::rotation_matrix_from_frame_axes(const Eigen::Vector3d &x,
                                                const Eigen::Vector3d &y,
                                                const Eigen::Vector3d &z)
{
Eigen::Matrix3d matrix;
matrix <<
    x(0), y(0), z(0),
    x(1), y(1), z(1),
    x(2), y(2), z(2);
return matrix;
}

Eigen::Matrix3d math::rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double degrees)
{
    Eigen::Matrix3d matrix {};
    double radians {deg_to_rad(degrees)};

    Eigen::Matrix3d skewed_vector {skew_symmetric(axis)};
    Eigen::Matrix3d I {Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d skewed_vector_squared {skewed_vector * skewed_vector};

    // Bruker Rodrigues' formula for rot(omega,thetta)
    matrix = I + std::sin(radians) * skewed_vector + (1-std::cos(radians))*skewed_vector_squared;

    return matrix;
}

Eigen::Matrix3d math::rotation_matrix_from_euler_zyx(const Eigen::Vector3d &e)
{
    Eigen::Matrix3d I {Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d R_z {};
    Eigen::Matrix3d R_y {};
    Eigen::Matrix3d R_x {};
    Eigen::Matrix3d R {};

    R_z = rotate_z(e(0));
    R_y = rotate_y(e(1));
    R_x = rotate_x(e(2));

    R = I * R_z * R_y * R_x;

    return R;
}

Eigen::Matrix4d math::transformation_matrix(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    Eigen::Matrix4d matrix;
    // implement the necessary equations and functionality.
    matrix <<
        r(0,0), r(0,1), r(0,2), p(0),
        r(1,0), r(1,1), r(1,2), p(1),
        r(2,0), r(2,1), r(2,2), p(2),
                    0,              0,            0,          1;
    return matrix;
}

Eigen::Vector3d math::euler_zyx_from_rotation(const Eigen::Matrix3d &r)
{
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;

    if(floatEquals(r(2, 0), -1.0))
    {
        b = EIGEN_PI / 2.0;
        a = 0.0;
        c = std::atan2(r(0, 1), r(1, 1));
    }
    else if(floatEquals(r(2, 0), 1.0))
    {
        b = -(EIGEN_PI / 2.0);
        a = 0.0;
        c = -std::atan2(r(0, 1), r(1, 1));
    }
    else
    {
        b = std::atan2(-r(2, 0), std::sqrt(r(0, 0) * r(0, 0) + r(1, 0) * r(1, 0)));
        a = std::atan2(r(1, 0), r(0, 0));
        c = std::atan2(r(2, 1), r(2, 2));
    }
    return Eigen::Vector3d{a, b, c};
}

Eigen::VectorXd math::twist(const Eigen::Vector3d &w, const Eigen::Vector3d &v) {
    return Eigen::VectorXd {w(0), w(1), w(2), v(0), v(1), v(2)};
}

