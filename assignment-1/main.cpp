#include <iostream>

#include <Eigen/Dense>

double deg_to_rad(double degrees)
{
    return degrees * M_PI / 180;
}

Eigen::Matrix3d rotate_x(double degrees)
{
    double radians = deg_to_rad(degrees);
    Eigen::Matrix3d matrix;
    matrix <<
        1, 0, 0,
        0, std::cos(radians), -std::sin(radians),
        0, std::sin(radians), std::cos(radians);
    return matrix;
}

Eigen::Matrix3d rotate_y(double degrees) {
    double radians = deg_to_rad(degrees);
    Eigen::Matrix3d matrix;
    matrix <<
         std::cos(radians), 0, -std::sin(radians),
                         0, 1,                  0,
        -std::sin(radians), 0,  std::cos(radians);
    return matrix;
}

Eigen::Matrix3d rotate_z(double degrees) {
    double radians = deg_to_rad(degrees);
    Eigen::Matrix3d matrix;
    matrix <<
        std::cos(radians), -std::sin(radians), 0,
        std::sin(radians),  std::cos(radians), 0,
                        0,                  0, 1;
    return matrix;
}

Eigen::Matrix3d skew_symmetric(Eigen::Vector3d v) {
    Eigen::Matrix3d skewed_vector;
    skewed_vector <<
             0, -v.z(),  v.y(),
         v.z(),      0, -v.x(),
        -v.y(),  v.x(),      0;
    return skewed_vector;
}

void skew_symmetric_test()
{
    Eigen::Matrix3d skew_matrix = skew_symmetric(Eigen::Vector3d{0.5, 0.5, 0.707107});
    std::cout << "Skew-symmetric matrix: " << std::endl;
    std::cout << skew_matrix << std::endl;
    std::cout << "Skew-symmetric matrix transposition: " << std::endl;
    std::cout << -skew_matrix.transpose() << std::endl;
}

Eigen::Matrix3d rotation_matrix_from_frame_axes(const Eigen::Vector3d &x,
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

Eigen::Matrix3d rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double degrees)
{
    Eigen::Matrix3d matrix {};
    // implement the necessary equations and functionality.
    double radians {deg_to_rad(degrees)};

    Eigen::Matrix3d skewed_vector {skew_symmetric(axis)};
    Eigen::Matrix3d I {Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d skewed_vector_squared {skewed_vector * skewed_vector};

    matrix = I + std::sin(radians) * skewed_vector + (1-std::cos(radians))*skewed_vector_squared;

    return matrix;
}


void example(double constant)
{
    Eigen::Matrix3d identity;
    identity <<
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0;
    std::cout << "I: " << std::endl << identity << std::endl << std::endl;
    std::cout << constant <<"*I: " << std::endl << constant * identity << std::endl << std::endl;
}

void rotation_matrix_test()
{
Eigen::Matrix3d rot =
rotation_matrix_from_euler_zyx(Eigen::Vector3d{45.0, -45.0, 90.0});
Eigen::Matrix3d rot_aa =
rotation_matrix_from_axis_angle(Eigen::Vector3d{0.8164966, 0.0, 0.5773503}, 120.0);
Eigen::Matrix3d rot_fa =
rotation_matrix_from_frame_axes(Eigen::Vector3d{0.5, 0.5, 0.707107},
Eigen::Vector3d{-0.5, -0.5, 0.707107},
Eigen::Vector3d{0.707107, -0.707107, 0.0});
std::cout << "Rotation matrix from Euler: " << std::endl;
std::cout << rot << std::endl << std::endl;
std::cout << "Rotation matrix from axis-angle pair: " << std::endl;
std::cout << rot_aa << std::endl << std::endl;
std::cout << "Rotation matrix from frame axes: " << std::endl;
std::cout << rot_fa << std::endl << std::endl;
}


int main()
{
    /*example(2.0);
    Eigen::Matrix3d x_0 = rotate_x(0);
    Eigen::Matrix3d x_45 = rotate_x(45);
    std::cout << x_0 << std::endl;
    std::cout << x_45 << std::endl;

    Eigen::Vector3d v(3, 5, 7);
    Eigen::Matrix3d skew_v = skew_symmetric(v);
    std::cout << skew_v << std::endl;*/

    skew_symmetric_test(); // Test sucsesfull
    


    return 0;
}

