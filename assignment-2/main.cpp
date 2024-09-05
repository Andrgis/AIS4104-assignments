#include <iostream>
#include <Eigen/Dense>
#include "math/math.h"

constexpr double c_rad_to_deg = 57.2957795;
constexpr double c_deg_to_rad = 0.0174532925;

bool floatEquals(double a, double b) {
    return std::abs(a - b) < 1e-6;
}


int main()
{
    Eigen::Vector3d e = Eigen::Vector3d{60.0,45.0,30.0};
    Eigen::Matrix3d R = math::rotation_matrix_from_euler_zyx(e);
    Eigen::Vector3d ea = math::euler_zyx_from_rotation(R);
    std::cout << " E: " << e.transpose() << std::endl;
    std::cout << " Ea: " << ea.transpose() << std::endl;




    return 0;
}

