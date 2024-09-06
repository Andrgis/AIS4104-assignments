#include <iostream>
#include <Eigen/Dense>
#include "math/math.h"




int main()
{
    Eigen::Vector3d e = Eigen::Vector3d{60.0,45.0,30.0};
    Eigen::Vector3d ev = Eigen::Vector3d{60.0,45.0,30.0};
    Eigen::Vector3d s = Eigen::Vector3d{1,-1,2}.normalized();
    Eigen::Vector3d q = Eigen::Vector3d{4,1,2};
    Eigen::Matrix3d R = math::rotation_matrix_from_euler_zyx(e);
    Eigen::Vector3d ea = math::euler_zyx_from_rotation(R);
    Eigen::Matrix4d tf;
        tf << 0,-1,0,2,
            1,0,0,1,
            0,0,-1,3,
            0,0,0,1;


    std::cout << " E: " << e.transpose() << std::endl;
    std::cout << " Ea: " << ea.transpose() << std::endl;

    std::cout << " Twist, V: " << math::twist(e,ev).transpose() << std::endl;

    std::cout << " Screw axis, S: " << math::screw_axis(q,s,1.5).transpose() << std::endl;

    std::cout << " Adjoint Matrix, [Ad_T]: " << std::endl << math::adjoint_matrix(tf) <<std::endl;

    std::cout << " cot(pi/6): " << math::cot(EIGEN_PI/6) << std::endl;

    return 0;
}

