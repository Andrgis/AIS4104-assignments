#include <iostream>
#include <Eigen/Dense>
#include "math/math.h"




int main()
{
    Eigen::Vector3d e = Eigen::Vector3d{60.0,45.0,30.0};
    Eigen::Vector3d ev = Eigen::Vector3d{60.0,45.0,30.0};
    Eigen::Vector3d s = Eigen::Vector3d{1,-1,2}.normalized();
    Eigen::Vector3d q = Eigen::Vector3d{4,1,2};
    Eigen::Vector3d w = Eigen::Vector3d{1,1,0}.normalized();
    Eigen::Matrix3d R = math::rotation_matrix_from_euler_zyx(e);
    Eigen::Vector3d ea = math::euler_zyx_from_rotation(R);
    /* Eigen::Matrix4d tf;
        tf << 0,-1,0,2,
            1,0,0,1,
            0,0,-1,3,
            0,0,0,1; */

    Eigen::Matrix3d R_af;
      R_af << 1,0,0,0,0,1,0,-1,0;
    Eigen::Matrix3d R_hf = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_af {-0.25,0,0};
    Eigen::Vector3d p_hf {-0.1,0,0};
    Eigen::VectorXd F_a(6);
    Eigen::VectorXd F_h(6);
    F_a << 0,0,0,0,0,1;
    F_h << 0,0,0,0,-5,0;

    Eigen::Matrix3d Rotmat {math::matrix_exponential(w,90)};

    const Eigen::Matrix4d T_hf {math::transformation_matrix(R_hf,p_hf)};
    const Eigen::Matrix4d T_af {math::transformation_matrix(R_af,p_af)};

    Eigen::VectorXd F_f(6);
    F_f = math::wrench_f(F_a,F_h,T_af, T_hf);

    std::cout << " E: " << e.transpose() << std::endl;
    std::cout << " Ea: " << ea.transpose() << std::endl;

    std::cout << " Twist, V: " << math::twist(e,ev).transpose() << std::endl;

    std::cout << " Screw axis, S: " << math::screw_axis(q,s,1.5).transpose() << std::endl;

    //std::cout << " Adjoint Matrix, [Ad_T]: " << std::endl << math::adjoint_matrix(tf) <<std::endl;

    std::cout << " cot(pi/6): " << math::cot(EIGEN_PI/6) << std::endl;

    math::wrench_in_s_and_w();

    std::cout << " F_f: " << F_f.transpose() << std::endl;

    std::cout << " R: "<< std::endl << R << std::endl;

    // Call the matrix_logarithm function
    std::pair<Eigen::Vector3d, double> result = math::matrix_logarithm(Rotmat);

    // Extract the rotation vector and angle
    Eigen::Vector3d rotation_vector = result.first.normalized();
    const double rotation_angle = result.second;

    // Output the results
    std::cout << "Rotation vector: " << rotation_vector.transpose() << std::endl;
    std::cout << "Rotation angle: " << rotation_angle << std::endl;

    Eigen::Vector3d s1 = Eigen::Vector3d{0,0,1}.normalized();
    Eigen::Vector3d v {3.37,-3.37,0};

    Eigen::Matrix4d T = math::matrix_exponential(s1, v, 60);

    std::cout << " T: "<< std::endl << T << std::endl;


    return 0;
}

