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

Eigen::Matrix3d math::rotation_matrix_from_frame_axes(const Eigen::Vector3d &x, const Eigen::Vector3d &y, const Eigen::Vector3d &z)
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

Eigen::Matrix3d math::rotation_matrix_from_euler_yzx(const Eigen::Vector3d &e)
{
    const Eigen::Matrix3d I {Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d R_z {};
    Eigen::Matrix3d R_y {};
    Eigen::Matrix3d R_x {};
    Eigen::Matrix3d R {};

    R_y = rotate_y(e(0));
    R_z = rotate_z(e(1));
    R_x = rotate_x(e(2));

    R = I * R_y * R_z * R_x;

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
    Eigen::VectorXd twist(6);
    twist << w(0), w(1), w(2), v(0), v(1), v(2);
    return twist;
}

Eigen::VectorXd math::screw_axis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h) {
    Eigen::Vector3d w {s.x(), s.y(), s.z()};
    Eigen::Vector3d v {-s.cross(w) + h*s};
    Eigen::VectorXd S(6);
    S << w(0), w(1), w(2), v(0), v(1), v(2);
    return S;
}

Eigen::MatrixXd math::adjoint_matrix(const Eigen::Matrix4d &tf) {
    Eigen::Vector3d p {tf(0, 3), tf(1, 3), tf(2, 3)};
    Eigen::Matrix3d skew_p {skew_symmetric(p)};
    Eigen::Matrix3d R;
        R << tf(0,0), tf(0,1), tf(0,2),
             tf(1,0), tf(1,1), tf(1,2),
             tf(2,0), tf(2,1), tf(2,2);
    Eigen::Matrix3d pR {skew_p*R};

    Eigen::MatrixXd adjoint(6, 6);
    adjoint <<  R(0,0),  R(0,1),  R(0,2),  0,             0,             0,
                R(1,0),  R(1,1),  R(1,2),  0,             0,             0,
                R(2,0),  R(2,1),  R(2,2),  0,             0,             0,
                pR(0,0), pR(0,1), pR(0,2), R(0,0), R(0,1), R(0,2),
                pR(1,0), pR(1,1), pR(1,2), R(1,0), R(1,1), R(1,2),
                pR(2,0), pR(2,1), pR(2,2), R(2,0), R(2,1), R(2,2);

    return adjoint;
}

double math::cot(double x){
    return 1/tan(x);
}

void math::wrench_in_s_and_w(){
    const Eigen::Vector3d f_w {-30,0,0};
    const Eigen::Vector3d m_s {0,0,2};
    const Eigen::Vector3d e_ws {60,-60,0}; // Euler angles YZX in degrees.
    Eigen::Matrix3d R_ws {rotation_matrix_from_euler_yzx(e_ws)};

    Eigen::Vector3d m_w {R_ws*m_s};
    Eigen::Vector3d f_s {R_ws.transpose()*f_w};


    //std::cout << "Still in progress" << std::endl;
    std::cout << "f_w: " << f_w.transpose() << std::endl;
    std::cout << "m_w: " << m_w.transpose() << std::endl;
    std::cout << "f_s: " << f_s.transpose() << std::endl;
    std::cout << "m_s: " << m_s.transpose() << std::endl;
    std::cout << "R_ws: " << std::endl << R_ws << std::endl;
}

Eigen::VectorXd math::wrench_w(const Eigen::Vector3d &f_w, const Eigen::Vector3d &m_s, const Eigen::Vector3d &e_ws) {
    Eigen::Matrix3d R_ws {rotation_matrix_from_euler_yzx(e_ws)};
    Eigen::Vector3d m_w {R_ws*m_s};
    //Eigen::Vector3d f_s {R_ws.transpose()*f_w};

    Eigen::VectorXd F_w(6);
        F_w << m_w(0), m_w(1), m_w(2), f_w(0), f_w(1), f_w(2);

    return F_w;
}

Eigen::VectorXd math::wrench_s(const Eigen::Vector3d &f_w, const Eigen::Vector3d &m_s, const Eigen::Vector3d &e_ws) {
    Eigen::Matrix3d R_ws {rotation_matrix_from_euler_yzx(e_ws)};
    //Eigen::Vector3d m_w {R_ws*m_s};
    Eigen::Vector3d f_s {R_ws.transpose()*f_w};

    Eigen::VectorXd F_s(6);
    F_s << m_s(0), m_s(1), m_s(2), f_s(0), f_s(1), f_s(2);

    return F_s;
}

Eigen::VectorXd math::wrench_a_to_b(const Eigen::VectorXd &F_a, const Eigen::Matrix4d &tf) {
    Eigen::MatrixXd Ad_T(6,6);
    Ad_T = {math::adjoint_matrix(tf)};

    return Ad_T.transpose()*F_a;
}

// Task 2b
Eigen::VectorXd math::wrench_f(const Eigen::VectorXd &F_a, const Eigen::VectorXd &F_b, const Eigen::MatrixXd &tf_af, const Eigen::MatrixXd &tf_bf) {
    return wrench_a_to_b(F_a, tf_af) + wrench_a_to_b(F_b, tf_bf);
}

// Task 3a
Eigen::Matrix3d math::matrix_exponential(const Eigen::Vector3d &w, const double theta) {
    Eigen::Matrix3d skew_w {skew_symmetric(w)};
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    const double rads = theta*M_PI/180;

    return I + sin(rads)*skew_w + (1-cos(rads))*skew_w*skew_w;
}

// Task 3b
std::pair<Eigen::Vector3d, double> math::matrix_logarithm(const Eigen::Matrix3d &r) {
    /*double theta = acos((r(0,0)+r(1,1)+r(2,2)-1)/2);
    double w_1 {(r(2,1)-r(2,3))/2/sin(theta)};
    double w_2 {(r(0,2)-r(2,0))/2/sin(theta)};
    double w_3 {(r(1,0)-r(0,1))/2/sin(theta)};

    return std::make_pair(Eigen::Vector3d {w_1,w_2,w_3}, theta);*/
    Eigen::AngleAxisd angle_axis(r); // Extracts the angle-axis representation from the rotation matrix
    Eigen::Vector3d rotation_vector = angle_axis.axis() * angle_axis.angle();
    double rotation_angle = angle_axis.angle();

    return std::make_pair(rotation_vector, rotation_angle);
}

//Task 3c. se(3) -> SE(3)
Eigen::Matrix4d math::matrix_exponential(const Eigen::Vector3d &w, const Eigen::Vector3d &v, const double theta) {
    const Eigen::Matrix3d skew_w {skew_symmetric(w)};
    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    const double rads = theta*M_PI/180;

    Eigen::Matrix3d R = matrix_exponential(w, theta);

    Eigen::Vector3d p = (I*rads + (1-cos(rads))*skew_w + (rads- sin(rads))*skew_w*skew_w)*v;

    Eigen::Matrix4d T ;
    T <<    R(0,0), R(0,1), R(0,2), p(0),
            R(1,0), R(1,1), R(1,2), p(1),
            R(2,0), R(2,1), R(2,2), p(2),
                        0,             0,              0,        1;

    return T;
}

// Task 3d. SE(3) -> se(3)
Eigen::Matrix3d  math::G(const Eigen::Vector3d &w, const double &theta) {
    Eigen::Matrix3d skew_w {skew_symmetric(w)};
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    const double rads = theta*M_PI/180;

    return I*rads + (1-cos(rads))*skew_w + (rads- sin(rads))*skew_w*skew_w;
}

Eigen::Matrix3d math::G_inverse(const Eigen::Vector3d &w, const double &theta) {
    Eigen::Matrix3d skew_w {skew_symmetric(w)};
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    const double rads = theta*M_PI/180;

    return I/rads - skew_w/2 + (1/rads-cot(rads/2)/2)*skew_w*skew_w;
}

// Problem: Works, but outputs are up to 5 % wrong
std::pair<Eigen::VectorXd, double> math::matrix_logarithm(const Eigen::Matrix4d &t) {
    Eigen::Vector3d w;
    Eigen::Vector3d v;
    double theta;

    Eigen::Matrix3d R;
    R << t(0,0), t(0,1), t(0,2),
        t(1,0), t(1,1), t(1,2),
        t(2,0), t(2,1), t(2,2);
    Eigen::Vector3d p {t(0,3), t(1,3), t(2,3)};

    if(R == Eigen::Matrix3d::Identity()) {
        w = {0,0,0};
        v = p.normalized();
        theta = sqrt(p(0)*p(0) + p(1)*p(1) + p(2)*p(2));
    }

    else {
        auto [fst, scd] = math::matrix_logarithm(R);
        w = fst;
        theta = scd;

        const Eigen::Matrix3d G_inv = math::G_inverse(w,theta*180/EIGEN_PI);
        v = G_inv*p;
    }
    Eigen::VectorXd S(6);
    S << w(0), w(1), w(2), v(0), v(1), v(2);

    return std::make_pair(S, theta);
}
