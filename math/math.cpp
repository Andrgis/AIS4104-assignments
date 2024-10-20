#include <iostream>
#include <Eigen/Dense>
#include "math/math.h"


bool math::floatEquals(double a, double b) {
    return std::abs(a - b) < 1e-6;
}

double math::deg_to_rad(double degrees) {
    return degrees * M_PI / 180;
}

Eigen::Matrix3d math::rotate_x(double degrees) {
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
            0, 1, 0,
            -std::sin(radians), 0, std::cos(radians);
    return matrix;
}

Eigen::Matrix3d math::rotate_z(double degrees) {
    double radians = deg_to_rad(degrees);
    Eigen::Matrix3d matrix;
    matrix <<
            std::cos(radians), -std::sin(radians), 0,
            std::sin(radians), std::cos(radians), 0,
            0, 0, 1;
    return matrix;
}

Eigen::Matrix3d math::skew_symmetric(Eigen::Vector3d v) {
    Eigen::Matrix3d skewed_vector;
    skewed_vector <<
            0, -v.z(), v.y(),
            v.z(), 0, -v.x(),
            -v.y(), v.x(), 0;
    return skewed_vector;
}

Eigen::Matrix3d math::rotation_matrix_from_frame_axes(const Eigen::Vector3d &x, const Eigen::Vector3d &y,
                                                      const Eigen::Vector3d &z) {
    Eigen::Matrix3d matrix;
    matrix <<
            x(0), y(0), z(0),
            x(1), y(1), z(1),
            x(2), y(2), z(2);
    return matrix;
}

Eigen::Matrix3d math::rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double degrees) {
    Eigen::Matrix3d matrix{};
    double radians{deg_to_rad(degrees)};

    Eigen::Matrix3d skewed_vector{skew_symmetric(axis)};
    Eigen::Matrix3d I{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d skewed_vector_squared{skewed_vector * skewed_vector};

    // Bruker Rodrigues' formula for rot(omega,thetta)
    matrix = I + std::sin(radians) * skewed_vector + (1 - std::cos(radians)) * skewed_vector_squared;

    return matrix;
}

Eigen::Matrix3d math::rotation_matrix_from_euler_zyx(const Eigen::Vector3d &e) {
    Eigen::Matrix3d I{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d R_z{};
    Eigen::Matrix3d R_y{};
    Eigen::Matrix3d R_x{};
    Eigen::Matrix3d R{};

    R_z = rotate_z(e(0));
    R_y = rotate_y(e(1));
    R_x = rotate_x(e(2));

    R = I * R_z * R_y * R_x;

    return R;
}

Eigen::Matrix3d math::rotation_matrix_from_euler_yzx(const Eigen::Vector3d &e) {
    const Eigen::Matrix3d I{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d R_z{};
    Eigen::Matrix3d R_y{};
    Eigen::Matrix3d R_x{};
    Eigen::Matrix3d R{};

    R_y = rotate_y(e(0));
    R_z = rotate_z(e(1));
    R_x = rotate_x(e(2));

    R = I * R_y * R_z * R_x;

    return R;
}

Eigen::Matrix4d math::transformation_matrix(const Eigen::Matrix3d &r, const Eigen::Vector3d &p) {
    Eigen::Matrix4d matrix;
    // implement the necessary equations and functionality.
    matrix <<
            r(0, 0), r(0, 1), r(0, 2), p(0),
            r(1, 0), r(1, 1), r(1, 2), p(1),
            r(2, 0), r(2, 1), r(2, 2), p(2),
            0, 0, 0, 1;
    return matrix;
}

Eigen::Vector3d math::euler_zyx_from_rotation(const Eigen::Matrix3d &r) {
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;

    if (floatEquals(r(2, 0), -1.0)) {
        b = EIGEN_PI / 2.0;
        a = 0.0;
        c = std::atan2(r(0, 1), r(1, 1));
    } else if (floatEquals(r(2, 0), 1.0)) {
        b = -(EIGEN_PI / 2.0);
        a = 0.0;
        c = -std::atan2(r(0, 1), r(1, 1));
    } else {
        b = std::atan2(-r(2, 0), std::sqrt(r(0, 0) * r(0, 0) + r(1, 0) * r(1, 0)));
        a = std::atan2(r(1, 0), r(0, 0));
        c = std::atan2(r(2, 1), r(2, 2));
    }
    return Eigen::Vector3d{a, b, c};
}

/* Eigen::Vector3d math::euler_zyx_from_transformation(const Eigen::Matrix4d &t) {
    Eigen::Matrix3d R {};
    R << t(0, 0), t(0, 1), t(0, 2),
        t(1, 0), t(1, 1), t(1, 2),
        t(2, 0), t(2, 1), t(2, 2);

    return euler_zyx_from_rotation(R);
}*/

Eigen::VectorXd math::twist(const Eigen::Vector3d &w, const Eigen::Vector3d &v) {
    Eigen::VectorXd twist(6);
    // twist << w(0), w(1), w(2), v(0), v(1), v(2);
    twist << w, v;
    return twist;
}

Eigen::VectorXd math::screw_axis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h) {
    const Eigen::Vector3d w{s.x(), s.y(), s.z()};
    const Eigen::Vector3d v{-s.cross(q) + h * s};
    Eigen::VectorXd S(6);
    S << w,v;
    return S;
}

Eigen::MatrixXd math::adjoint_matrix(const Eigen::Matrix4d &tf) {
    // Eigen::Vector3d p{tf(0, 3), tf(1, 3), tf(2, 3)};
    Eigen::Vector3d p = tf.block<3, 1>(0, 3);
    Eigen::Matrix3d skew_p{skew_symmetric(p)};
    Eigen::Matrix3d R = tf.block<3, 3>(0, 0);
    // R << tf(0, 0), tf(0, 1), tf(0, 2),
    // tf(1, 0), tf(1, 1), tf(1, 2),
    // tf(2, 0), tf(2, 1), tf(2, 2);
    // Eigen::Matrix3d pR{skew_p * R};

    Eigen::MatrixXd adjoint(6, 6);
    adjoint.block<3, 3>(0, 0) = R;
    adjoint.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
    adjoint.block<3, 3>(3, 0) = skew_p * R;
    adjoint.block<3, 3>(3, 3) = R;

    // adjoint << R(0, 0), R(0, 1), R(0, 2), 0, 0, 0,
    // R(1, 0), R(1, 1), R(1, 2), 0, 0, 0,
    // R(2, 0), R(2, 1), R(2, 2), 0, 0, 0,
    // pR(0, 0), pR(0, 1), pR(0, 2), R(0, 0), R(0, 1), R(0, 2),
    // pR(1, 0), pR(1, 1), pR(1, 2), R(1, 0), R(1, 1), R(1, 2),
    // pR(2, 0), pR(2, 1), pR(2, 2), R(2, 0), R(2, 1), R(2, 2);

    return adjoint;
}

double math::cot(const double x) {
    return 1.0 / std::tan(x);
}

void math::wrench_in_s_and_w() {
    const Eigen::Vector3d f_w{-30, 0, 0};
    const Eigen::Vector3d m_s{0, 0, 2};
    const Eigen::Vector3d e_ws{60, -60, 0}; // Euler angles YZX in degrees.
    Eigen::Matrix3d R_ws{rotation_matrix_from_euler_yzx(e_ws)};

    Eigen::Vector3d m_w{R_ws * m_s};
    Eigen::Vector3d f_s{R_ws.transpose() * f_w};


    //std::cout << "Still in progress" << std::endl;
    std::cout << "f_w: " << f_w.transpose() << std::endl;
    std::cout << "m_w: " << m_w.transpose() << std::endl;
    std::cout << "f_s: " << f_s.transpose() << std::endl;
    std::cout << "m_s: " << m_s.transpose() << std::endl;
    std::cout << "R_ws: " << std::endl << R_ws << std::endl << std::endl;
}

Eigen::VectorXd math::wrench_w(const Eigen::Vector3d &f_w, const Eigen::Vector3d &m_s, const Eigen::Vector3d &e_ws) {
    Eigen::Matrix3d R_ws{rotation_matrix_from_euler_yzx(e_ws)};
    Eigen::Vector3d m_w{R_ws * m_s};
    //Eigen::Vector3d f_s {R_ws.transpose()*f_w};

    Eigen::VectorXd F_w(6);
    F_w << m_w(0), m_w(1), m_w(2), f_w(0), f_w(1), f_w(2);

    return F_w;
}

Eigen::VectorXd math::wrench_s(const Eigen::Vector3d &f_w, const Eigen::Vector3d &m_s, const Eigen::Vector3d &e_ws) {
    Eigen::Matrix3d R_ws{rotation_matrix_from_euler_yzx(e_ws)};
    //Eigen::Vector3d m_w {R_ws*m_s};
    Eigen::Vector3d f_s{R_ws.transpose() * f_w};

    Eigen::VectorXd F_s(6);
    F_s << m_s(0), m_s(1), m_s(2), f_s(0), f_s(1), f_s(2);

    return F_s;
}

Eigen::VectorXd math::wrench_a_to_b(const Eigen::VectorXd &F_a, const Eigen::Matrix4d &tf_ab) {
    Eigen::MatrixXd Ad_Tab = math::adjoint_matrix(tf_ab);

    return Ad_Tab.transpose() * F_a;
}

// Task 2b
Eigen::VectorXd math::wrench_f(const Eigen::VectorXd &F_a, const Eigen::VectorXd &F_b, const Eigen::MatrixXd &tf_af,
                               const Eigen::MatrixXd &tf_bf) {
    return wrench_a_to_b(F_a, tf_af) + wrench_a_to_b(F_b, tf_bf);
}

// Task 3a
Eigen::Matrix3d math::matrix_exponential(const Eigen::Vector3d &w, double theta) {
    const Eigen::Matrix3d skew_w{skew_symmetric(w)};
    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    const double rads = theta * M_PI / 180.0;

    return I + std::sin(rads) * skew_w + (1 - std::cos(rads)) * skew_w * skew_w;
}

// Task 3b
std::pair<Eigen::Vector3d, double> math::matrix_logarithm(const Eigen::Matrix3d &r) {
    double theta{};
    double w_1{};
    double w_2{};
    double w_3{};
    Eigen::Vector3d w;
    const double trR = r.trace();

    if (floatEquals(trR, 3.0)) {
        theta = 0.0;
        w << 0.0, 0.0, 0.0;
    } else if (floatEquals(trR, -1.0)) {
        theta = EIGEN_PI;
        w_1 = r(0, 2) / std::sqrt(2.0 * (1.0 + r(2, 2)));
        w_2 = r(1, 2) / std::sqrt(2.0 * (1.0 + r(2, 2)));
        w_3 = (1.0 + r(2, 2)) / std::sqrt(2.0 * (1.0 + r(2, 2)));
        w << w_1, w_2, w_3;
    } else {
        theta = std::acos(0.5*(trR - 1.0));
        Eigen::Matrix3d skew_w = (r - r.transpose()) / (2.0 * std::sin(theta));
        w << skew_w(2, 1), skew_w(0, 2), skew_w(1, 0);
    }


    return std::make_pair(w, theta * 180.0/ EIGEN_PI);
}

//Task 3c. se(3) -> SE(3)
Eigen::Matrix4d math::matrix_exponential(const Eigen::Vector3d &w, const Eigen::Vector3d &v,double theta) {
    const Eigen::Matrix3d skew_w{skew_symmetric(w)};
    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    const double rads = theta * M_PI / 180;

    Eigen::Matrix3d R = matrix_exponential(w, theta);

    Eigen::Vector3d p = (I * rads + (1 - cos(rads)) * skew_w + (rads - sin(rads)) * skew_w * skew_w) * v;

    Eigen::Matrix4d T; //block
    T << R(0, 0), R(0, 1), R(0, 2), p(0),
            R(1, 0), R(1, 1), R(1, 2), p(1),
            R(2, 0), R(2, 1), R(2, 2), p(2),
            0, 0, 0, 1;

    return T; // LEGG INN CASE FOR PRISMATIC
}

// Task 3d. SE(3) -> se(3)
Eigen::Matrix3d math::G(const Eigen::Vector3d &w, const double &theta) { // Primitiver trengs ikke sendes som const
    Eigen::Matrix3d skew_w{skew_symmetric(w)};
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    const double rads = theta * M_PI / 180.0;

    return I * rads + (1 - std::cos(rads)) * skew_w + (rads - std::sin(rads)) * skew_w * skew_w;
}

Eigen::Matrix3d math::G_inverse(const Eigen::Vector3d &w, const double &degrees) {
    Eigen::Matrix3d skew_w{skew_symmetric(w)};
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    const double rads = degrees * M_PI / 180;

    return I / rads - skew_w / 2 + (1 / rads - cot(rads / 2) / 2) * skew_w * skew_w;
}

// Working
std::pair<Eigen::VectorXd, double> math::matrix_logarithm(const Eigen::Matrix4d &t) {
    Eigen::Vector3d w;
    Eigen::Vector3d v;
    double degrees;

    Eigen::Matrix3d R = t.block<3,3>(0,0);
    Eigen::Vector3d p = t.block<3,1>(0,3);

    if (floatEquals(R.trace(),3.0)) {
        w << 0.0, 0.0, 0.0;
        v = p.normalized(); // v = p/|p|
        degrees = p.norm(); // theta = |p|
    } else {
        auto [fst, snd] = matrix_logarithm(R);
        w = fst;
        degrees = snd;

        const Eigen::Matrix3d G_inv = G_inverse(w, degrees);
        v = G_inv * p;
    }
    Eigen::VectorXd S(6);
    S << w, v;

    return std::make_pair(S, degrees);
}

void math::print_pose(const Eigen::Matrix4d &tf, std::string label) {
    Eigen::Matrix3d R = tf.block<3,3>(0,0);
    Eigen::Vector3d p = tf.block<3,1>(0,3);

    Eigen::Vector3d e_zyx = euler_zyx_from_rotation(R);

    std::cout << "Label: " << label << std::endl;
    std::cout << "Euler ZYX angles: " << e_zyx.transpose() * 180 / EIGEN_PI << std::endl;
    std::cout << "Linear position: " << p.transpose() << std::endl;
    std::cout << " " << std::endl;
}

// Task 4b
Eigen::Matrix4d math::planar_3r_fk_transform(const std::vector<double> &joint_positions) {
    constexpr double L1 = 10, L2 = 10, L3 = 10;

    // Making transformation matrix for each succeeding frame. Every joint rotates around z-axis.
    const Eigen::Matrix4d T01 = transformation_matrix(rotate_z(joint_positions[0]), Eigen::Vector3d(0, 0, 0));
    const Eigen::Matrix4d T12 = transformation_matrix(rotate_z(joint_positions[1]), Eigen::Vector3d(L1, 0, 0));
    const Eigen::Matrix4d T23 = transformation_matrix(rotate_z(joint_positions[2]), Eigen::Vector3d(L2, 0, 0));
    const Eigen::Matrix4d T34 = transformation_matrix(rotate_z(0), Eigen::Vector3d(L3, 0, 0));

    Eigen::Matrix4d T04 = T01 * T12 * T23 * T34;

    return T04;
}

void math::test_planar_3r_fk_transform(const std::string &label, const std::vector<double> &joint_positions) {
    const Eigen::Matrix4d T{math::planar_3r_fk_transform(joint_positions)};

    print_pose(T,label);
}

// Task 4c
Eigen::Matrix4d math::planar_3r_fk_screw(const std::vector<double> &joint_positions) {
    constexpr double L1 = 10, L2 = 10, L3 = 10;

    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    M(0, 3) = L1 + L2 + L3;

    Eigen::Vector3d w1, w2, w3;
    w1 << 0, 0, 1;
    w2 << 0, 0, 1;
    w3 << 0, 0, 1;

    Eigen::Vector3d v1, v2, v3; //Use screw function.
    v1 << 0, 0, 0;
    v2 << 0, -L1, 0;
    v3 << 0, -L1 - L2, 0;

    const Eigen::Matrix4d e1 = matrix_exponential(w1, v1, joint_positions[0]);
    const Eigen::Matrix4d e2 = matrix_exponential(w2, v2, joint_positions[1]);
    const Eigen::Matrix4d e3 = matrix_exponential(w3, v3, joint_positions[2]);

    const Eigen::Matrix4d T04 = e1 * e2 * e3 * M;

    return T04;
}

void math::test_planar_3r_fk_screw(const std::string &label, const std::vector<double> &joint_positions) {
    const Eigen::Matrix4d T{planar_3r_fk_screw(joint_positions)};

    print_pose(T,label);
}

// TASK 5
// Task 5a
Eigen::Matrix4d math::ur3e_fk_screw(const std::vector<double> &joint_positions) {
    //OK
    constexpr double h1{0.15185}, l1{0.24355}, l2{0.2132}, h2{0.08535},
            y1{0.13105}, y2{0.0921};

    Eigen::Matrix4d M; //ok
    M << 1, 0, 0, -l1 - l2,
        0, 0, -1, -y1 - y2,
        0, 1, 0, h1 - h2,
        0, 0, 0, 1;

    Eigen::Vector3d w0, w1, w2, w3, w4, w5;
    w0 << 0, 0, 1; //(base) //ok
    w1 << 0, -1, 0; //ok
    w2 << 0, -1, 0; //ok
    w3 << 0, -1, 0; //ok
    w4 << 0, 0, -1; //ok
    w5 << 0, -1, 0; //ok

    Eigen::Vector3d v0, v1, v2, v3, v4, v5;
    v0 << 0, 0, 0; //ok
    v1 << h1, 0, 0; //ok
    v2 << h1, 0, l1; //ok
    v3 << h1, 0, l1 + l2; //ok
    v4 << y1, -l1 - l2, 0; //ok
    v5 << h1 - h2, 0, l1 + l2; //ok


    const Eigen::Matrix4d e0 = matrix_exponential(w0, v0, joint_positions[0]);
    const Eigen::Matrix4d e1 = matrix_exponential(w1, v1, joint_positions[1]);
    const Eigen::Matrix4d e2 = matrix_exponential(w2, v2, joint_positions[2]);
    const Eigen::Matrix4d e3 = matrix_exponential(w3, v3, joint_positions[3]);
    const Eigen::Matrix4d e4 = matrix_exponential(w4, v4, joint_positions[4]);
    const Eigen::Matrix4d e5 = matrix_exponential(w5, v5, joint_positions[5]);

    const Eigen::Matrix4d T = e0 * e1 * e2 * e3 * e4 * e5 * M;

    return T;
}


void math::test_ur3e_fk_screw(const std::string &label, const std::vector<double> &joint_positions) {
    const Eigen::Matrix4d T{ur3e_fk_screw(joint_positions)};

    print_pose(T,label);
}

Eigen::Matrix4d DH_transformation_matrix(const double &joint_angle, const double &alpha, const double &a,
                                         const double &d) {
    Eigen::Matrix4d T_mn;
    double joint_rads = joint_angle * M_PI / 180;
    double alpha_rads = alpha * M_PI / 180;
    T_mn << cos(joint_rads), -sin(joint_rads) * cos(alpha_rads), sin(joint_rads) * sin(alpha_rads), a * cos(joint_rads),
            //ok
            sin(joint_rads), cos(joint_rads) * cos(alpha_rads), -cos(joint_rads) * sin(alpha_rads), a * sin(joint_rads),
            0, sin(alpha_rads), cos(alpha_rads), d,
            0, 0, 0, 1;
    return T_mn;
}

Eigen::Matrix4d math::ur3e_fk_transform(const std::vector<double> &joint_positions) {
    //YES!
    constexpr double h0{0.15185}, h1{0.24355}, h2{0.2132}, h3{0.08535},
            y0{0.13105}, y1{0.0921};

    const Eigen::Matrix4d T01 = DH_transformation_matrix(joint_positions[0], 90, 0, h0);
    const Eigen::Matrix4d T12 = DH_transformation_matrix(joint_positions[1], 0, -h1, 0);
    const Eigen::Matrix4d T23 = DH_transformation_matrix(joint_positions[2], 0, -h2, 0);
    const Eigen::Matrix4d T34 = DH_transformation_matrix(joint_positions[3], 90, 0, y0);
    const Eigen::Matrix4d T45 = DH_transformation_matrix(joint_positions[4], -90, 0, h3);
    const Eigen::Matrix4d T56 = DH_transformation_matrix(joint_positions[5], 0, 0, y1);


    Eigen::Matrix4d T06 = T01 * T12 * T23 * T34 * T45 * T56;

    return T06;
}

void math::test_ur3e_fk_transform(const std::string &label, const std::vector<double> &joint_positions) {
    const Eigen::Matrix4d T{ur3e_fk_transform(joint_positions)};

    print_pose(T,label);
}

// Assignment 3
// T1a
Eigen::VectorXd math::std_vector_to_eigen(const std::vector<double> &v) {
    const int l = v.size();
    Eigen::VectorXd v_eigen(l);
    for (int i = 0; i < l; i++) {
        v_eigen(i) = v[i];
    }
    return v_eigen;
}
// T1b
bool math::is_average_below_eps(const std::vector<double> &values, double eps, uint8_t n_values) {
    const int l = values.size();

    if (l < n_values) {
        return false;
    } else {
        int sum = 0;
        for (int i = l - n_values; i < l; i++) {
            sum += values[i];
        }
        double avg = sum / n_values;
        if (avg > eps) {
            return false;
        } else {
            return true;
        }
    }
}
// T1c // Works perfectly
std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd>> math::ur3e_space_chain() {
    constexpr double h1{0.15185}, l1{0.24355}, l2{0.2132}, h2{0.08535},
            y1{0.13105}, y2{0.0921};

    Eigen::Matrix4d M; //ok
    M << 1.0, 0.0,  0.0, -l1 - l2,
         0.0, 0.0, -1.0, -y1 - y2,
         0.0, 1.0,  0.0,  h1 - h2,
         0.0, 0.0,  0.0, 1.0;

    Eigen::Vector3d w0, w1, w2, w3, w4, w5;
    w0 << 0.0, 0.0, 1.0; //(base) //ok
    w1 << 0.0, -1.0, 0.0; //ok
    w2 << 0.0, -1.0, 0.0; //ok
    w3 << 0.0, -1.0, 0.0; //ok
    w4 << 0.0, 0.0, -1.0; //ok
    w5 << 0.0, -1.0, 0.0; //ok

    Eigen::Vector3d q0, q1, q2, q3, q4, q5;
    q0 << 0.0, 0.0, 0.0;
    q1 << 0.0, 0.0, h1;
    q2 << -l1, 0.0, h1;
    q3 << -l1-l2, 0.0, h1;
    q4 << -l1-l2, -y1, 0.0;
    q5 << -l1-l2, 0.0, h1-h2;

    const Eigen::VectorXd S0 = screw_axis(q0, w0, 0.0);
    const Eigen::VectorXd S1 = screw_axis(q1, w1, 0.0);
    const Eigen::VectorXd S2 = screw_axis(q2, w2, 0.0);
    const Eigen::VectorXd S3 = screw_axis(q3, w3, 0.0);
    const Eigen::VectorXd S4 = screw_axis(q4, w4, 0.0);
    const Eigen::VectorXd S5 = screw_axis(q5, w5, 0.0);

    return std::make_pair(M, std::vector{S0, S1, S2, S3, S4, S5});

}
// T1d // Works perfectly
Eigen::Matrix4d math::ur3e_space_fk(const Eigen::VectorXd &joint_positions){
    auto [M, SSS] = ur3e_space_chain();
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    for (int i=0; i<6; i++) {
        Eigen::Matrix4d ei = matrix_exponential(SSS[i].block<3,1>(0,0), SSS[i].block<3,1>(3,0), joint_positions[i]);
        T = T*ei;
    }
    T = T*M;

    return T;
}
// T1e // Works perfectly
std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd>> math::ur3e_body_chain() {
    std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd>> result = ur3e_space_chain();
    Eigen::Matrix4d M = result.first; //Msb
    std::vector<Eigen::VectorXd> SSS = result.second;
    Eigen::MatrixXd Ad_bs = adjoint_matrix(M.inverse()); // Msb^-1 = Mbs
    std::vector<Eigen::VectorXd> BBB(SSS.size());

    for (int i=0 ; i<SSS.size() ; i++) {
        const Eigen::VectorXd B = Ad_bs * SSS[i];  // Vb = Ad_bs * Vs
        BBB[i] = B;
    }

    return std::make_pair(M.inverse(), BBB);
}
// T1f // Works perfectly
Eigen::Matrix4d math::ur3e_body_fk(const Eigen::VectorXd &joint_positions) {
    std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd>> result = ur3e_body_chain();
    Eigen::Matrix4d T_B = result.first; // T_B = M_bs
    std::vector<Eigen::VectorXd> BBB = result.second;

    for (int i=0; i<BBB.size(); i++) { // Inputs should be ok, but seems like t
        Eigen::Matrix4d ei = matrix_exponential(BBB[i].block<3,1>(0,0), BBB[i].block<3,1>(3,0), joint_positions[i]);
        T_B = T_B * ei; // Ok
    }
    return T_B;
}
// T1g // Forward kinematics for space frame is good, but unsure about body frame. Seems to be a problem somewhere. Can't find it.
void math::ur3e_test_fk(){
std::cout << "Forward kinematics tests" << std::endl;
print_pose(ur3e_space_fk(std_vector_to_eigen(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0})));
print_pose(ur3e_body_fk(std_vector_to_eigen(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0})));
std::cout << std::endl;
print_pose(ur3e_space_fk(std_vector_to_eigen(std::vector<double>{0.0, 0.0, 0.0, -90.0, 0.0, 0.0})));
print_pose(ur3e_body_fk(std_vector_to_eigen(std::vector<double>{0.0, 0.0, 0.0, -90.0, 0.0, 0.0})));
std::cout << std::endl;
print_pose(ur3e_space_fk(std_vector_to_eigen(std::vector<double>{0.0, 0.0, -180.0, 0.0, 0.0, 0.0})));
print_pose(ur3e_body_fk(std_vector_to_eigen(std::vector<double>{0.0, 0.0, -180.0, 0.0, 0.0, 0.0})));
std::cout << std::endl;
print_pose(ur3e_space_fk(std_vector_to_eigen(std::vector<double>{0.0, 0.0, -90.0, 0.0, 0.0, 0.0})));
print_pose(ur3e_body_fk(std_vector_to_eigen(std::vector<double>{0.0, 0.0, -90.0, 0.0, 0.0, 0.0})));
}

// Task 2: Numerical optimization
double dfdx(const std::function<double(double)> &f, const double &x, const double &dx = 0.00001) {
    return (f(x+dx)-f(x-dx))/(2*dx);
}

// T2a - Implement Newton-Raphson to find the root of a scalar function.
std::pair<uint32_t, double> math::newton_raphson_root_find(const std::function<double(double)> &f, double &x_0, double dx_0, double eps) {
    double x_n = x_0;
    uint32_t iter{0};
    uint32_t max_iter{1000};

    for (double dx_n = dx_0 ; dx_n>eps and iter < max_iter; iter++) {
        double df_n = dfdx(f, x_n);
        double x_np = x_n - f(x_n)/df_n;
        dx_n = std::abs(x_np-x_n);
        x_n = x_np;
    }

    return std::make_pair(iter, x_n);
}
// Finds extreme points. //Seems to be working
std::pair<uint32_t, double> math::gradient_descent_extreme_find(const std::function<double(double)> &f, double &x_0, double gamma, double dx_0, double eps) {
    double x_n = x_0;
    // double x_n = x_nm - gamma*dfdx(f, x_nm);
    uint32_t iter{0};
    uint32_t max_iter{1000};

    for (double dx_n = dx_0 ; dx_n>eps and iter < max_iter; iter++) {
        double df_n = dfdx(f, x_n);
        // double df_nm = dfdx(f, x_nm);
        // gamma = std::abs((x_n-x_nm)*(df_n-df_nm))/(std::abs(df_n-df_nm)*std::abs(df_n-df_nm));
        double x_np = x_n - gamma*df_n;
        dx_n = std::abs(x_np-x_n);
        x_n = x_np;
    }

    return std::make_pair(iter, x_n);
}
// Finds root.  // Seems to be working
std::pair<uint32_t, double> math::gradient_descent_root_find(const std::function<double(double)> &f, double &x_0, double gamma, double dx_0, double eps) {
    double x_nm = x_0;
    double x_n = x_nm + gamma*f(x_nm);
    uint32_t iter{0};
    uint32_t max_iter{1000};

    for (double dx_n = dx_0 ; dx_n>eps and iter < max_iter; iter++) {
        double df_n = f(x_n);
        double df_nm = f(x_nm);
        gamma = std::abs((x_n-x_nm)*(df_n-df_nm))/(std::abs(df_n-df_nm)*std::abs(df_n-df_nm)); // gamma update (en.wikipedia.org/wiki/Gradient_descent)
        //std::cout << gamma << std::endl;
        double x_np = x_n + gamma*df_n;
        dx_n = std::abs(x_np-x_n);
        x_nm = x_n;
        x_n = x_np;
    }

    return std::make_pair(iter, x_n);
}


void math::test_newton_raphson_root_find(const std::function<double(double)> &f, double x0){
    auto [iterations, x_hat] = newton_raphson_root_find(f, x0);
    std::cout << "NR root f, x0=" << x0 << " -> it=" << iterations << " x=" << x_hat << " f(x)=" << f(x_hat) << std::endl;
}

void math::test_gradient_descent_root_find(const std::function<double(double)> &f, double x0)
{
auto [iterations, x_hat] = gradient_descent_root_find(f, x0);
std::cout << "GD root f, x0=" << x0 << " -> it=" << iterations << " x=" << x_hat << " f(x)=" << f(x_hat) << std::endl;
}

void math::test_root_find(){
    std::cout << "Root finding tests" << std::endl;
    auto f1 = [](double x)
    {
    return (x - 3.0) * (x - 3.0) - 1.0;
    };
    test_newton_raphson_root_find(f1, -20);
    test_gradient_descent_root_find(f1, -200);
    std::cout << " " << std::endl;
}
// Task 3 - Velocity kinematics: UR3e 6R open chain
// T.3 a) Construct the space Jacobian.
Eigen::MatrixXd math::ur3e_space_jacobian(const Eigen::VectorXd &current_joint_positions) {
    std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd>> result = ur3e_space_chain();
    //Eigen::Matrix4d M = result.first;
    std::vector<Eigen::VectorXd> SSS = result.second;

    Eigen::MatrixXd jacobian(6,SSS.size());
    jacobian.block<6,1>(0,0) = SSS[0];

    for (int i=1; i<SSS.size(); i++) { // i = column. j = joint
        Eigen::Matrix4d T_S = Eigen::Matrix4d::Identity();
        for (int j=i-1; j>-1; j--) {
            Eigen::Matrix4d ei = matrix_exponential(SSS[j].block<3,1>(0,0), SSS[j].block<3,1>(3,0), current_joint_positions[j]);
            T_S = ei * T_S;
        }
        Eigen::MatrixXd transforms_S_to_Arbitrary_Theta = adjoint_matrix(T_S);
        jacobian.block<6,1>(0,i) = transforms_S_to_Arbitrary_Theta*SSS[i];
    }
    return jacobian;
}
// T.3 b) Construct the body Jacobian
Eigen::MatrixXd math::ur3e_body_jacobian(const Eigen::VectorXd &current_joint_positions){
    std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd>> result = ur3e_body_chain();
    //Eigen::Matrix4d M = result.first;
    std::vector<Eigen::VectorXd> BBB = result.second;

    Eigen::MatrixXd jacobian(6,BBB.size());
    jacobian.block<6,1>(0,BBB.size()-1) = BBB[BBB.size()-1];

    for (int i=BBB.size()-2; i>-1; i--) { // i = column. j = joint
        Eigen::Matrix4d T_B = Eigen::Matrix4d::Identity();
        for (int j=BBB.size()-1; j>i; j--) {
            Eigen::Matrix4d ei = matrix_exponential(BBB[j].block<3,1>(0,0), BBB[j].block<3,1>(3,0), -current_joint_positions[j]);
            T_B = T_B*ei;
        }
        Eigen::MatrixXd transforms_B_to_Arbitrary_Theta = adjoint_matrix(T_B);
        jacobian.block<6,1>(0,i) = transforms_B_to_Arbitrary_Theta*BBB[i];
    }
    return jacobian;
}
// The mistake was in this function. It was set to: tsb = ur3e_body_fk(), but that is tbs.
void math::ur3e_test_jacobian(const Eigen::VectorXd &joint_positions){
    Eigen::Matrix4d tsb = ur3e_space_fk(joint_positions);
    auto [m, space_screws] = ur3e_space_chain();
    Eigen::MatrixXd jb = ur3e_body_jacobian(joint_positions);
    Eigen::MatrixXd js = ur3e_space_jacobian(joint_positions);
    Eigen::MatrixXd ad_tsb = adjoint_matrix(tsb);
    Eigen::MatrixXd ad_tbs = adjoint_matrix(tsb.inverse());
    std::cout << "Jb: " << std::endl << jb << std::endl << "Ad_tbs*Js:" << std::endl << ad_tbs * js << std::endl << std::endl;
    std::cout << "Js: " << std::endl << js << std::endl << "Ad_tsb*Jb:" << std::endl << ad_tsb * jb << std::endl << std::endl;
    std::cout << "d Jb: " << std::endl << jb - ad_tbs * js << std::endl << std::endl;
    std::cout << "d Js: " << std::endl << js - ad_tsb * jb << std::endl << std::endl;
    }
void math::ur3e_test_jacobian(){
    std::cout << "Jacobian matrix tests" << std::endl;
    ur3e_test_jacobian(std_vector_to_eigen(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
    ur3e_test_jacobian(std_vector_to_eigen(std::vector<double>{45.0, -20.0, 10.0, 2.5, 30.0, -50.0}));
}

void math::debugging_ur3e_body_fk(const Eigen::VectorXd &joint_positions) {
    std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd>> result = ur3e_body_chain();
    Eigen::Matrix4d T_B = result.first; // T_B = M_bs
    std::vector<Eigen::VectorXd> BBB = result.second;

    for (int i=0; i<BBB.size(); i++) {
        Eigen::Matrix4d ei = matrix_exponential(BBB[i].block<3,1>(0,0), BBB[i].block<3,1>(3,0), joint_positions[i]);
        T_B = T_B * ei;
        std::cout << i << ": " << std::endl << ei << std::endl << T_B << std::endl;
    }
}
// CANT GET IT RIGHT! DONT KNOW WHERE THE PROBLEM IS. HAVE SPENT TOO MANY HOURS TROUBLESHOOTING
std::pair<size_t, Eigen::VectorXd> math::ur3e_ik_body(const Eigen::Matrix4d &t_sd, const Eigen::VectorXd &current_joint_positions, double gamma, double v_e, double w_e) {

    Eigen::MatrixXd J_b = ur3e_body_jacobian(current_joint_positions); // Jacobian for first position
    Eigen::VectorXd theta = current_joint_positions; // The iterating joint positions
    Eigen::Matrix4d t_sb = ur3e_body_fk(current_joint_positions); // Current position and orientation of end effector
    auto [S_sb, angle_s] = matrix_logarithm(t_sb); // Finding V of current position
    Eigen::VectorXd V_sb = S_sb*angle_s*c_deg_to_rad;
    Eigen::Matrix4d t_bs = t_sb.inverse();
    Eigen::Matrix4d t_bd = t_bs * t_sd;
    auto [S_bd, angle_bd] = matrix_logarithm(t_bd); // Finding V of desired position
    Eigen::VectorXd V_b = S_bd*angle_bd*c_deg_to_rad;
    constexpr int max_iter = 5;  // Set low to make computing quicker, while troubleshooting
    int iter = 0;

    bool err = true;
    // Inverse kinematics continues until results are inside error margins. i = iteration
    while (iter<max_iter and err) {
        theta = theta - J_b.completeOrthogonalDecomposition().pseudoInverse()*V_b;
        // Updating:
        J_b = ur3e_body_jacobian(theta); // Updating jacobian
        t_sb = ur3e_body_fk(theta); // Current position and orientation of end effector
        t_bs = t_sb.inverse();
        t_bd = t_bs * t_sd;
        auto [fst, snd] = matrix_logarithm(t_bd);
        S_bd = fst;
        angle_bd = snd;
        V_b = S_bd*angle_bd*c_deg_to_rad;

        Eigen::Vector3d w_b = V_b.head(3); // Desired angular position
        Eigen::Vector3d v_b = V_b.tail(3); // Desired translation position
        std::cout <<"|w_b|: "<< w_b.norm() <<", |v_b|: "<< v_b.norm() << std::endl;
        err = w_b.norm()>w_e || v_b.norm()>v_e; // error between current position and desired position
        iter++;
    }
    return std::make_pair(iter, theta);
}

void math::ur3e_ik_test_pose(const Eigen::Vector3d &pos, const Eigen::Vector3d &zyx, const Eigen::VectorXd &j0)
{
    std::cout << "Test from pose" << std::endl;
    Eigen::Matrix4d t_sd = transformation_matrix(rotation_matrix_from_euler_zyx(zyx), pos);
    auto [iterations, j_ik] = ur3e_ik_body(t_sd, j0);
    Eigen::Matrix4d t_ik = ur3e_body_fk(j_ik);
    print_pose(t_ik, " IK pose");
    print_pose(t_sd, "Desired pose");
    std::cout << "Converged after " << iterations << " iterations" << std::endl;
    std::cout << "J_0: " << j0.transpose() << std::endl;
    std::cout << "J_ik: " << j_ik.transpose() << std::endl << std::endl;
}
void math::ur3e_ik_test_configuration(const Eigen::VectorXd &joint_positions, const Eigen::VectorXd &j0)
{
    std::cout << "Test from configuration" << std::endl;
    Eigen::Matrix4d t_sd = ur3e_space_fk(joint_positions);
    auto [iterations, j_ik] = ur3e_ik_body(t_sd, j0);
    Eigen::Matrix4d t_ik = ur3e_body_fk(j_ik);
    print_pose(t_ik, " IK pose");
    print_pose(t_sd, "Desired pose");
    std::cout << "Converged after " << iterations << " iterations" << std::endl;
    std::cout << "J_0: " << j0.transpose() << std::endl;
    std::cout << "J_d: " << joint_positions.transpose() << std::endl;
    std::cout << "J_ik: " << j_ik.transpose() << std::endl << std::endl;
}
void math::ur3e_ik_test()
{
    Eigen::VectorXd j_t0 = std_vector_to_eigen(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    Eigen::VectorXd j_t1 = std_vector_to_eigen(std::vector<double>{0.0, 0.0, -89.0, 0.0, 0.0, 0.0});
    ur3e_ik_test_pose(Eigen::Vector3d{0.3289, 0.22315, 0.36505}, Eigen::Vector3d{0.0, 90.0, -90.0}, j_t0);
    ur3e_ik_test_pose(Eigen::Vector3d{0.3289, 0.22315, 0.36505}, Eigen::Vector3d{0.0, 90.0, -90.0}, j_t1);
    Eigen::VectorXd j_t2 = std_vector_to_eigen(std::vector<double>{50.0, -30.0, 20.0, 0.0, -30.0, 50.0});
    Eigen::VectorXd j_d1 = std_vector_to_eigen(std::vector<double>{45.0, -20.0, 10.0, 2.5, 30.0,-50.0});
    ur3e_ik_test_configuration(j_d1, j_t0);
    ur3e_ik_test_configuration(j_d1, j_t2);
}