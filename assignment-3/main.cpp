#include <iostream>
#include <Eigen/Dense>
#include "math/math.h"

void test_T1a() {
    const std::vector<double> v = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
    std::cout << "Task 1 a: " << std::endl;
    std::cout << "Testing std_vector_to_eigen({ 1, 2, 3, 4, 5, 6, 7, 8, 9 }): " << std::endl;
    std::cout << math::std_vector_to_eigen(v) << std::endl << std::endl;
}
void test_T1b() {
    const std::vector<double> v = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
    std::cout << "Task 1 b: " << std::endl;
    std::cout << "Testing is_average_below_eps({ 1, 2, 3, 4, 5, 6, 7, 8, 9 }): " << std::endl;
    std::cout << math::is_average_below_eps(v) << std::endl << std::endl;
}
void test_T1c() {
    std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd>> result = math::ur3e_space_chain();
    Eigen::Matrix4d M = result.first;
    std::vector<Eigen::VectorXd> SSS = result.second;
    std::cout << "Task 1 c: " << std::endl;
    std::cout << "Testing ur3e_space_chain(): " << std::endl;
    std::cout << "Frame of end effector in zero configuration: " << std::endl << M << std::endl;
    std::cout << "Screw axes: " << std::endl;
    for(int i=0; i < SSS.size(); i++) {
        std::cout << "S" << i << ": " << SSS[i].transpose() << std::endl;
    }
    std::cout << std::endl;
}
void test_T1d() {
    math::print_pose("Task 1d: ",math::ur3e_space_fk(math::std_vector_to_eigen(std::vector<double>{10.0, 20.0, 30.0, 40.0, 50.0, 60.0})));
}


int main()
{
    test_T1a();
    test_T1b();
    test_T1c();
    test_T1d();

    return 0;
}




