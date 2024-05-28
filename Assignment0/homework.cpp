#include <Eigen/Core>
#include <iostream>

int main() {
    auto p = Eigen::Vector3d(2.0, 1.0, 1.0);
    // counter clock 45 degree
    auto r1 = Eigen::Matrix3d();
    r1 << std::cos(M_PI / 4), -std::sin(M_PI / 4), 0,
            std::sin(M_PI / 4), std::cos(M_PI / 4), 0,
            0, 0, 1;
    // add (1,2)
    auto r2 = Eigen::Matrix3d();
    r2 << 1, 0, 1,
            0, 1, 2,
            0, 0, 1;
    std::cout << r2 * r1 * p << std::endl;

}