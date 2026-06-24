#include <Eigen/Dense>
#include <iostream>
#include "math/Math.h"

int main()
{
    Eigen::Vector3d v(1.0, 2.0, 3.0);
    std::cout << "Eigen works: norm = " << v.norm() << "\n";
    std::cout << "Stewart Platform V3\n";
    return 0;
}