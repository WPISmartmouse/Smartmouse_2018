#include <iostream>
#include <common/Eigen/Eigen/Dense>

int main()
{
  Eigen::Matrix<double, 3, 2> A;
  A << 0.68,0.597, -0.211,0.823, 0.566,-0.605;
  std::cout << "Here is the matrix A:\n" << A << std::endl;
  Eigen::Matrix<double, 3, 1> b;
  b << -0.33, 0.536, -0.444;
  std::cout << "Here is the right hand side b:\n" << b << std::endl;
  auto sln = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);
  std::cout << "The least-squares solution is:\n" << sln << std::endl;
}