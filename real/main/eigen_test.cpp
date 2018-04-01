// Example By: RandomVibe
// Eigen Doc: http://eigen.tuxfamily.org/dox/
// Quick Reference: http://eigen.tuxfamily.org/dox/QuickRefPage.html

#include <common/Eigen/Eigen.h>     // Calls main Eigen matrix class library
#include <common/Eigen/Eigen/Dense>             // Calls inverse, determinant, LU decomp., etc.
using namespace Eigen;    // Eigen related statement; simplifies syntax for declaration of matrices

#include <Arduino.h>
#include <real/RealMouse.h>

void print_mtx(const Eigen::Matrix<double, Dynamic, Dynamic> &X);

void setup() {

   Serial.begin(9600);
   delay(1000);

  Eigen::Matrix<double, 3, 2> A;
  A << 0.68, 0.597,
      -0.211, 0.823,
       0.566, -0.605;
  print("Here is the matrix A:\n");
  print_mtx(A);
  Eigen::Matrix<double, 3, 1> b;
  b << -0.33,
       0.536,
       -0.444;
  print("Here is the right hand side b:\n");
  print_mtx(b);
  auto sln = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);
  print("The least-squares solution is:\n");
  print_mtx(sln);
}

void loop() {
  RealMouse::checkVoltage();
}


void print_mtx(const Eigen::Matrix<double, Dynamic, Dynamic> &X)
{
  int i, j, nrow, ncol;

  nrow = X.rows();
  ncol = X.cols();

  print("nrow: %i, ncol: %i\n", nrow, ncol);

  for (i=0; i<nrow; i++)
  {
    for (j=0; j<ncol; j++)
    {
      print("%0.4f, ", X(i,j));
    }
    print("\n");
  }
  print("\n");
}
