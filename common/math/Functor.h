#pragma once

#include <common/Eigen/Eigen.h>

/** Generic functor
 * See http://eigen.tuxfamily.org/index.php?title=Functors
 * C++ version of a function pointer that stores meta-data about the function
 */
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
class Functor {
 public:

  /** Information that tells the caller the numeric type (eg. double) and size (input / output dim) **/
  typedef _Scalar Scalar;
  enum { // Required by numerical differentiation module
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };

  /** Tell the caller the matrix sizes associated with the input, output, and jacobian **/
  typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

  /** Local copy of the number of inputs **/
  int m_inputs, m_values;

  Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}

  Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

  // Get methods for users to determine function input and output dimensions
  int inputs() const { return m_inputs; }

  int values() const { return m_values; }

};
