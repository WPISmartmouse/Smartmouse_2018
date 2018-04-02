#include <math.h>
#include <limits>

namespace smartmouse {
namespace math {
/** \brief Computes the signed shorted angle between y2 and y1. Check CommonTest.cpp to see examples
 *
 * @param y1 the second angle in the subtraction
 * @param y2 the first angle in the subtraction
 * @return the signed shorted angle between y2 and y1.
 */
inline double yaw_diff(double y1, double y2) {
  double diff = y2 - y1;
  if (diff > M_PI) return diff - M_PI * 2;
  if (diff < -M_PI) return diff + M_PI * 2;
  return diff;
}

// SOURCE: https://stackoverflow.com/questions/4633177/c-how-to-wrap-a-float-to-the-interval-pi-pi
// Floating-point modulo
// The result (the remainder) has same sign as the divisor.
// Similar to matlab's mod(); Not similar to fmod() -   Mod(-3,4)= 1   fmod(-3,4)= -3
template<typename T>
T MyMod(T x, T y)
{
  static_assert(!std::numeric_limits<T>::is_exact , "Mod: floating-point type expected");

  if (0. == y)
    return x;

  double m= x - y * floor(x/y);

  // handle boundary cases resulted from floating-point cut off:

  if (y > 0)              // modulo range: [0..y)
  {
    if (m>=y)           // Mod(-1e-16             , 360.    ): m= 360.
      return 0;

    if (m<0 )
    {
      if (y+m == y)
        return 0  ; // just in case...
      else
        return y+m; // Mod(106.81415022205296 , _TWO_PI ): m= -1.421e-14
    }
  }
  else                    // modulo range: (y..0]
  {
    if (m<=y)           // Mod(1e-16              , -360.   ): m= -360.
      return 0;

    if (m>0 )
    {
      if (y+m == y)
        return 0  ; // just in case...
      else
        return y+m; // Mod(-106.81415022205296, -_TWO_PI): m= 1.421e-14
    }
  }

  return m;
}

// wrap [rad] angle to [-PI..PI)
inline double wrapAngleRad(double angle_rad)
{
  return MyMod(angle_rad + M_PI, 2*M_PI) - M_PI;
}

inline void wrapAngleRadInPlace(double *angle_rad)
{
  *angle_rad = MyMod(*angle_rad + M_PI, 2*M_PI) - M_PI;
}

constexpr double rad_to_deg(double rad) {
  return rad * 180 / (2 * M_PI);
}

} // math
} // smartmouse
