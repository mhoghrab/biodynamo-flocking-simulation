
#ifndef DIRECTION_ARRAY_H_
#define DIRECTION_ARRAY_H_

#include "core/container/math_array.h"
#include <math.h> 
namespace bdm {

class DirectionArray {
  // https://github.com/SebLague/Boids/blob/master/Assets/Scripts/BoidHelper.cs
  const static int n = 300;
  public:
  double directions[n][3];

  DirectionArray() {
    double goldenRatio = (1 + sqrt(5)) / 2;
    double angleIncrement = M_PI * 2 * goldenRatio;

    for (int i = 0; i < n; i++) {
      double t = (double) i / n;
      double inclination = acos(1 - 2 * t);
      double azimuth = angleIncrement * i;

      double x = sin(inclination) * cos(azimuth);
      double y = sin(inclination) * sin(azimuth);
      double z = cos(inclination);

      directions[i][0] = x;
      directions[i][1] = y;
      directions[i][2] = z;
    }
  }
};

}  // namespace bdm

#endif  // DIRECTION_ARRAY_H_
