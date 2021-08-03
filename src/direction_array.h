
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
    float goldenRatio = (1 + sqrt(5)) / 2;
    float angleIncrement = M_PI * 2 * goldenRatio;

    for (int i = 0; i < n; i++) {
      float t = (float) i / n;
      float inclination = acos(1 - 2 * t);
      float azimuth = angleIncrement * i;

      float x = sin(inclination) * cos(azimuth);
      float y = sin(inclination) * sin(azimuth);
      float z = cos(inclination);

      directions[i][0] = x;
      directions[i][1] = y;
      directions[i][2] = z;
    }
  }
};

}  // namespace bdm

#endif  // DIRECTION_ARRAY_H_
