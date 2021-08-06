#ifndef DIRECTION_ARRAY_H_
#define DIRECTION_ARRAY_H_

#include <math.h>
#include "core/container/math_array.h"

namespace bdm {

class DirectionArray {
  // https://github.com/SebLague/Boids/blob/master/Assets/Scripts/BoidHelper.cs
  const static int n = 200;

 public:
  std::array<Double3, n> directions;

  DirectionArray() {
    double goldenRatio = (1 + sqrt(5)) / 2;
    double angleIncrement = M_PI * 2 * goldenRatio;

    for (int i = 0; i < n; i++) {
      double t = (double)i / n;
      double inclination = acos(1 - 2 * t);
      double azimuth = angleIncrement * i;

      double x = sin(inclination) * cos(azimuth);
      double y = sin(inclination) * sin(azimuth);
      double z = cos(inclination);
      Double3 Ray = {x, y, z};
      directions[i] = Ray;
    }
  }

  std::array<Double3, n> GetAlignedDirections(Double3 velocityDirection) {
    // align directions[0] with velocityDirection and rotate all other Rays with
    // same parameters

    // crossprodukt directions[0] x velocityDirection
    Double3 axis = {directions[0][1] * velocityDirection[2] -
                        directions[0][2] * velocityDirection[1],
                    directions[0][2] * velocityDirection[0] -
                        directions[0][0] * velocityDirection[2],
                    directions[0][0] * velocityDirection[1] -
                        directions[0][1] * velocityDirection[0]};

    const double cosA = directions[0] * velocityDirection;
    const double k = 1 / (1 + cosA);

    // Alignment Matrix with coli als columns
    Double3 col1 = {(axis[0] * axis[0] * k) + cosA,
                    (axis[1] * axis[0] * k) - axis[2],
                    (axis[2] * axis[0] * k) + axis[1]};
    Double3 col2 = {(axis[0] * axis[1] * k) + axis[2],
                    (axis[1] * axis[1] * k) + cosA,
                    (axis[2] * axis[1] * k) - axis[0]};
    Double3 col3 = {(axis[0] * axis[2] * k) - axis[1],
                    (axis[1] * axis[2] * k) + axis[0],
                    (axis[2] * axis[2] * k) + cosA};

    // Alignment Matrix * directions[i]
    std::array<Double3, n> directions_aligned;
    for (int i = 0; i < n; i++) {
      directions_aligned[i] = col1 * directions[0][0] +
                              col2 * directions[0][1] + col3 * directions[0][2];
    }

    return directions_aligned;
  }
};

}  // namespace bdm

#endif  // DIRECTION_ARRAY_H_
