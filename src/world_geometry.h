// don't delete this comment or the whole program crashes, no joke... #2
#ifndef WORLD_GEOMETRY_H_
#define WORLD_GEOMETRY_H_

#include "TGeoManager.h"
#include "core/container/math_array.h"

namespace bdm {

struct WorldGeometry {
  void CreateCentreBox();
};

class SphereObstacle {
 public:
  SphereObstacle(Double3 centre, double radius) {
    centre_ = centre;
    radius_ = radius;
  }

  Double3 centre_;
  double radius_;
};

}  // namespace bdm

#endif  // WORLD_GEOMETRY_H_
