// don't delete this comment or the whole program crashes, no joke... #2
// might be fixed by now but let's not risk it ;P
#ifndef WORLD_GEOMETRY_H_
#define WORLD_GEOMETRY_H_

#include "TGeoManager.h"
#include "core/container/math_array.h"

namespace bdm {

void CreateRootObstacles();

////////////////////////////////////////////////////////////////////////////////
// Sphere and Cuboid Obstacles
////////////////////////////////////////////////////////////////////////////////

void InitializeRootGeometry();

void CreateObstacleSetup_0();

void CreateWallObstacle();

// ---------------------------------------------------------------------------

struct SphereObstacle {
  SphereObstacle(Double3 centre, double radius);

  Double3 centre_;
  double radius_;

 public:
  static std::vector<SphereObstacle> sphere_obstacles;
};

void CreateSphereObstacles();

// ---------------------------------------------------------------------------

struct CuboidObstacle {
  CuboidObstacle(Double3 lower_bound, Double3 upper_bound);

  Double3 lower_bound_, upper_bound_;

 public:
  static std::vector<CuboidObstacle> cuboid_obstacles;
};

void CreateCuboidObstacles();

}  // namespace bdm

#endif  // WORLD_GEOMETRY_H_
