#ifndef SIM_PARAM_H_
#define SIM_PARAM_H_

#include "biodynamo.h"

namespace bdm {

// -----------------------------------------------------------------------------
// Parameters specific for this simulation
// -----------------------------------------------------------------------------
struct SimParam : public ParamGroup {
  BDM_PARAM_GROUP_HEADER(SimParam, 1);

  size_t n_boids = 1000;
  double actual_diameter = 15;
  double perception_radius = 300;
  double perception_angle_deg = 120;
  double obst_avoid_dist = 150;
  double neighbor_distance = 10;
  double obstacle_distance = 40;
  double max_force = 3;
  double max_speed = 15;
  double cruising_speed = 12;
  double min_speed = 0;
  double cohesion_weight = 1;
  double alignment_weight = 2;
  double separation_weight = 1.5;
  double avoid_domain_boundary_weight = 25;
  double obstacle_avoidance_weight = 5;
  uint64_t simulation_steps = 10;
};

}  // namespace bdm

#endif  // SIM_PARAM_H_
