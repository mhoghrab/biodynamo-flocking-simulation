#ifndef SIM_PARAM_H_
#define SIM_PARAM_H_

#include "biodynamo.h"

namespace bdm {

// -----------------------------------------------------------------------------
// Parameters specific for this simulation
// -----------------------------------------------------------------------------
struct SimParam : public ParamGroup {
  BDM_PARAM_GROUP_HEADER(SimParam, 1);

  size_t n_boids = 100;
  double actual_diameter = 15;
  double perception_radius = 300;
  double obstacle_perception_radius = 150;
  double perception_angle_deg = 120;
  double obst_avoid_dist = 150;
  double neighbor_distance = 10;
  double obstacle_distance = 80;
  double max_force = 3;
  double max_speed = 15;
  double crusing_speed = 12;
  double min_speed = 0;
  double cohesion_weight = 1;
  double alignment_weight = 2;
  double seperation_weight = 1.5;
  double avoid_domain_boundary_weight = 25;
  double obstacle_avoidance_weight = 5;
  uint64_t simulation_steps = 10;
  bool obstacles_obstruct_view = true;

  // Flocking 2 Algorithm
  double c_a_1 = 1;
  double c_a_2 = 2 * sqrt(c_a_1);
  double c_b_1 = 3;
  double c_b_2 = 2 * sqrt(c_b_1);
  double eps = 0.1;
  double h_a = 0.2;
  double h_b = 0.5;

  //
  double d_t = 0.1;
};

}  // namespace bdm

#endif  // SIM_PARAM_H_
