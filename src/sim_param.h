#ifndef SIM_PARAM_H_
#define SIM_PARAM_H_

#include "biodynamo.h"

namespace bdm {

// -----------------------------------------------------------------------------
// Parameters specific for this simulation
// -----------------------------------------------------------------------------
struct SimParam : public ParamGroup {
  BDM_PARAM_GROUP_HEADER(SimParam, 1);

  size_t n_boids;
  double actual_diameter;
  double perception_angle_deg;
  double boid_perception_radius;
  double boid_interaction_radius;
  double obstacle_perception_radius;
  double neighbor_distance;
  double obstacle_distance;
  double max_force;
  double max_speed;
  double min_speed;
  double crusing_speed;
  uint64_t simulation_steps;
  bool obstacles_obstruct_view = true;

  // Flocking 2 Algorithm
  double c_a_1 = 0.2;
  double c_a_2 = 0.1;
  double c_b_1 = 0.2;
  double c_b_2 = 0.4;
  double c_y = 0.1;
  double eps = 0.1;
  double h_a = 0.25;
  double h_b = 0.5;

  //
  std::string test_setup = "flocking";

  //
  double d_t = 0.05;
};

}  // namespace bdm

#endif  // SIM_PARAM_H_
