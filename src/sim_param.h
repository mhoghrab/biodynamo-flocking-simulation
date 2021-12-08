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
  uint64_t simulation_steps;
  bool obstacles_obstruct_view = true;

  // Flocking 2 Algorithm
  double c_a_1;
  double c_a_2;
  double c_a_3;
  double c_b_1;
  double c_b_2;
  double c_y;
  double eps;
  double h_a;
  double h_b;

  double d_t;

  std::string test_setup = "flocking";

  bool export_distances = false;

  // wind field stuff
  std::string wind_field_path_npy = "pywind/data/wind.npy";
  bool apply_wind_field = false;
  double c_wind_turb;
  double c_wind_mean;
  double c_wind_force;
};

}  // namespace bdm

#endif  // SIM_PARAM_H_
