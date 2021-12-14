#ifndef SIM_PARAM_H_
#define SIM_PARAM_H_

#include "biodynamo.h"
#include "core/container/math_array.h"

namespace bdm {

// -----------------------------------------------------------------------------
// Parameters specific for this simulation
// -----------------------------------------------------------------------------
struct SimParam : public ParamGroup {
  BDM_PARAM_GROUP_HEADER(SimParam, 1);

  size_t n_boids = 250;
  double starting_sphere_radius = 150;
  double actual_diameter = 10;
  double perception_angle_deg = 150;
  double boid_perception_radius = 250;
  double boid_interaction_radius = 70;
  double obstacle_perception_radius = 200;
  double neighbor_distance = 50;
  double obstacle_distance = 50;
  double max_accel = 0.4;
  double max_speed = 5;
  uint64_t computational_steps;
  bool obstacles_obstruct_view = true;
  bool limit_speed = true;

  // Flocking 2 Algorithm
  double c_a_1 = 0.37;
  double c_a_2 = 0.05;
  double c_a_3 = 0.05;
  double c_b_1 = 0.45;
  double c_b_2 = 0.1;
  double c_y = 0.05;
  double eps = 0.1;
  double h_a = 0.25;
  double h_b = 0.6;

  double d_t = 0.05;

  Double3 pos_gamma = {1000, 0, 0};
  Double3 wind_mean = {1, 0, 0};

  std::string simulation_setup;

  bool export_distances = false;
  bool export_velocity = false;

  // wind field stuff
  std::string wind_field_path_npy = "pywind/data/wind.npy";
  bool apply_wind_field = false;
  double c_wind_turb;
  double c_wind_mean;
  double c_wind_force;
};

}  // namespace bdm

#endif  // SIM_PARAM_H_
