#ifndef FLOCKING_SIMULATION_H_
#define FLOCKING_SIMULATION_H_

#include "behavior.h"
#include "biodynamo.h"
#include "boid.h"
#include "core/container/math_array.h"
#include "core/environment/environment.h"
//#include "parameters.h"

namespace bdm {

inline int Simulate(int argc, const char** argv) {
  // Param::RegisterParamGroup(new SimParam());
  Simulation simulation(argc, argv);
  auto* rm = simulation.GetResourceManager();
  auto* random = simulation.GetRandom();
  auto* param = simulation.GetParam();
  // auto* sparam = param->Get<SimParam>();

  // Create n_boids boids uniformly distributed in 3D space with a random
  // staring velocity
  size_t n_boids = 600;
  double x_coord, y_coord, z_coord;
  double x_vel, y_vel, z_vel;

  for (size_t i = 0; i < n_boids; ++i) {
    x_coord = random->Uniform(param->min_bound, param->max_bound);
    y_coord = random->Uniform(param->min_bound, param->max_bound);
    z_coord = random->Uniform(param->min_bound, param->max_bound);

    int vel_bound = 5;
    x_vel = random->Uniform(-vel_bound, vel_bound);
    y_vel = random->Uniform(-vel_bound, vel_bound);
    z_vel = random->Uniform(-vel_bound, vel_bound);

    auto* boid = new Boid({x_coord, y_coord, z_coord});

    boid->SetPerceptionRadius(150);
    boid->SetActualDiameter(15);
    boid->SetVelocity({x_vel, y_vel, z_vel});
    boid->AddBehavior(new Flocking());

    rm->AddAgent(boid);
  }

  // Run simulation
  simulation.GetScheduler()->Simulate(400);

  std::cout << "Simulation completed successfully!" << std::endl;
  return 0;
};

}  // namespace bdm

#endif  // FLOCKING_SIMULATION_H_
