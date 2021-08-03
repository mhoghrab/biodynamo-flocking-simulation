#ifndef FLOCKING_SIMULATION_H_
#define FLOCKING_SIMULATION_H_

#include "biodynamo.h"
#include "core/container/math_array.h"
#include "core/environment/environment.h"
#include "boid.h"
#include "behavior.h"

namespace bdm {

inline int Simulate(int argc, const char** argv) {
  Simulation simulation(argc, argv);
  // References
  auto* rm = simulation.GetResourceManager();
  auto* random = simulation.GetRandom();
  auto* param = simulation.GetParam();

  // Create n_boids boids uniformly distributed in 3D space with a random staring velocity
  size_t n_boids = 400;
  double x_coord, y_coord, z_coord;
  double x_vel, y_vel, z_vel;

  for (size_t i = 0; i < n_boids; ++i) {
    x_coord = random->Uniform(param->min_bound, param->max_bound);
    y_coord = random->Uniform(param->min_bound, param->max_bound);
    z_coord = random->Uniform(param->min_bound, param->max_bound);

    int vel_bound = 15;
    x_vel = random->Uniform(-vel_bound, vel_bound);
    y_vel = random->Uniform(-vel_bound, vel_bound);
    z_vel = random->Uniform(-vel_bound, vel_bound);

    auto* boid = new Boid({x_coord, y_coord, z_coord});

    //boid->SetTempPosition({x_coord, y_coord, z_coord});
    boid->SetPerceptionRadius(150);
    boid->SetActualDiameter(15);
    boid->SetVelocity({x_vel, y_vel, z_vel});
    boid->AddBehavior(new Flocking());  

    rm->AddAgent(boid); 
  }

  // Run simulation for one timestep
  simulation.GetScheduler()->Simulate(400);

  std::cout << "Simulation completed successfully!" << std::endl;
  return 0;
};

}  // namespace bdm


#endif  // FLOCKING_SIMULATION_H_
