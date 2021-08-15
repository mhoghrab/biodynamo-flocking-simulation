#ifndef FLOCKING_SIMULATION_H_
#define FLOCKING_SIMULATION_H_

#include "UpdateOp.h"
#include "behavior.h"
#include "biodynamo.h"
#include "boid.h"
#include "core/container/math_array.h"
#include "core/environment/environment.h"
#include "core/operation/operation.h"
#include "core/operation/operation_registry.h"

namespace bdm {

///////////////////////////////////////////////////////////////////////////////
// Parameters specific for this simulation
///////////////////////////////////////////////////////////////////////////////
struct SimParam : public ParamGroup {
  BDM_PARAM_GROUP_HEADER(SimParam, 1);

  double actualDiameter_ = 15, perceptionRadius_ = 150,
         perceptionAngle_ = (3 * M_PI) / 5;
  double maxForce_ = 3, maxSpeed_ = 15, crusingSpeed = 12, minSpeed_ = 8;
  double cohesionWeight = 1, alignmentWeight = 2, seperationWeight = 1.5,
         avoidDomainBoundaryWeight = 25, obstacleAvoidanceWeight = 5;
};

///////////////////////////////////////////////////////////////////////////////
// Simulation
///////////////////////////////////////////////////////////////////////////////
inline int Simulate(int argc, const char** argv) {
  Param::RegisterParamGroup(new SimParam());
  Simulation simulation(argc, argv);
  auto* rm = simulation.GetResourceManager();
  auto* random = simulation.GetRandom();
  auto* param = simulation.GetParam();
  auto* scheduler = simulation.GetScheduler();

  // Create n_boids boids uniformly distributed in 3D space with a random
  // staring velocity
  size_t n_boids = 2000;
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

    boid->SetVelocity({x_vel, y_vel, z_vel});
    boid->Initialize();  // this sets newPosition and newVelocity, so do after
                         // setting initial position and velocity
    boid->AddBehavior(new Flocking());

    rm->AddAgent(boid);
  }

  // add PostScheduledOp to set the actual position/velocity to the calculated
  // newPosition/newVelocity
  OperationRegistry::GetInstance()->AddOperationImpl(
      "UpdateOp", OpComputeTarget::kCpu, new UpdateOp());
  auto* update_op = NewOperation("UpdateOp");
  scheduler->ScheduleOp(update_op, OpType::kPostSchedule);

  // Run simulation
  simulation.GetScheduler()->Simulate(400);

  std::cout << "Simulation completed successfully!" << std::endl;
  return 0;
};

}  // namespace bdm

#endif  // FLOCKING_SIMULATION_H_
