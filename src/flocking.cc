#include "flocking.h"
#include "TGeoManager.h"
#include "biodynamo.h"
#include "boid.h"
#include "sim_param.h"
#include "update_operation.h"
#include "world_geometry.h"

namespace bdm {

const ParamGroupUid SimParam::kUid = ParamGroupUidGenerator::Get()->NewUid();

int Simulate(int argc, const char** argv) {
  Param::RegisterParamGroup(new SimParam());
  auto set_param = [&](Param* param) {};
  Simulation simulation(argc, argv, set_param);
  auto* rm = simulation.GetResourceManager();
  auto* random = simulation.GetRandom();
  auto* param = simulation.GetParam();
  auto* sparam = param->Get<SimParam>();
  auto* scheduler = simulation.GetScheduler();

  // Initializing the wold geometry / obstacles
  // CreateRootObstacles();
  CreateSphereObstacles();
  CreateCuboidObstacles();
  InitializeRootGeometry();

  // spawning boids
  size_t n_boids = sparam->n_boids;
  double x_coord, y_coord, z_coord;
  double x_vel, y_vel, z_vel;

  for (size_t i = 0; i < n_boids; ++i) {
    x_coord = random->Uniform(100, 300);
    y_coord = random->Uniform(850, 1150);
    z_coord = random->Uniform(850, 1150);
    // x_coord = random->Uniform(param->min_bound, param->max_bound);
    // y_coord = random->Uniform(param->min_bound, param->max_bound);
    // z_coord = random->Uniform(param->min_bound, param->max_bound);

    int vel_bound = 2;
    x_vel = 10;
    y_vel = random->Uniform(-vel_bound, vel_bound);
    z_vel = random->Uniform(-vel_bound, vel_bound);

    auto* boid = new Boid({x_coord, y_coord, z_coord});
    boid->SetVelocity({x_vel, y_vel, z_vel});
    boid->InitializeMembers();
    boid->AddBehavior(new Flocking2());

    rm->AddAgent(boid);
  }

  // add PostScheduledOp to update Boid Data ftaer each timestep
  OperationRegistry::GetInstance()->AddOperationImpl(
      "UpdateOp", OpComputeTarget::kCpu, new UpdateOp());
  auto* update_op = NewOperation("UpdateOp");
  scheduler->ScheduleOp(update_op, OpType::kPostSchedule);

  // Run simulation
  scheduler->Simulate(sparam->simulation_steps);

  std::cout << "Simulation completed successfully!" << std::endl;
  return 0;
}

}  // namespace bdm