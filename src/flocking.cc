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
  // ---------------------------------------------------------------------------
  // set simulation references
  Param::RegisterParamGroup(new SimParam());
  auto set_param = [&](Param* param) {};
  Simulation simulation(argc, argv, set_param);
  auto* rm = simulation.GetResourceManager();
  auto* random = simulation.GetRandom();
  auto* param = simulation.GetParam();
  auto* sparam = param->Get<SimParam>();
  auto* scheduler = simulation.GetScheduler();

  // ---------------------------------------------------------------------------
  // initialization of static Boid::wind_data reference
  // if (sparam->apply_wind_field) {
  //   InitializeWindField();
  // }

  // ---------------------------------------------------------------------------
  // Creating and initializing obstacles
  if (!(sparam->test_setup == "free flocking")) {
    // CreateSphereObstacles();
    // CreateCuboidObstacles();
    // CreateWallObstacle();
    // CreateObstacleSetup_0();
  }

  InitializeRootGeometry();

  // ---------------------------------------------------------------------------
  // creating, initializing and adding agent to simulation
  double x_vel, y_vel, z_vel;
  double centre = (param->max_bound + param->min_bound) / 2;

  for (size_t i = 0; i < sparam->n_boids; ++i) {
    auto* boid = new Boid();

    if (sparam->test_setup == "free flocking") {
      double radius = 150;
      Double3 transl = {centre, centre, centre};
      Double3 coord = GetRandomVectorInUnitSphere() * radius + transl;

      boid->SetPosition(coord);
      boid->SetVelocity({x_vel, y_vel, z_vel});
      boid->SetHeadingDirection(GetRandomVectorInUnitSphere());
      boid->AddBehavior(new FreeFlocking());
    } else {
      double radius = 100;
      Double3 transl = {param->min_bound + 300, centre, centre};
      Double3 coord = GetRandomVectorInUnitSphere() * radius + transl;

      boid->SetPosition(coord);
      boid->SetVelocity({x_vel, y_vel, z_vel});
      boid->SetHeadingDirection(GetRandomVectorInUnitSphere());
      boid->AddBehavior(new Flocking());
    }

    boid->InitializeMembers();
    rm->AddAgent(boid);
  }

  // ---------------------------------------------------------------------------
  // add PostScheduledOp to update Boid Data after each timestep
  OperationRegistry::GetInstance()->AddOperationImpl(
      "UpdateOp", OpComputeTarget::kCpu, new UpdateOp());
  auto* update_op = NewOperation("UpdateOp");

  scheduler->ScheduleOp(update_op, OpType::kPostSchedule);

  // PostScheduledOp that calculates the avg distance for a boid to it's
  // neighbors at each timestep and saves it in boid->avg_dist_r_i_
  if (sparam->export_distances) {
    OperationRegistry::GetInstance()->AddOperationImpl(
        "FlockingAnalysisOP", OpComputeTarget::kCpu, new FlockingAnalysisOP());
    auto* flocking_analysis_op = NewOperation("FlockingAnalysisOP");
    scheduler->ScheduleOp(flocking_analysis_op, OpType::kPostSchedule);
  }

  // ---------------------------------------------------------------------------
  scheduler->Simulate(sparam->simulation_steps);

  // ---------------------------------------------------------------------------
  // end of simulation
  std::cout << "Simulation completed successfully!" << std::endl;

  // ---------------------------------------------------------------------------
  // to export the avg distance for a boid to it's neighbors at each
  // timestep we saved them in a vector for each boid
  //  this vector get's exported as a line in output/avg_dist_r_i.csv
  // the csv will have the boids as a line an the distance in each
  // timestep as the column

  // 2do nicht so beschissen alles machen
  // 500, 20 dynmaisch über param bestimmen

  if (sparam->export_distances) {
    std::remove("output/avg_dist.csv");

    rm->ForEachAgent([](Agent* agent) {
      auto* boid = dynamic_cast<Boid*>(agent);

      std::ofstream myFile("output/avg_dist.csv", std::ios::app);

      for (size_t i = 0; i < 3000; i++) {
        size_t idx = 20 * i;
        if (boid->avg_dist_r_i_[idx] == -1)
          // "-1" means there are no neighbors, so no actual value to be
          // saved
          // but we save 0 and filter them out later in mathlab for each
          // timestep
          myFile << "0"
                 << ",";
        else
          myFile << boid->avg_dist_r_i_[idx] << ",";
      }
      myFile << "\n";
      myFile.close();
    });
  }

  // ---------------------------------------------------------------------------
  return 0;
}

}  // namespace bdm
