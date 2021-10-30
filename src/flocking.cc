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

  // Creating and initializing obstacles
  CreateSphereObstacles();
  //CreateCuboidObstacles();
  InitializeRootGeometry();

  // spawning boids
  size_t n_boids = sparam->n_boids;
  double x_coord, y_coord, z_coord;
  double x_vel = 0, y_vel = 0, z_vel = 0;

  for (size_t i = 0; i < n_boids; ++i) {
    auto* boid = new Boid();

    if (sparam->test_setup == "free_flocking") {
      double box_width = 50;
      x_coord = (param->max_bound - param->min_bound) / 2 +
                random->Uniform(0, box_width);
      y_coord = (param->max_bound - param->min_bound) / 2 +
                random->Uniform(0, box_width);
      z_coord = (param->max_bound - param->min_bound) / 2 +
                random->Uniform(0, box_width);

      boid->SetPosition({x_coord, y_coord, z_coord});
      boid->SetVelocity({x_vel, y_vel, z_vel});
      boid->AddBehavior(new FreeFlocking());
    } else {
      x_coord = random->Uniform(100, 300);
      y_coord = random->Uniform(850, 1150);
      z_coord = random->Uniform(850, 1150);

      boid->SetPosition({x_coord, y_coord, z_coord});
      boid->SetVelocity({x_vel, y_vel, z_vel});
      boid->AddBehavior(new Flocking2());
    }

    boid->InitializeMembers();
    rm->AddAgent(boid);
  }

  // add PostScheduledOp to update Boid Data ftaer each timestep
  OperationRegistry::GetInstance()->AddOperationImpl(
      "UpdateOp", OpComputeTarget::kCpu, new UpdateOp());
  auto* update_op = NewOperation("UpdateOp");
  scheduler->ScheduleOp(update_op, OpType::kPostSchedule);

  // PostScheduledOp that calculattes the avg distance for a boid to it's
  // neighbors at each timestep and saves it in boid->avg_dist_r_i_
  OperationRegistry::GetInstance()->AddOperationImpl(
      "FlockingAnalysisOP", OpComputeTarget::kCpu, new FlockingAnalysisOP());
  auto* flocking_analysis_op = NewOperation("FlockingAnalysisOP");
  scheduler->ScheduleOp(flocking_analysis_op, OpType::kPostSchedule);

  // Run simulation
  scheduler->Simulate(sparam->simulation_steps);

  // ---------------------------------------------------------------------------
  // to export the avg distance for a boid to it's neighbors at each timestep we
  // saved them in a vector for each boid
  // now this vector get's exported as a line in output/avg_dist_r_i.csv
  // the csv will have the boids as a line an the distance in each timestep as
  // the column
  std::remove("output/avg_dist_r_i.csv");

  rm->ForEachAgent([](Agent* agent) {
    auto* boid = dynamic_cast<Boid*>(agent);

    std::ofstream myFile("output/avg_dist_r_i.csv", std::ios::app);

    for (size_t i = 0; i < boid->avg_dist_r_i_.size(); i++) {
      if (boid->avg_dist_r_i_[i] == -1)
        // "-1" means there are no neighbors, so no actual value to be saved
        myFile << ""
               << ",";
      else
        myFile << boid->avg_dist_r_i_[i] << ",";
    }
    myFile << "\n";
    myFile.close();
  });
  // ---------------------------------------------------------------------------

  std::cout << "Simulation completed successfully!" << std::endl;
  return 0;
}

}  // namespace bdm
