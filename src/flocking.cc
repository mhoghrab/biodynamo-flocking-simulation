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
  // set references
  Param::RegisterParamGroup(new SimParam());
  auto set_param = [&](Param* param) {};
  Simulation simulation(argc, argv, set_param);
  auto* rm = simulation.GetResourceManager();
  auto* param = simulation.GetParam();
  auto* sparam = param->Get<SimParam>();
  auto* scheduler = simulation.GetScheduler();
  std::string simulation_setup = sparam->simulation_setup;

  // ---------------------------------------------------------------------------
  // creating and initializing obstacles
  if (simulation_setup == "obstacle_spherical")
    CreateSphereObstacles();
  else if (simulation_setup == "obstacle_cuboid")
    CreateCuboidObstacles();
  else if (simulation_setup == "obstacle_wall")
    CreateWallObstacle();

  // always initialize root geometry, even when no actual obstacles are aded
  InitializeRootGeometry();

  // ---------------------------------------------------------------------------
  // creating, initializing and adding agent to simulation
  double centre = (param->max_bound + param->min_bound) / 2;
  double radius = sparam->starting_sphere_radius;

  if (simulation_setup == "free_space") {
    Double3 transl = {centre, centre, centre};
    for (size_t i = 0; i < sparam->n_boids; ++i) {
      auto* boid = new Boid();
      Double3 coord = GetRandomVectorInUnitSphere() * radius + transl;

      boid->SetPosition(coord);
      boid->SetVelocity({0, 0, 0});
      boid->SetHeadingDirection(GetRandomVectorInUnitSphere());
      boid->AddBehavior(new FreeSpaceFlocking());
      boid->InitializeMembers();
      rm->AddAgent(boid);
    }
  } else if (simulation_setup == "free_space_avg_dist") {
    for (size_t i = 0; i < sparam->n_boids; ++i) {
      auto* boid = new Boid();
      Double3 coord = GetRandomVectorInUnitSphere() * radius + centre;

      boid->SetPosition(coord);
      boid->SetVelocity({0, 0, 0});
      boid->SetHeadingDirection(GetRandomVectorInUnitSphere());
      boid->AddBehavior(new FreeSpaceFlocking());
      boid->InitializeMembers();
      rm->AddAgent(boid);
    }
  } else if (simulation_setup == "obstacle_spherical" ||
             simulation_setup == "obstacle_cuboid" ||
             simulation_setup == "obstacle_wall") {
    Double3 transl;
    if (simulation_setup == "obstacle_wall") {
      transl = {-750, centre, centre};
    } else {
      transl = {-500, centre, centre};
    }

    for (size_t i = 0; i < sparam->n_boids; ++i) {
      auto* boid = new Boid();
      Double3 coord = GetRandomVectorInUnitSphere() * radius + transl;

      boid->SetPosition(coord);
      boid->SetVelocity({0, 0, 0});
      boid->SetHeadingDirection(GetRandomVectorInUnitSphere());
      boid->AddBehavior(new Flocking());
      boid->InitializeMembers();
      rm->AddAgent(boid);
    }
  } else if (simulation_setup == "wind") {
    for (size_t i = 0; i < sparam->n_boids; ++i) {
      auto* boid = new Boid();
      Double3 transl = {param->min_bound + radius * 2.5, centre, centre};
      Double3 coord = GetRandomVectorInUnitSphere() * radius + transl;

      boid->SetPosition(coord);
      boid->SetVelocity({0, 0, 0});
      boid->SetHeadingDirection({1, 0, 0});
      boid->AddBehavior(new FreeSpaceFlocking());
      boid->InitializeMembers();
      rm->AddAgent(boid);
    }
  } else if (simulation_setup == "x_vel_test") {
    for (size_t i = 0; i < sparam->n_boids; ++i) {
      auto* boid = new Boid();
      Double3 coord = GetRandomVectorInUnitSphere() * radius + centre;

      boid->SetPosition(coord);
      boid->SetVelocity({5, 0, 0});
      boid->SetHeadingDirection(GetRandomVectorInUnitSphere());
      boid->InitializeMembers();
      rm->AddAgent(boid);
    }
  } else if (simulation_setup == "wind_deacc") {
    auto* boid = new Boid();

    boid->SetPosition({0, 0, 0});
    boid->SetVelocity({5, 0, 0});
    boid->InitializeMembers();
    rm->AddAgent(boid);

    // save position - velocity data of boid
    OperationRegistry::GetInstance()->AddOperationImpl(
        "SavePosVelOP", OpComputeTarget::kCpu, new SavePosVelOP());
    auto* op = NewOperation("SavePosVelOP");
    scheduler->ScheduleOp(op, OpType::kPostSchedule);
  } else {
    Double3 transl = {centre, centre, centre};

    for (size_t i = 0; i < sparam->n_boids; ++i) {
      auto* boid = new Boid();
      Double3 coord = GetRandomVectorInUnitSphere() * radius + transl;

      boid->SetPosition(coord);
      boid->SetVelocity({0, 0, 0});
      boid->SetHeadingDirection(GetRandomVectorInUnitSphere());
      boid->AddBehavior(new Flocking());
      boid->InitializeMembers();
      rm->AddAgent(boid);
    }
  }

  // ---------------------------------------------------------------------------
  // add PostScheduledOps

  // add PostScheduledOp to update Boid Data after each timestep
  OperationRegistry::GetInstance()->AddOperationImpl(
      "UpdateOp", OpComputeTarget::kCpu, new UpdateOp());
  auto* update_op = NewOperation("UpdateOp");
  scheduler->ScheduleOp(update_op, OpType::kPostSchedule);

  // PostScheduledOp that calculates the avg distance for a boid to it's
  // neighbors at each simulation step and saves it in boid->avg_dist_r_i_
  if (sparam->export_distances) {
    OperationRegistry::GetInstance()->AddOperationImpl(
        "SaveSaveAvgDistOP", OpComputeTarget::kCpu, new SaveAvgDistOP());
    auto* op = NewOperation("SaveAvgDistOP");
    scheduler->ScheduleOp(op, OpType::kPostSchedule);
  }

  // PostScheduledOp that saves the velocity of an agent at each simulation
  // step
  if (sparam->export_velocity) {
    OperationRegistry::GetInstance()->AddOperationImpl(
        "SaveVelOP", OpComputeTarget::kCpu, new SaveVelOP());
    auto* op = NewOperation("SaveVelOP");
    scheduler->ScheduleOp(op, OpType::kPostSchedule);
  }

  // ---------------------------------------------------------------------------
  scheduler->Simulate(sparam->computational_steps);

  // ---------------------------------------------------------------------------
  // to export the avg distance for a boid to it's neighbors at each
  // timestep we saved them in a vector for each boid
  // this vector get's exported as a line in output/avg_dist_r_i.csv
  // the csv will have the boids as a line an the distance in each
  // simulation step as the column
  // 2do nicht so blöd alles machen ;P

  if (sparam->export_distances) {
    std::remove("output/data/avg_dist.csv");

    rm->ForEachAgent([](Agent* agent) {
      auto* boid = dynamic_cast<Boid*>(agent);
      const auto* sparam =
          bdm::Simulation::GetActive()->GetParam()->Get<SimParam>();
      int f = (int)(1 / sparam->d_t);
      int n_sim_steps = (int)(sparam->computational_steps / f);

      std::ofstream myFile("output/data/avg_dist.csv", std::ios::app);

      for (int i = 0; i < n_sim_steps; i++) {
        int idx = i * 20;
        myFile << boid->avg_dist_r_i_[idx] << ",";
      }
      myFile << "\n";
      myFile.close();
    });
  }

  if (sparam->export_velocity) {
    std::remove("output/data/vel_data.csv");

    rm->ForEachAgent([](Agent* agent) {
      auto* boid = dynamic_cast<Boid*>(agent);
      const auto* sparam =
          bdm::Simulation::GetActive()->GetParam()->Get<SimParam>();
      int f = (int)(1 / sparam->d_t);
      int n_sim_steps = (int)(sparam->computational_steps / f);

      std::ofstream myFile("output/data/vel_data.csv", std::ios::app);

      for (int i = 0; i < n_sim_steps; i++) {
        int idx = i * 20;
        myFile << boid->vel_data_[idx] << ",";
      }
      myFile << "\n";
      myFile.close();
    });
  }

  if (simulation_setup == "wind_deacc") {
    std::remove("output/data/wind_deacc.csv");

    rm->ForEachAgent([](Agent* agent) {
      auto* boid = dynamic_cast<Boid*>(agent);
      const auto* sparam =
          bdm::Simulation::GetActive()->GetParam()->Get<SimParam>();
      int f = (int)(1 / sparam->d_t);
      int n_sim_steps = (int)(sparam->computational_steps / f);

      std::ofstream myFile("output/data/wind_deacc.csv", std::ios::app);

      for (int i = 0; i < n_sim_steps; i++) {
        int idx = i * 20;
        myFile << boid->vel_data_[idx] << ",";
      }
      myFile << "\n";
      for (int i = 0; i < n_sim_steps; i++) {
        int idx = i * 20;
        myFile << boid->pos_data_[idx] << ",";
      }

      myFile.close();
    });
  }

  // ---------------------------------------------------------------------------
  // end of simulation
  std::cout << "Simulation completed successfully!" << std::endl;
  return 0;
}

}  // namespace bdm
