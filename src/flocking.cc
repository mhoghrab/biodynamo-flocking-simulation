// ---------------------------------------------------------------------------
//
// Copyright (C) 2021 CERN & Newcastle University for the benefit of the
// BioDynaMo collaboration. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// See the LICENSE file distributed with this work for details.
// See the NOTICE file distributed with this work for additional information
// regarding copyright ownership.
//
// ---------------------------------------------------------------------------
#ifndef FLOCKING_H_
#define FLOCKING_H_

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
  auto set_param = [&](Param* param) { param->statistics = true; };
  Simulation simulation(argc, argv, set_param);
  auto* rm = simulation.GetResourceManager();
  auto* random = simulation.GetRandom();
  auto* param = simulation.GetParam();
  auto* sparam = param->Get<SimParam>();
  auto* scheduler = simulation.GetScheduler();

  // Initializing the wold geometry / obstacles
  auto* worldgeo = new WorldGeometry();
  worldgeo->CreateCentreBox();

  // spawning boids
  size_t n_boids = sparam->n_boids;
  double x_coord, y_coord, z_coord;
  double x_vel, y_vel, z_vel;

  for (size_t i = 0; i < n_boids; ++i) {
    x_coord = random->Uniform(100, 300);
    y_coord = random->Uniform(900, 1100);
    z_coord = random->Uniform(900, 1100);

    // x_coord = random->Uniform(param->min_bound, param->max_bound);
    // y_coord = random->Uniform(param->min_bound, param->max_bound);
    // z_coord = random->Uniform(param->min_bound, param->max_bound);

    // x_coord = random->Uniform(500, 1000);
    // y_coord = random->Uniform(500, 1000);
    // z_coord = random->Uniform(0, 100);

    int vel_bound = 2;
    // x_vel = random->Uniform(-vel_bound, vel_bound);
    // y_vel = random->Uniform(-vel_bound, vel_bound);
    // z_vel = random->Uniform(-vel_bound, vel_bound);

    // z_vel = 10;
    // x_vel = random->Uniform(-vel_bound, vel_bound);
    // y_vel = random->Uniform(-vel_bound, vel_bound);

    x_vel = 10;
    y_vel = random->Uniform(-vel_bound, vel_bound);
    z_vel = random->Uniform(-vel_bound, vel_bound);

    auto* boid = new Boid({x_coord, y_coord, z_coord});
    boid->SetVelocity({x_vel, y_vel, z_vel});
    boid->InitializeMembers();
    boid->AddBehavior(new Flocking2());

    rm->AddAgent(boid);
  }

  // add PostScheduledOp to set the actual position / velocity to the calculated
  // newPosition / newVelocity
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

#endif  // FLOCKING_H_
