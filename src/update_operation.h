#ifndef UPDATE_OP_H_
#define UPDATE_OP_H_

#include "boid.h"
#include "core/operation/operation.h"
#include "core/operation/operation_registry.h"
#include "core/resource_manager.h"

using namespace bdm;

struct UpdateOp : public StandaloneOperationImpl {
  BDM_OP_HEADER(UpdateOp);
  void operator()() override {
    auto* sim = Simulation::GetActive();
    auto* rm = sim->GetResourceManager();

    rm->ForEachAgent([](Agent* agent) {
      auto* boid = dynamic_cast<Boid*>(agent);

      boid->UpdateNewVelocity();
      boid->UpdateNewPosition();
      boid->UpdateData();

      boid->ResetAcceleration();
    });
  }
};

#endif  // UPDATE_OP_H_