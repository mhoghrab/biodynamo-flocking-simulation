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

      // add wind force
      const auto* sparam =
          bdm::Simulation::GetActive()->GetParam()->Get<SimParam>();

      if (sparam->apply_wind_field) {
        boid->acceleration_ += boid->CalculateWindForce();
      }

      boid->UpdateNewVelocity();
      boid->UpdateNewPosition();
      boid->UpdateData();

      boid->ResetAcceleration();
    });
  }
};

struct FlockingAnalysisOP : public StandaloneOperationImpl {
  BDM_OP_HEADER(FlockingAnalysisOP);
  void operator()() override {
    auto* sim = Simulation::GetActive();
    auto* rm = sim->GetResourceManager();

    rm->ForEachAgent([](Agent* agent) {
      auto* boid = dynamic_cast<Boid*>(agent);
      auto* ctxt = Simulation::GetActive()->GetExecutionContext();
      FlockingNeighborAnalysis Data(boid);
      ctxt->ForEachNeighbor(Data, *boid, pow(boid->boid_perception_radius_, 2));

      // save the avg distance in avg_dist_r_i_
      (boid->avg_dist_r_i_).push_back(Data.GetAvgDist_InteractionR());
    });
  }
};

#endif  // UPDATE_OP_H_