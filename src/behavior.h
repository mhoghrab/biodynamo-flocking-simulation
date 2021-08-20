#ifndef BEHAVIOR_H_
#define BEHAVIOR_H_

#include <math.h>
#include "boid.h"
#include "core/behavior/behavior.h"
#include "core/functor.h"

namespace bdm {

///////////////////////////////////////////////////////////////////////////////
// Functor class needed to calculate neighbor data in ForEachNeighbor call
///////////////////////////////////////////////////////////////////////////////
class CalculateNeighborData : public Functor<void, Agent*, double> {
 public:
  CalculateNeighborData(Boid* self) : self_(self) {}
  virtual ~CalculateNeighborData() {}

  void operator()(Agent* neighbor, double squared_distance) override {
    auto* other = bdm_static_cast<const Boid*>(neighbor);
    Double3 other_pos = other->GetPosition();

    // check if neighbor is in viewing cone
    if (self_->CheckIfVisible(other_pos)) {
      Double3 other_vel = other->GetVelocity();
      Double3 diff_pos = boid_pos_ - other_pos;

      double dist = diff_pos.Norm();

      sum_pos_ += other_pos;
      sum_vel_ += other_vel;

      if (dist < 0.5 * self_->GetPerceptionRadius()) {
        sum_diff_pos_ += diff_pos / (dist * dist);
      }
      n++;
    }
  }

  Double3 GetAvgPosDirection() {
    if (n != 0) {
      sum_pos_ /= n;
      sum_pos_ -= boid_pos_;
    }
    return sum_pos_;
  }

  Double3 GetDiffPos() {
    if (n != 0) {
      sum_diff_pos_ /= n;
    }
    return sum_diff_pos_;
  }

  Double3 GetAvgVel() {
    if (n != 0) {
      sum_vel_ /= n;
    }
    return sum_vel_;
  }

  Boid* self_;
  Double3 boid_pos_ = self_->GetPosition();
  Double3 sum_pos_ = {0, 0, 0};
  Double3 sum_vel_ = {0, 0, 0};
  Double3 sum_diff_pos_ = {0, 0, 0};
  int n = 0;
};

///////////////////////////////////////////////////////////////////////////////
// Flocking Behaviour
///////////////////////////////////////////////////////////////////////////////
struct Flocking : public Behavior {
  BDM_BEHAVIOR_HEADER(Flocking, Behavior, 1);

  void Run(Agent* agent) override {
    auto* boid = dynamic_cast<Boid*>(agent);
    auto* ctxt = Simulation::GetActive()->GetExecutionContext();

    double perception_radius = boid->GetPerceptionRadius();
    double perception_radius_squared = perception_radius * perception_radius;

    ///////////////////////////////////////////////////////////////////////////////
    // Calculate seperation_force, alignment_force, cohesion_force,
    // avoid_domain_boundary_force
    CalculateNeighborData NeighborData(boid);
    ctxt->ForEachNeighbor(NeighborData, *boid, perception_radius_squared);

    Double3 seperation_force = boid->SteerTowards(NeighborData.GetDiffPos());
    Double3 alignment_force = boid->SteerTowards(NeighborData.GetAvgVel());
    Double3 cohesion_force =
        boid->SteerTowards(NeighborData.GetAvgPosDirection());
    Double3 avoid_domain_boundary_force = boid->AvoidDomainBoundary();

    ///////////////////////////////////////////////////////////////////////////////
    // Update acceleration_, new_velocity_, new_position_ of boid
    boid->ResetAcceleration();
    boid->AccelerationAccumulator(seperation_force * boid->seperation_weight_);
    boid->AccelerationAccumulator(alignment_force * boid->alignment_weight_);
    boid->AccelerationAccumulator(cohesion_force * boid->cohesion_weight_);
    boid->AccelerationAccumulator(avoid_domain_boundary_force *
                                  boid->avoid_domain_boundary_weight_);

    boid->UpdateNewVelocity();
    boid->UpdateNewPosition();
  }
};

}  // namespace bdm

#endif  // BEHAVIOR_H_
