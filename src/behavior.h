#ifndef BEHAVIOR_H_
#define BEHAVIOR_H_

#include <math.h>
#include "boid.h"
#include "core/behavior/behavior.h"
#include "core/functor.h"

namespace bdm {

// -----------------------------------------------------------------------------
// Functor class needed to calculate neighbor data in ForEachNeighbor call
// -----------------------------------------------------------------------------
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
      Double3 diff_pos = boid_pos - other_pos;

      double dist = diff_pos.Norm();

      sum_pos += other_pos;
      sum_vel += other_vel;

      //// perhaps check if dist < eps
      if (dist < 0.5 * perception_radius) {
        sum_diff_pos += diff_pos / (dist * dist);
      }
      n++;
    }
  }

  Double3 GetAvgPos() {
    if (n != 0) {
      sum_pos /= n;
    }
    return sum_pos;
  }

  Double3 GetDiffPos() {
    if (n != 0) {
      sum_diff_pos /= n;
    }
    return sum_diff_pos;
  }

  Double3 GetAvgVel() {
    if (n != 0) {
      sum_vel /= n;
    }
    return sum_vel;
  }

  Boid* self_;
  Double3 boid_pos = self_->GetPosition();
  double perception_radius = self_->GetPerceptionRadius();
  Double3 sum_pos = {0, 0, 0};
  Double3 sum_vel = {0, 0, 0};
  Double3 sum_diff_pos = {0, 0, 0};
  int n = 0;
};

struct Flocking : public Behavior {
  BDM_BEHAVIOR_HEADER(Flocking, Behavior, 1);

  void Run(Agent* agent) override {
    auto* boid = dynamic_cast<Boid*>(agent);
    auto* ctxt = Simulation::GetActive()->GetExecutionContext();

    double perception_radius = boid->GetPerceptionRadius();
    double perception_radius_squared = perception_radius * perception_radius;

    Double3 const boid_position = boid->GetPosition();

    // -----------------------------------------------------------------------------
    // Calculate seperationForce, alignmentForce, cohesionForce,
    // avoidDomainBoundaryForce
    // -----------------------------------------------------------------------------

    CalculateNeighborData NeighborData(boid);
    ctxt->ForEachNeighbor(NeighborData, *boid, perception_radius_squared);

    Double3 seperationForce = boid->SteerTowards(NeighborData.GetDiffPos());
    Double3 alignmentForce = boid->SteerTowards(NeighborData.GetAvgVel());
    Double3 cohesionForce =
        boid->SteerTowards(NeighborData.GetAvgPos() - boid_position);
    Double3 avoidDomainBoundaryForce = boid->AvoidDomainBoundary();
    // Double3 ObstacleAvoidanceForce = boid->ObstacleAvoidanceForce();

    // -----------------------------------------------------------------------------
    // Update Position, velocity, acceleration of boid
    // -----------------------------------------------------------------------------

    boid->ResetAcceleration();
    boid->AccelerationAccumulator(seperationForce * boid->seperationWeight);
    boid->AccelerationAccumulator(alignmentForce * boid->alignmentWeight);
    boid->AccelerationAccumulator(cohesionForce * boid->cohesionWeight);
    boid->AccelerationAccumulator(avoidDomainBoundaryForce *
                                  boid->avoidDomainBoundaryWeight);
    // boid->AccelerationAccumulator(ObstacleAvoidanceForce *
    //                               boid->obstacleAvoidanceWeight);

    boid->UpdateVelocity();
    boid->UpdatePosition();
  }
};

// struct PredatorHunting : public Behavior {
//   BDM_BEHAVIOR_HEADER(PredatorHunting, Behavior, 1);

//   void Run(Agent* agent) override {
//     auto functor = L2F([&](Agent * neighbor, double squared_distance)) {
//       if (neighbor->GetDiameter() < threshold) {
//         std::cout << neighbor->GetUid() << std::endl;
//       }
//     };

//     auto* boid = dynamic_cast<Boid*>(agent);
//     auto* ctxt = Simulation::GetActive()->GetExecutionContext();

//     double perception_radius = boid->GetPerceptionRadius();
//     double perception_radius_squared = perception_radius * perception_radius;

//     Double3 const boid_position = boid->GetPosition();
//   }
// };
}  // namespace bdm

#endif  // BEHAVIOR_H_
