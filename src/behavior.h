#ifndef BEHAVIOR_H_
#define BEHAVIOR_H_

#include "core/behavior/behavior.h"
#include "boid.h"
#include <math.h>
//#include "usefulFunctions.h"

namespace bdm {

struct Flocking : public Behavior {
  BDM_BEHAVIOR_HEADER(Flocking, Behavior, 1);
  
  void Run(Agent* agent) override {
    // References
    auto* boid = dynamic_cast<Boid*>(agent);
    auto* ctxt = Simulation::GetActive()->GetExecutionContext();
    //const auto* param = Simulation::GetActive()->GetParam();

    double perception_radius = boid->GetPerceptionRadius();
    double perception_radius_squared = perception_radius * perception_radius;
    
    Double3 const boid_position = boid->GetPosition();

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
          Double3 other_vel = other->GetVelocity();
          Double3 boid_pos = self_->GetPosition();
          Double3 diff_pos = boid_pos - other_pos;

          double perception_radius = self_->GetPerceptionRadius();
          double dist = diff_pos.Norm();

          //check if neighbor is in viewing cone, if not return
          Double3 cone_normal = (self_->GetVelocity()).Normalize();
          float angle = std::acos((cone_normal*(other_pos - boid_pos)) / ((other_pos - boid_pos).Norm()));
          if (angle > (self_->perceptionAngle_)){
            return;
          }

          sum_pos += other_pos;
          sum_vel += other_vel;
          // if (dist < self_->comfort_radius_) {
          //   double magnitude = (self_->comfort_radius_ - dist) / (self_->comfort_radius_ - self_->dangerRadius_);
          //   diff_pos = diff_pos.Normalize() * magnitude;
          //   sum_diff_pos += diff_pos;
          // }
          if (dist < 0.3 * perception_radius) {
            diff_pos = diff_pos / (dist*dist);
            sum_diff_pos += diff_pos;
          }
          n++;
        }

        Double3 GetAvgPos() {
          if (n!=0) {sum_pos /= n;}
          return sum_pos;
        }

        Double3 GetAvgDiffPos() {
          if (n!=0) {sum_diff_pos /= n;}
          return sum_diff_pos;
        }

        Double3 GetAvgVel() {
          if (n!=0) {sum_vel /= n;}
          return sum_vel;
        }

        Boid* self_;
        Double3 sum_pos = {0,0,0};
        Double3 sum_vel = {0,0,0};
        Double3 sum_diff_pos = {0,0,0};
        int n = 0;
    };

    // -----------------------------------------------------------------------------
    // Calculate seperationForce, alignmentForce, cohesionForce
    // -----------------------------------------------------------------------------
    
    CalculateNeighborData NeighborData(boid);
    ctxt->ForEachNeighbor(NeighborData, *boid, perception_radius_squared);

    Double3 seperationForce = boid->SteerTowards(NeighborData.GetAvgDiffPos());
    Double3 alignmentForce = boid->SteerTowards(NeighborData.GetAvgVel());
    Double3 cohesionForce = boid->SteerTowards(NeighborData.GetAvgPos() - boid_position);
    //Double3 staForce = boid->STA();

    // -----------------------------------------------------------------------------
    // Update Position, velocity, acceleration of boid
    // -----------------------------------------------------------------------------

    boid->ResetAcceleration();
    boid->AccelerationAccumulator(seperationForce * boid->seperationWeight);
    boid->AccelerationAccumulator(alignmentForce * boid->alignmentWeight);
    boid->AccelerationAccumulator(cohesionForce * boid->cohesionWeight);

    // if (boid->IsHeadingForCollision()){
    //   Double3 obstacleAvoidanceDirection = boid->GetObstacleAvoidanceDirection();
    //   Double3 ObstacleAvoidanceForce = boid->SteerTowards(obstacleAvoidanceDirection);
    //   boid->AccelerationAccumulator(cohesionForce * boid->obstacleAvoidanceWeight);
    // }

    // boid->AccelerationAccumulator(staForce * boid->staWeight); 

    boid->UpdateVelocity();
    boid->UpdatePosition();
  } 
};    
}  // namespace bdm

#endif  // BEHAVIOR_H_
