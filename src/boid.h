#ifndef BOID_H_
#define BOID_H_

#include "core/agent/cell.h"
#include "core/behavior/behavior.h"
#include "core/container/math_array.h"
#include "core/functor.h"

namespace bdm {

class Boid : public Cell {
  BDM_AGENT_HEADER(Boid, Cell, 1);

 public:
  Boid() {}
  explicit Boid(const Double3& position) : Base(position) {}
  virtual ~Boid() {}

  ///////////////////////////////////////////////////////////////////////////////
  // Initializes Boid parameters with given SimParam
  void InitializeMembers();

  ///////////////////////////////////////////////////////////////////////////////
  // Various Getter and Setter
  Double3 GetVelocity() const { return velocity_; }
  void SetVelocity(Double3 velocity) {
    velocity_ = velocity;
    heading_direction_ = velocity_.Normalize();
    // why does it result in 2 completly different behaviors when using
    // velocity.Normalize() or velocity_.Normalize() ???????????
  }

  Double3 GetAcceleration() const { return acceleration_; }
  void SetAcceleration(Double3 acceleration) { acceleration_ = acceleration; }

  Double3 GetNewPosition() const { return new_position_; }
  void SetNewPosition(Double3 position) { new_position_ = position; }

  Double3 GetNewVelocity() const { return new_velocity_; }
  void SetNewVelocity(Double3 velocity) { new_velocity_ = velocity; }

  double GetActualDiameter() const { return actual_diameter_; }
  void SetActualDiameter(double actual_diameter) {
    actual_diameter_ = actual_diameter;
  }

  double GetPerceptionRadius() const { return perception_radius_; }
  void SetPerceptionRadius(double perception_radius) {
    perception_radius_ = perception_radius;
    SetDiameter(perception_radius_ * 2);
  }

  void SetPerceptionAngle(double angle) {
    perception_angle_ = angle;
    cos_perception_angle_ = std::cos(angle);
  }

  ///////////////////////////////////////////////////////////////////////////////
  // Limit/Clamp the length of a vector to a upper or/and lower limit
  Double3 UpperLimit(Double3 vector, double upper_limit);
  Double3 LowerLimit(Double3 vector, double lower_limit);
  Double3 ClampUpperLower(Double3 vector, double upper_limit,
                          double lower_limit);

  ///////////////////////////////////////////////////////////////////////////////
  // Returns bool wether given point is inside viewing cone defined by
  // heading_direction_ and perception_angle_
  bool CheckIfVisible(Double3 point);

  ///////////////////////////////////////////////////////////////////////////////
  // Returns a Steering-Force to avoid colliding into domain boundaries
  Double3 AvoidDomainBoundary();

  ///////////////////////////////////////////////////////////////////////////////
  // Returns the position vector, but if a coordinate exceeds the boundarys it
  // will get set to the opposite site of the somain
  Double3 OpenLoopDomain(Double3 position);

  ///////////////////////////////////////////////////////////////////////////////
  // Returns a Steering-Force in order to steer velocity towards
  // (vector.Normalize() * crusing_speed_)
  // Force is limited by max_force_
  Double3 SteerTowards(Double3 vector);

  ///////////////////////////////////////////////////////////////////////////////
  // Update new_position_ by adding new_velocity_
  void UpdateNewPosition();

  ///////////////////////////////////////////////////////////////////////////////
  // Update new_velocity_ by adding acceleration_ and clamping it by max_speed_
  // and min_speed_
  void UpdateNewVelocity();

  ///////////////////////////////////////////////////////////////////////////////
  // Sets acceleration_ to {0,0,0}
  void ResetAcceleration();

  ///////////////////////////////////////////////////////////////////////////////
  // Right now simply adds acc2add to the stored acceleration_
  void AccelerationAccumulator(Double3 acceleration_to_add);

  ///////////////////////////////////////////////////////////////////////////////
  // Sets the actual position / velocity to new_position_ / new_velocity_
  void UpdateData();

  ///////////////////////////////////////////////////////////////////////////////
  Double3 new_position_, new_velocity_;
  Double3 acceleration_, velocity_, heading_direction_;
  double actual_diameter_ = 15, perception_radius_ = 150,
         perception_angle_ = M_PI, cos_perception_angle_;
  double max_force_ = 3, max_speed_ = 20, crusing_speed_ = 15, min_speed_ = 10;
  double cohesion_weight_ = 1, alignment_weight_ = 2, seperation_weight_ = 1.5,
         avoid_domain_boundary_weight_ = 25, obstacle_avoidance_weight_ = 5;
};

///////////////////////////////////////////////////////////////////////////////
// Functor class needed to calculate neighbor data in ForEachNeighbor call
///////////////////////////////////////////////////////////////////////////////
class CalculateNeighborData : public Functor<void, Agent*, double> {
 public:
  CalculateNeighborData(Boid* boid) : boid_(boid) {
    boid_pos_ = boid_->GetPosition();
    sum_pos_ = {0, 0, 0};
    sum_vel_ = {0, 0, 0};
    sum_diff_pos_ = {0, 0, 0};
    n = 0;
  }
  virtual ~CalculateNeighborData() {}

  void operator()(Agent* neighbor, double squared_distance) override;

  Double3 GetAvgPosDirection();

  Double3 GetDiffPos();

  Double3 GetAvgVel();

 private:
  Boid* boid_;
  Double3 boid_pos_;
  Double3 sum_pos_;
  Double3 sum_vel_;
  Double3 sum_diff_pos_;
  int n;
};

///////////////////////////////////////////////////////////////////////////////
// Flocking Behaviour
///////////////////////////////////////////////////////////////////////////////
struct Flocking : public Behavior {
  BDM_BEHAVIOR_HEADER(Flocking, Behavior, 1);

  void Run(Agent* agent) override;
};

}  // namespace bdm

#endif  // BOID_H_
