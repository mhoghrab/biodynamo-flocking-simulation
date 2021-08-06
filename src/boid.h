#ifndef BOID_H_
#define BOID_H_

#include "core/agent/cell.h"
#include "core/container/math_array.h"
#include "direction_array.h"

namespace bdm {

class Boid : public Cell {
  BDM_AGENT_HEADER(Boid, Cell, 1);

 public:
  Boid() {}
  explicit Boid(const Double3& position) : Base(position) {}
  virtual ~Boid() {}

  Double3 GetVelocity() const { return velocity_; }
  void SetVelocity(Double3 velocity) {
    velocity_ = velocity;
    headingDirection_ = velocity.Normalize();
  }

  Double3 GetAcceleration() const { return acceleration_; }
  void SetAcceleration(Double3 acceleration) { acceleration_ = acceleration; }

  // Double3 GetTempPosition() const { return tempPosition_; }
  // void SetTempPosition(Double3 tempPosition) { tempPosition_ = tempPosition;
  // }

  // Double3 GetTempVelocity() const { return tempVelocity_; }
  // void SetTempVelocity(Double3 tempVelocity) { tempVelocity_ = tempVelocity;
  // }

  void SetActualDiameter(double actualDiameter) {
    actualDiameter_ = actualDiameter;
  }
  double GetActualDiameter() const { return actualDiameter_; }

  double GetPerceptionRadius() const { return perceptionRadius_; }
  void SetPerceptionRadius(double perceptionRadius) {
    perceptionRadius_ = perceptionRadius;
    this->SetDiameter(perceptionRadius_ * 2);
  }

  Double3 UpperLimit(Double3 vector, double limit);
  Double3 LowerLimit(Double3 vector, double limit);

  // obstacle avoidance as in
  // https://github.com/SebLague/Boids/blob/master/Assets/Scripts/Boid.cs
  bool IsHeadingForCollision();
  Double3 ObstacleAvoidanceForce();

  // returns bool if given point is visible from / inside cone defined by
  // velocity_ and perceptionAngle_
  bool CheckIfVisible(Double3 point);

  Double3 AvoidDomainBoundary();

  Double3 OpenLoopDomain(Double3 position);

  Double3 SteerTowards(Double3 vector);

  void UpdatePosition();

  void UpdateVelocity();

  void ResetAcceleration();

  void AccelerationAccumulator(Double3 acc2add);

  // Double3 tempPosition_, tempVelocity_;
  Double3 acceleration_, velocity_, headingDirection_;
  double actualDiameter_, perceptionRadius_, perceptionAngle_ = 3 / 4 * M_PI;
  double maxForce_ = 3, maxSpeed_ = 15, minSpeed_ = 10;
  double cohesionWeight = 1, alignmentWeight = 2, seperationWeight = 1.5,
         avoidDomainBoundaryWeight = 25, obstacleAvoidanceWeight = 5;
};

}  // namespace bdm

#endif  // BOID_H_