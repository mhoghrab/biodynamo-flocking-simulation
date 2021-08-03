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

  Double3 GetVelocity() const {return velocity_;}
  void SetVelocity(Double3 velocity) {velocity_ = velocity;}

  // Double3 GetAcceleration() const {return acceleration_;}
  // void SetAcceleration(Double3 acceleration) {acceleration_ = acceleration;}

  // Double3 GetTempPosition() const {return tempPosition_;}
  // void SetTempPosition(Double3 tempPosition) {tempPosition_ = tempPosition;}

  // Double3 GetTempVelocity() const {return tempVelocity_; }
  // void SetTempVelocity(Double3 tempVelocity) {tempVelocity_ = tempVelocity;}

  void SetActualDiameter(double actualDiameter) {actualDiameter_ = actualDiameter;}
  double GetActualDiameter() const {return actualDiameter_;}

  double GetPerceptionRadius() const {return perceptionRadius_;}
  void SetPerceptionRadius(double perceptionRadius) {
    perceptionRadius_ = perceptionRadius; 
    this->SetDiameter(perceptionRadius_*2);
  }

  // -----------------------------------------------------------------------------
  // Helper functions
  Double3 UpperLimit(Double3 vector, double limit);

  Double3 LowerLimit(Double3 vector, double limit);

  // obstacle avoidance as in https://github.com/SebLague/Boids/blob/master/Assets/Scripts/Boid.cs
  bool IsHeadingForCollision();

  Double3 GetObstacleAvoidanceDirection();

  Double3 STA();

  Double3 OpenLoopDomain(Double3 position);

  Double3 SteerTowards (Double3 vector);

  void UpdatePosition();

  void UpdateVelocity();

  void ResetAcceleration();

  void AccelerationAccumulator(Double3 acc2add);


  Double3 tempPosition_, tempVelocity_, velocity_, acceleration_;
  double actualDiameter_, perceptionRadius_, perceptionAngle_ = M_PI;
  double comfort_radius_ = 4 * actualDiameter_, dangerRadius_ = 2 * actualDiameter_;
  double maxForce_ = 2, maxSpeed_ = 15, minSpeed_ = 10;
  double cohesionWeight = 1, alignmentWeight = 2, seperationWeight = 2.5, staWeight = 100, obstacleAvoidanceWeight = 5;
};

}  // namespace bdm

#endif  // BOID_H_
