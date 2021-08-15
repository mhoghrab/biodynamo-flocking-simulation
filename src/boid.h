#ifndef BOID_H_
#define BOID_H_

#include "core/agent/cell.h"
#include "core/container/math_array.h"

namespace bdm {

class Boid : public Cell {
  BDM_AGENT_HEADER(Boid, Cell, 1);

 public:
  Boid() {}
  explicit Boid(const Double3& position) : Base(position) {}
  virtual ~Boid() {}

  ///////////////////////////////////////////////////////////////////////////////
  // Initializes Boid parameters with given SimParam
  void Initialize();

  ///////////////////////////////////////////////////////////////////////////////
  // Various Getter and Setter
  Double3 GetVelocity() const { return velocity_; }
  void SetVelocity(Double3 velocity) {
    velocity_ = velocity;
    headingDirection_ = velocity_.Normalize();
    // why does it result in 2 completly different behaviors when using
    // velocity.Normalize() or velocity_.Normalize() ???????????
  }

  Double3 GetAcceleration() const { return acceleration_; }
  void SetAcceleration(Double3 acceleration) { acceleration_ = acceleration; }

  Double3 GetNewPosition() const { return newPosition_; }
  void SetNewPosition(Double3 position) { newPosition_ = position; }

  Double3 GetNewVelocity() const { return newVelocity_; }
  void SetNewVelocity(Double3 velocity) { newVelocity_ = velocity; }

  double GetActualDiameter() const { return actualDiameter_; }
  void SetActualDiameter(double actualDiameter) {
    actualDiameter_ = actualDiameter;
  }

  double GetPerceptionRadius() const { return perceptionRadius_; }
  void SetPerceptionRadius(double perceptionRadius) {
    perceptionRadius_ = perceptionRadius;
    this->SetDiameter(perceptionRadius_ * 2);
  }

  void SetPerceptionAngle(double angle) {
    perceptionAngle_ = angle;
    cosPerceptionAngle_ = std::cos(angle);
  }

  ///////////////////////////////////////////////////////////////////////////////
  // Limit/Clamp the length of a vector to a upper or/and lower limit
  Double3 UpperLimit(Double3 vector, double upperLimit);
  Double3 LowerLimit(Double3 vector, double lowerLimit);
  Double3 ClampUpperLower(Double3 vector, double upperLimit, double lowerLimit);

  ///////////////////////////////////////////////////////////////////////////////
  // Returns bool wether given point is inside viewing cone defined by
  // headingDirection_ and perceptionAngle_
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
  // (vector.Normalize() * crusingSpeed_)
  // Force is limited by maxForce_
  Double3 SteerTowards(Double3 vector);

  ///////////////////////////////////////////////////////////////////////////////
  // Update newPosition_ by adding newVelocity_
  void UpdateNewPosition();

  ///////////////////////////////////////////////////////////////////////////////
  // Update newVelocity_ by adding acceleration_ and clamping it by maxSpeed_
  // and minSpeed_
  void UpdateNewVelocity();

  ///////////////////////////////////////////////////////////////////////////////
  // Sets acceleration_ to {0,0,0}
  void ResetAcceleration();

  ///////////////////////////////////////////////////////////////////////////////
  // Right now simply adds acc2add to the stored acceleration_
  void AccelerationAccumulator(Double3 acc2add);

  ///////////////////////////////////////////////////////////////////////////////
  // Sets the actual position/velocity to newPosition_/newVelocity_
  void UpdateData();

  ///////////////////////////////////////////////////////////////////////////////
  Double3 newPosition_, newVelocity_;
  Double3 acceleration_, velocity_, headingDirection_;
  double actualDiameter_ = 15, perceptionRadius_ = 150, perceptionAngle_ = M_PI,
         cosPerceptionAngle_;
  double maxForce_ = 3, maxSpeed_ = 20, crusingSpeed_ = 15, minSpeed_ = 10;
  double cohesionWeight = 1, alignmentWeight = 2, seperationWeight = 1.5,
         avoidDomainBoundaryWeight = 25, obstacleAvoidanceWeight = 5;
};

}  // namespace bdm

#endif  // BOID_H_