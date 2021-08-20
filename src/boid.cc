#include "boid.h"
#include "core/container/math_array.h"
#include "flocking_simulation.h"

using namespace bdm;

void Boid::InitializeMembers() {
  const auto* param = Simulation::GetActive()->GetParam();
  const auto* sparam = param->Get<SimParam>();
  actualDiameter_ = sparam->actualDiameter_;
  SetPerceptionRadius(sparam->perceptionRadius_);
  SetPerceptionAngle(sparam->perceptionAngle_);
  maxForce_ = sparam->maxForce_;
  maxSpeed_ = sparam->maxSpeed_;
  crusingSpeed_ = sparam->crusingSpeed;
  minSpeed_ = sparam->minSpeed_;
  cohesionWeight = sparam->cohesionWeight;
  alignmentWeight = sparam->alignmentWeight;
  seperationWeight = sparam->seperationWeight;
  avoidDomainBoundaryWeight = sparam->avoidDomainBoundaryWeight;
  obstacleAvoidanceWeight = sparam->obstacleAvoidanceWeight;

  SetNewPosition(GetPosition());
  SetNewVelocity(GetVelocity());
}

Double3 Boid::UpperLimit(Double3 vector, double upperLimit) {
  double length = vector.Norm();
  if (length > upperLimit) {
    vector = (vector / length) * upperLimit;
  }
  return vector;
}

Double3 Boid::LowerLimit(Double3 vector, double lowerLimit) {
  double length = vector.Norm();
  if (length < lowerLimit) {
    vector = (vector / length) * lowerLimit;
  }
  return vector;
}

Double3 Boid::ClampUpperLower(Double3 vector, double upperLimit,
                              double lowerLimit) {
  double length = vector.Norm();
  if (length > upperLimit) {
    vector = (vector / length) * upperLimit;
  }
  if (length < lowerLimit) {
    vector = (vector / length) * lowerLimit;
  }
  return vector;
}

bool Boid::CheckIfVisible(Double3 point) {
  Double3 cone_normal = headingDirection_;
  Double3 direction_normal = (point - GetPosition()).Normalize();
  double cosAngle = cone_normal * direction_normal;
  if (cosAngle >= cosPerceptionAngle_) {
    return true;
  } else {
    return false;
  }
}

Double3 Boid::AvoidDomainBoundary() {
  const auto* param = Simulation::GetActive()->GetParam();
  double min_bound = param->min_bound;
  double max_bound = param->max_bound;

  Double3 avoidDomainBoundaryForce = {0, 0, 0};
  Double3 position = GetPosition();
  double getawayVelocity = 0.7 * minSpeed_;
  double avoidanceDistance = 0.3 * perceptionRadius_;

  for (int i = 0; i <= 2; i++) {
    if (velocity_[i] <= getawayVelocity and
        abs(position[i] - min_bound) < avoidanceDistance) {
      Double3 Force = {0, 0, 0};
      Force[i] = 1;
      double dist = abs(position[i] - min_bound);
      Force = Force * (3 * actualDiameter_) / (dist * dist + 0.01);
      avoidDomainBoundaryForce += SteerTowards(Force);
    }
  }

  for (int i = 0; i <= 2; i++) {
    if (velocity_[i] >= -1 * getawayVelocity and
        abs(position[i] - max_bound) < avoidanceDistance) {
      Double3 Force = {0, 0, 0};
      Force[i] = -1;
      double dist = abs(position[i] - max_bound);
      Force = Force * (3 * actualDiameter_) / (dist * dist + 0.01);
      avoidDomainBoundaryForce += SteerTowards(Force);
    }
  }

  return UpperLimit(avoidDomainBoundaryForce, maxForce_);
}

Double3 Boid::OpenLoopDomain(Double3 position) {
  const auto* param = Simulation::GetActive()->GetParam();
  double min_bound = param->min_bound;
  double max_bound = param->max_bound;

  if (position[0] > max_bound) {
    position[0] = min_bound;
  }
  if (position[1] > max_bound) {
    position[1] = min_bound;
  }
  if (position[2] > max_bound) {
    position[2] = min_bound;
  }

  if (position[0] < min_bound) {
    position[0] = max_bound;
  }
  if (position[1] < min_bound) {
    position[1] = max_bound;
  }
  if (position[2] < min_bound) {
    position[2] = max_bound;
  }
  return position;
}

Double3 Boid::SteerTowards(Double3 vector) {
  if (vector.Norm() == 0) {
    return Double3{0, 0, 0};
  }
  Double3 steer = vector.Normalize() * crusingSpeed_ - velocity_;
  return UpperLimit(steer, maxForce_);
}

void Boid::UpdateNewPosition() {
  newPosition_ += newVelocity_;
  // newPosition_ = OpenLoopDomain(newPosition);
}

void Boid::UpdateNewVelocity() {
  acceleration_ = UpperLimit(acceleration_, maxForce_);
  newVelocity_ += acceleration_;
  newVelocity_ = ClampUpperLower(newVelocity_, maxSpeed_, minSpeed_);
}

void Boid::ResetAcceleration() { acceleration_ = {0, 0, 0}; }

void Boid::UpdateData() {
  SetVelocity(GetNewVelocity());
  SetPosition(GetNewPosition());
}

void Boid::AccelerationAccumulator(Double3 acc2add) {
  acceleration_ += acc2add;
}
