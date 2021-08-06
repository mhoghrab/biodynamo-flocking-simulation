#include "boid.h"
#include "core/container/math_array.h"
#include "direction_array.h"

using namespace bdm;

Double3 Boid::UpperLimit(Double3 vector, double limit) {
  if (vector.Norm() > limit) {
    vector = vector.Normalize() * limit;
  }
  return vector;
}

Double3 Boid::LowerLimit(Double3 vector, double limit) {
  if (vector.Norm() < limit) {
    vector = vector.Normalize() * limit;
  }
  return vector;
}

// bool Boid::IsHeadingForCollision() { return false; }

// Double3 Boid::ObstacleAvoidanceForce() {
//   auto rayDirections = DirectionArray().GetAlignedDirections(velocity_);
//   for (int i = 0; i < sizeof(rayDirections); i++) {
//   }
// }

bool Boid::CheckIfVisible(Double3 point) {
  Double3 cone_normal = headingDirection_;
  Double3 direction_normal = (point - GetPosition()).Normalize();
  double angle = std::acos(cone_normal * direction_normal);

  if (angle > perceptionAngle_) {
    return false;
  } else {
    return true;
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
  Double3 steer = vector.Normalize() * maxSpeed_ - velocity_;
  return UpperLimit(steer, maxForce_);
}

void Boid::UpdatePosition() {
  Double3 position = GetPosition();
  position += velocity_;
  // position = OpenLoopDomain(position);
  SetPosition(position);
}

void Boid::UpdateVelocity() {
  acceleration_ = UpperLimit(acceleration_, maxForce_);
  velocity_ += acceleration_;
  velocity_ = UpperLimit(velocity_, maxSpeed_);
  velocity_ = LowerLimit(velocity_, minSpeed_);
  headingDirection_ = velocity_;
}

void Boid::ResetAcceleration() { acceleration_ = {0, 0, 0}; }

void Boid::AccelerationAccumulator(Double3 acc2add) {
  acceleration_ += acc2add;
}
