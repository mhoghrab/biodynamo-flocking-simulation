#include "boid.h"

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
// obstacle avoidance as in
// https://github.com/SebLague/Boids/blob/master/Assets/Scripts/Boid.cs
bool Boid::IsHeadingForCollision() {
  // 2do
  return true;
}

Double3 Boid::GetObstacleAvoidanceDirection() {
  auto rayDirections =
      DirectionArray().directions;  // with rayDirections[0] = {0,0,1}
  // convert to velocity directions
  for (int i = 0; i < sizeof(rayDirections); i++) {
    Double3 dir = {rayDirections[i][0], rayDirections[i][1],
                   rayDirections[i][2]};
    // if (Ray(position, dir) is clear path)

    return dir;
    //}
  }

  return velocity_.Normalize();
}

Double3 Boid::STA() {  // Steer To Avoid Force for avoiding crashing into the domains
                 // boundarys
  const auto* param = Simulation::GetActive()->GetParam();
  double min_bound = param->min_bound;
  double max_bound = param->max_bound;

  Double3 staForce = {0, 0, 0};
  Double3 position = GetPosition();
  // parameters
  double getAwayVelocity = minSpeed_ * 0.3;
  double avoidanceDistance = 5 * actualDiameter_;
  //

  for (int i = 0; i < 2; i++) {
    if (velocity_[i] <= getAwayVelocity and
        abs(position[i] - min_bound) < avoidanceDistance) {
      Double3 Force = {0, 0, 0};
      Force[i] = 1;
      double dist = abs(position[i] - min_bound);
      // Force = Force / (dist*dist + 1);
      staForce += Force;
    }
  }

  for (int i = 0; i < 2; i++) {
    if (velocity_[i] >= -1 * getAwayVelocity and
        abs(position[i] - max_bound) < avoidanceDistance) {
      Double3 Force = {0, 0, 0};
      Force[i] = -1;
      double dist = abs(position[i] - max_bound);
      // Force = Force / (dist*dist + 1);
      staForce += Force;
    }
  }

  return UpperLimit(staForce, maxForce_);
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
  position = OpenLoopDomain(position);
  SetPosition(position);
}

void Boid::UpdateVelocity() {
  acceleration_ = UpperLimit(acceleration_, maxForce_);
  velocity_ += acceleration_;
  velocity_ = UpperLimit(velocity_, maxSpeed_);
  velocity_ = LowerLimit(velocity_, minSpeed_);
}

void Boid::ResetAcceleration() { acceleration_ = {0, 0, 0}; }

void Boid::AccelerationAccumulator(Double3 acc2add) { acceleration_ += acc2add; }