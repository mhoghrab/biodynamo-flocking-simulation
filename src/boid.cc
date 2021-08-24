#include "boid.h"
#include "core/container/math_array.h"
#include "sim_param.h"

using namespace bdm;

void Boid::InitializeMembers() {
  const auto* param = Simulation::GetActive()->GetParam();
  const auto* sparam = param->Get<SimParam>();
  actual_diameter_ = sparam->actual_diameter;
  SetPerceptionRadius(sparam->perception_radius);
  SetPerceptionAngle(sparam->perception_angle);
  max_force_ = sparam->max_force;
  max_speed_ = sparam->max_speed;
  crusing_speed_ = sparam->crusing_speed;
  min_speed_ = sparam->min_speed;
  cohesion_weight_ = sparam->cohesion_weight;
  alignment_weight_ = sparam->alignment_weight;
  seperation_weight_ = sparam->seperation_weight;
  avoid_domain_boundary_weight_ = sparam->avoid_domain_boundary_weight;
  obstacle_avoidance_weight_ = sparam->obstacle_avoidance_weight;

  SetNewPosition(GetPosition());
  SetNewVelocity(GetVelocity());
}

Double3 Boid::UpperLimit(Double3 vector, double upper_limit) {
  double length = vector.Norm();
  if (length > upper_limit) {
    vector = (vector / length) * upper_limit;
  }
  return vector;
}

Double3 Boid::LowerLimit(Double3 vector, double lower_limit) {
  double length = vector.Norm();
  if (length < lower_limit) {
    vector = (vector / length) * lower_limit;
  }
  return vector;
}

Double3 Boid::ClampUpperLower(Double3 vector, double upper_limit,
                              double lower_limit) {
  double length = vector.Norm();
  if (length > upper_limit) {
    vector = (vector / length) * upper_limit;
  }
  if (length < lower_limit) {
    vector = (vector / length) * lower_limit;
  }
  return vector;
}

bool Boid::CheckIfVisible(Double3 point) {
  Double3 cone_normal = heading_direction_;
  Double3 direction_normal = (point - GetPosition()).Normalize();
  double cosAngle = cone_normal * direction_normal;
  if (cosAngle >= cos_perception_angle_) {
    return true;
  } else {
    return false;
  }
}

Double3 Boid::AvoidDomainBoundary() {
  const auto* param = Simulation::GetActive()->GetParam();
  double min_bound = param->min_bound;
  double max_bound = param->max_bound;

  Double3 avoid_domain_boundary_force = {0, 0, 0};
  Double3 position = GetPosition();
  double get_away_velocity = 0.7 * min_speed_;
  double avoidance_distance = 0.3 * perception_radius_;

  for (int i = 0; i <= 2; i++) {
    if (velocity_[i] <= get_away_velocity and
        abs(position[i] - min_bound) < avoidance_distance) {
      Double3 force = {0, 0, 0};
      force[i] = 1;
      double dist = abs(position[i] - min_bound);
      force = force * (3 * actual_diameter_) / (dist * dist + 0.01);
      avoid_domain_boundary_force += SteerTowards(force);
    }
  }

  for (int i = 0; i <= 2; i++) {
    if (velocity_[i] >= -1 * get_away_velocity and
        abs(position[i] - max_bound) < avoidance_distance) {
      Double3 force = {0, 0, 0};
      force[i] = -1;
      double dist = abs(position[i] - max_bound);
      force = force * (3 * actual_diameter_) / (dist * dist + 0.01);
      avoid_domain_boundary_force += SteerTowards(force);
    }
  }

  return UpperLimit(avoid_domain_boundary_force, max_force_);
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
  Double3 steer = vector.Normalize() * crusing_speed_ - velocity_;
  return UpperLimit(steer, max_force_);
}

void Boid::UpdateNewPosition() {
  new_position_ += new_velocity_;
  // new_position_ = OpenLoopDomain(newPosition);
}

void Boid::UpdateNewVelocity() {
  acceleration_ = UpperLimit(acceleration_, max_force_);
  new_velocity_ += acceleration_;
  new_velocity_ = ClampUpperLower(new_velocity_, max_speed_, min_speed_);
}

void Boid::ResetAcceleration() { acceleration_ = {0, 0, 0}; }

void Boid::UpdateData() {
  SetVelocity(GetNewVelocity());
  SetPosition(GetNewPosition());
}

void Boid::AccelerationAccumulator(Double3 acceleration_to_add) {
  acceleration_ += acceleration_to_add;
}

void CalculateNeighborData::operator()(Agent* neighbor,
                                       double squared_distance) {
  auto* other = bdm_static_cast<const Boid*>(neighbor);
  Double3 other_pos = other->GetPosition();

  // check if neighbor is in viewing cone
  if (boid_->CheckIfVisible(other_pos)) {
    Double3 other_vel = other->GetVelocity();
    Double3 diff_pos = boid_pos_ - other_pos;

    double dist = diff_pos.Norm();

    sum_pos_ += other_pos;
    sum_vel_ += other_vel;

    if (dist < 0.5 * boid_->GetPerceptionRadius()) {
      diff_pos /= pow(dist, 2);
      sum_diff_pos_ += diff_pos;
    }
    n++;
  }
}

Double3 CalculateNeighborData::GetAvgPosDirection() {
  if (n != 0) {
    sum_pos_ = sum_pos_ / static_cast<double>(n);
    sum_pos_ -= boid_pos_;
  }
  return sum_pos_;
}

Double3 CalculateNeighborData::GetDiffPos() {
  if (n != 0) {
    sum_diff_pos_ = sum_diff_pos_ / static_cast<double>(n);
  }
  return sum_diff_pos_;
}

Double3 CalculateNeighborData::GetAvgVel() {
  if (n != 0) {
    sum_vel_ = sum_vel_ / static_cast<double>(n);
  }
  return sum_vel_;
}

void Flocking::Run(Agent* agent) {
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