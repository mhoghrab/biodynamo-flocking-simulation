#include "boid.h"
#include "sim_param.h"

using namespace bdm;

void Boid::InitializeMembers() {
  const auto* param = Simulation::GetActive()->GetParam();
  const auto* sparam = param->Get<SimParam>();

  actual_diameter_ = sparam->actual_diameter;
  SetPerceptionRadius(sparam->perception_radius);
  double perception_angle_rad = (sparam->perception_angle_deg / 180) * M_PI;
  SetPerceptionAngle(perception_angle_rad);
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

  // Initialize Navigator
  gGeoManager->InitTrack(new_position_[0], new_position_[1], new_position_[2],
                         heading_direction_[0], heading_direction_[1],
                         heading_direction_[2]);
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
  if ((point - GetPosition()).Norm() == 0) {
    return true;
  }

  Double3 cone_normal = heading_direction_;
  Double3 direction_normal = (point - GetPosition()).Normalize();
  double cos_angle = cone_normal * direction_normal;

  if (cos_angle >= cos_perception_angle_) {
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
  double get_away_velocity = 0.4 * min_speed_;
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
    return {0, 0, 0};
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

Double3 Boid::ObstacleAvoidance() {
  // Update navigator
  Double3 position = GetPosition();
  gGeoManager->SetCurrentPoint(position[0], position[1], position[2]);
  gGeoManager->SetCurrentDirection(heading_direction_[0], heading_direction_[1],
                                   heading_direction_[2]);

  gGeoManager->FindNextBoundary(obst_avoid_dist_);
  if (gGeoManager->GetStep() == obst_avoid_dist_) {
    // heading_direction_ is clear
    return {0, 0, 0};
  } else {
    return SteerTowards(GetUnobstructedDirection());
  }
}

Double3 Boid::GetUnobstructedDirection() {
  double viewing_distance = 400;
  Double3 best_direction;
  double furthest_unobstructed_distance = 0;

  std::vector<Double3> directions =
      GetTransformedDirections(heading_direction_);
  Double3 position = GetPosition();

  for (int i = 0; i < (int)directions.size(); i++) {
    gGeoManager->SetCurrentPoint(position[0], position[1], position[2]);
    gGeoManager->SetCurrentDirection(directions[i][0], directions[i][1],
                                     directions[i][2]);
    gGeoManager->FindNextBoundary(viewing_distance);
    double step = gGeoManager->GetStep();
    if (step == viewing_distance) {
      // no obstacle within viewing_distance in direction
      return directions[i];
    } else {
      // obstacle is in direction
      if (step > furthest_unobstructed_distance) {
        furthest_unobstructed_distance = step;
        best_direction = directions[i];
      }
    }
  }
  // no clear direction is found, return best_direction
  return best_direction;
}

std::vector<Double3> Boid::GetTransformedDirections(Double3 heading_direction) {
  // max_idx is the maximal index of the original directions that still is in
  // viewing cone of a boid
  int n_dir = directions_.size();
  int max_idx = std::floor((n_dir / 2) * (1 - cos_perception_angle_));
  std::vector<Double3> directions_aligned = directions_;
  directions_aligned.resize(max_idx);

  // calculate the rotation axis (= crossprodukt directions_[0] x
  // heading_direction_) and the rotation angle
  // only works if the two vectors are not parallel, so check for that first
  double angle = acos(directions_[0] * heading_direction_);
  if (angle < 0.001) {
    if (signbit(heading_direction_[2]) == 0) {
      // heading_direction_ and directions_[0] show in the same direction since
      // directions_ = {0,0,1} and heading_direction_[2] > 0
      return directions_aligned;
    } else {
      // heading_direction_ and directions_[0] show in opposite directions
      for (int i = 0; i < max_idx; i++)
        directions_aligned[i] = directions_aligned[i] * -1;
      return directions_aligned;
    }
  }

  Double3 axis = {-heading_direction_[1], heading_direction_[0], 0};
  axis = axis.Normalize();
  double cos_half_angle = cos(angle / 2);
  double sin_half_angle = sin(angle / 2);

  //// here quaternion rotation is used to rotates directions_[0] onto
  //// heading_directionand and apply the same rotation to the other directions
  // define the Quaternion responible for aligning the vectors and define the
  // rotation matrix [col_0, col_1, col_2]
  Double4 Quat = {cos_half_angle, axis[0] * sin_half_angle,
                  axis[1] * sin_half_angle, axis[2] * sin_half_angle};

  Double3 col_0 = {1 - 2 * Quat[2] * Quat[2] - 2 * Quat[3] * Quat[3],
                   2 * Quat[1] * Quat[2] + 2 * Quat[0] * Quat[3],
                   2 * Quat[1] * Quat[3] - 2 * Quat[0] * Quat[2]};
  Double3 col_1 = {2 * Quat[1] * Quat[2] - 2 * Quat[0] * Quat[3],
                   1 - 2 * Quat[1] * Quat[1] - 2 * Quat[3] * Quat[3],
                   2 * Quat[2] * Quat[3] + 2 * Quat[0] * Quat[1]};
  Double3 col_2 = {2 * Quat[1] * Quat[3] + 2 * Quat[0] * Quat[2],
                   2 * Quat[2] * Quat[3] - 2 * Quat[0] * Quat[1],
                   1 - 2 * Quat[1] * Quat[1] - 2 * Quat[2] * Quat[2]};

  // rotation matrix * directions[i] to transform all directions to the boids
  // reference system

  for (int i = 0; i < max_idx; i++) {
    directions_aligned.at(i) = col_0 * directions_[i][0] +
                               col_1 * directions_[i][1] +
                               col_2 * directions_[i][2];
  }
  return directions_aligned;
}

std::vector<Double3> GetDirections() {
  // using the golden spiral method to generate "evenly" distributed points on
  // the unit sphere
  // directions[0] = {0,0,1};  the angle to this starting
  // point increases uniformly with the index

  double golden_ratio = (1 + sqrt(5)) / 2;
  double angle_inc = M_PI * 2 * golden_ratio;
  int n_dir = 200;

  std::vector<Double3> directions(n_dir);

  for (int i = 0; i < n_dir; i++) {
    double t = (double)i / (double)n_dir;
    double phi = acos(1 - 2 * t);
    double theta = angle_inc * i;

    double x = sin(phi) * cos(theta);
    double y = sin(phi) * sin(theta);
    double z = 1 - 2 * t;  // = cos(phi);
    directions.at(i) = {x, y, z};
  }
  return directions;
}

const std::vector<Double3> Boid::directions_ = GetDirections();

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

  // Calculate seperation_force, alignment_force, cohesion_force,
  // avoid_domain_boundary_force
  CalculateNeighborData NeighborData(boid);
  ctxt->ForEachNeighbor(NeighborData, *boid, perception_radius_squared);

  Double3 seperation_force = boid->SteerTowards(NeighborData.GetDiffPos());
  Double3 alignment_force = boid->SteerTowards(NeighborData.GetAvgVel());
  Double3 cohesion_force =
      boid->SteerTowards(NeighborData.GetAvgPosDirection());
  Double3 avoid_domain_boundary_force = boid->AvoidDomainBoundary();
  Double3 avoid_obstacle_force = boid->ObstacleAvoidance();

  // Update acceleration_, new_velocity_, new_position_ of boid
  boid->ResetAcceleration();
  boid->AccelerationAccumulator(seperation_force * boid->seperation_weight_);
  boid->AccelerationAccumulator(alignment_force * boid->alignment_weight_);
  boid->AccelerationAccumulator(cohesion_force * boid->cohesion_weight_);
  boid->AccelerationAccumulator(avoid_domain_boundary_force *
                                boid->avoid_domain_boundary_weight_);
  boid->AccelerationAccumulator(avoid_obstacle_force *
                                boid->obstacle_avoidance_weight_);

  boid->UpdateNewVelocity();
  boid->UpdateNewPosition();
}