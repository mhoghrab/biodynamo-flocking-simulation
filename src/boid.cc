#include "boid.h"
#include "sim_param.h"

namespace bdm {

////////////////////////////////////////////////////////////////////////////////
// Boid Class
////////////////////////////////////////////////////////////////////////////////

void Boid::InitializeMembers() {
  const auto* param = Simulation::GetActive()->GetParam();
  const auto* sparam = param->Get<SimParam>();

  actual_diameter_ = sparam->actual_diameter;
  SetPerceptionRadius(sparam->perception_radius);
  neighbor_distance_ = sparam->neighbor_distance;
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
  obst_avoid_dist_ = sparam->obst_avoid_dist;

  SetNewPosition(GetPosition());
  SetNewVelocity(GetVelocity());

  navig_ = gGeoManager->AddNavigator();
};

// ---------------------------------------------------------------------------
// Double3 helper functions

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

// ---------------------------------------------------------------------------

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

Double3 Boid::SteerTowards(Double3 vector) {
  if (vector.Norm() == 0) {
    return {0, 0, 0};
  }
  Double3 steer = vector.Normalize() * crusing_speed_ - velocity_;
  return UpperLimit(steer, max_force_);
}

// ---------------------------------------------------------------------------
// Data Updates

void Boid::UpdateNewPosition() {
  new_position_ += new_velocity_;
  // new_position_ = UpdatePositionTorus(new_position_);
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

Double3 Boid::UpdatePositionTorus(Double3 position) {
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

// ---------------------------------------------------------------------------
// Obstacle Avoidance

Double3 Boid::ObstacleAvoidance() {
  DirectionIsUnobstructed(heading_direction_, GetPosition(), obst_avoid_dist_);
  if (DirectionIsUnobstructed(heading_direction_, GetPosition(),
                              obst_avoid_dist_)) {
    // heading_direction_ is clear
    return {0, 0, 0};
  } else {
    return SteerTowards(GetUnobstructedDirection());
  }
}

bool Boid::DirectionIsUnobstructed(Double3 direction, Double3 position,
                                   double distance) {
  // navig_->SetCurrentPoint(position[0], position[1], position[2]);
  // navig_->SetCurrentDirection(direction[0], direction[1], direction[2]);
  navig_->InitTrack(position[0], position[1], position[2], direction[0],
                    direction[1], direction[2]);
  navig_->FindNextBoundary(distance);
  if (navig_->GetStep() == distance) {
    return true;
  } else {
    return false;
  }
}

Double3 Boid::GetUnobstructedDirection() {
  Double3 best_direction;
  double furthest_unobstructed_distance = 0;

  std::vector<Double3> directions =
      TransformDirections(directions_, heading_direction_, directions_[0]);

  for (int i = 0; i < (int)directions.size(); i++) {
    if (DirectionIsUnobstructed(directions[i], GetPosition(),
                                obst_avoid_dist_)) {
      // no obstacle within obst_avoid_dist_ in direction
      return directions[i];
    } else {
      // obstacle is in direction
      double step = navig_->GetStep();
      if (step > furthest_unobstructed_distance) {
        furthest_unobstructed_distance = step;
        best_direction = directions[i];
      }
    }
  }
  // no clear direction is found, return best_direction
  return best_direction;
}

std::vector<Double3> Boid::TransformDirections(std::vector<Double3> directions,
                                               Double3 ref_A, Double3 ref_B) {
  // rotates ref_A onto ref_B and applies same rotaion onto all vectors in
  // directions
  int n_dir = directions.size();
  std::vector<Double3> directions_aligned(n_dir);

  // calculate the angle betwenn ref_A and ref_B used for rotation
  double angle = acos(ref_A * ref_B);

  double thresh = 0.001;
  // test if ref_A and ref_B are parallel
  if (angle < thresh) {
    directions_aligned = directions;
    return directions_aligned;
  }
  // test if ref_A and ref_B are anti-parallel
  if (angle - M_PI < thresh) {
    for (int i = 0; i < n_dir; i++)
      directions_aligned[i] = directions[i] * -1;
    return directions_aligned;
  }

  // calculate the rotation axis (= crossprodukt ref_A x ref_B)
  Double3 axis = {ref_A[1] * ref_B[2] - ref_A[2] * ref_B[1],
                  ref_A[2] * ref_B[0] - ref_A[0] * ref_B[2],
                  ref_A[0] * ref_B[1] - ref_A[1] * ref_B[0]};
  axis = axis.Normalize();
  double cos_half_angle = cos(angle / 2);
  double sin_half_angle = sin(angle / 2);

  // quaternion rotation is used to calculate the rotation matrix;
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

  for (int i = 0; i < n_dir; i++) {
    directions_aligned.at(i) = col_0 * directions[i][0] +
                               col_1 * directions[i][1] +
                               col_2 * directions[i][2];
  }
  return directions_aligned;
}

// initializing the static const direction-"arrays"
std::vector<Double3> GetDirections() {
  // using the golden spiral method to generate "evenly" distributed points on
  // the unit sphere
  // directions[0] = {0,0,1};  the angle towards this starting
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

std::vector<Double3> GetConeDirections() {
  // ....
  double cone_angle_deg = 20;
  double cone_angle_rad = (cone_angle_deg / 180) * M_PI;
  int n_dir = 20;

  std::vector<Double3> directions(n_dir);

  for (int i = 0; i < n_dir; i++) {
    double phi = i * 2 * M_PI / n_dir;
    double x = cos(phi);
    double y = sin(phi);
    double z = 1 / tan(cone_angle_rad);

    Double3 direction = {x, y, z};
    directions.at(i) = direction.Normalize();
  }
  return directions;
}
const std::vector<Double3> Boid::cone_directions_ = GetConeDirections();

////////////////////////////////////////////////////////////////////////////////
// Flocking Behaviour
////////////////////////////////////////////////////////////////////////////////

void Flocking::Run(Agent* agent) {
  auto* boid = dynamic_cast<Boid*>(agent);
  auto* ctxt = Simulation::GetActive()->GetExecutionContext();

  double perception_radius = boid->GetPerceptionRadius();
  double perception_radius_squared = perception_radius * perception_radius;

  // Calculate seperation_force, alignment_force, cohesion_force,
  // avoid_domain_boundary_force
  CalculateNeighborData NeighborData(boid);
  ctxt->ForEachNeighbor(NeighborData, *boid, perception_radius_squared);

  Double3 cohesion_force =
      boid->SteerTowards(NeighborData.GetCenterOfMassDir());
  Double3 seperation_force =
      boid->SteerTowards(NeighborData.GetSeperationDir());
  Double3 alignment_force = boid->SteerTowards(NeighborData.GetAvgVel());
  Double3 avoid_domain_boundary_force = boid->AvoidDomainBoundary();
  Double3 avoid_obstacle_force = boid->ObstacleAvoidance();

  Double3 social_force = boid->SteerTowards(NeighborData.GetCenterOfMassDir() +
                                            NeighborData.GetSeperationDir());

  // Update acceleration_, new_velocity_, new_position_ of boid
  boid->ResetAcceleration();

  boid->AccelerationAccumulator(seperation_force * boid->seperation_weight_);
  boid->AccelerationAccumulator(cohesion_force * boid->cohesion_weight_);
  // boid->AccelerationAccumulator(social_force * 5);
  boid->AccelerationAccumulator(alignment_force * boid->alignment_weight_);
  boid->AccelerationAccumulator(avoid_domain_boundary_force *
                                boid->avoid_domain_boundary_weight_);
  boid->AccelerationAccumulator(avoid_obstacle_force *
                                boid->obstacle_avoidance_weight_);

  boid->UpdateNewVelocity();
  boid->UpdateNewPosition();
}

void CalculateNeighborData::operator()(Agent* neighbor,
                                       double squared_distance) {
  auto* neighbor_boid = bdm_static_cast<const Boid*>(neighbor);
  Double3 neighbor_position = neighbor_boid->GetPosition();

  // check if neighbor is in viewing cone
  if (boid_->CheckIfVisible(neighbor_position)) {
    Double3 neighbor_velocity = neighbor_boid->GetVelocity();
    Double3 diff_pos = boid_position_ - neighbor_position;
    double dist = diff_pos.Norm();

    // Cohesion Data
    sum_position_ += neighbor_position;

    // Alignment Data
    sum_vel_ += neighbor_velocity;

    // Seperation Data
    if (dist < 0.5 * boid_->GetPerceptionRadius()) {
      diff_pos /= pow(dist, 2);
      sum_diff_pos_ += diff_pos;
    }

    double w_s = 0.6, r_s = boid_->actual_diameter_ * 2;
    double sep_weight = 1 / (1 + (exp(w_s * (dist - r_s))));
    sum_seperation_dir_exp += (diff_pos / dist) * sep_weight;

    n++;
  }
}

Double3 CalculateNeighborData::GetCenterOfMassDir() {
  if (n != 0)
    return (sum_position_ / static_cast<double>(n) - boid_position_) / 1;
  else
    return sum_position_;  // = {0,0,0}
}

Double3 CalculateNeighborData::GetSeperationDir() {
  if (n != 0)
    return sum_diff_pos_ / static_cast<double>(n);
  else
    return sum_diff_pos_;  // = {0,0,0}
}

Double3 CalculateNeighborData::GetSeperationDir_Exp() {
  if (n != 0)
    return sum_seperation_dir_exp / static_cast<double>(n);
  else
    return sum_seperation_dir_exp;  // = {0,0,0}
}

Double3 CalculateNeighborData::GetAvgVel() {
  if (n != 0)
    return sum_vel_ / static_cast<double>(n);
  else
    return sum_vel_;  // = {0,0,0}
}

////////////////////////////////////////////////////////////////////////////////
// Flocking2 Behaviour
////////////////////////////////////////////////////////////////////////////////

void Flocking2::Run(Agent* agent) {
  auto* boid = dynamic_cast<Boid*>(agent);
  auto* ctxt = Simulation::GetActive()->GetExecutionContext();

  double perception_radius = boid->GetPerceptionRadius();
  double perception_radius_squared = perception_radius * perception_radius;

  CalculateNeighborData2 NeighborData(boid);
  ctxt->ForEachNeighbor(NeighborData, *boid, perception_radius_squared);

  Double3 acceleration = NeighborData.GetProtocol1();
  Double3 avoid_domain_boundary_force = boid->AvoidDomainBoundary();
  Double3 avoid_obstacle_force = boid->ObstacleAvoidance();

  boid->ResetAcceleration();
  // std::cout << acceleration.Norm() << std::endl;

  boid->AccelerationAccumulator(acceleration * 0.1);
  // boid->AccelerationAccumulator(avoid_domain_boundary_force *
  //                               boid->avoid_domain_boundary_weight_);
  // boid->AccelerationAccumulator(avoid_obstacle_force *
  //                               boid->obstacle_avoidance_weight_);

  boid->UpdateNewVelocity();
  boid->UpdateNewPosition();
}

void CalculateNeighborData2::operator()(Agent* neighbor,
                                        double squared_distance) {
  auto* neighbor_boid = bdm_static_cast<const Boid*>(neighbor);
  Double3 q_j = neighbor_boid->GetPosition();
  Double3 p_j = neighbor_boid->GetVelocity();

  // check if neighbor is in viewing cone
  if (boid_->CheckIfVisible(q_j)) {
    // add gradient-based term to u_i_a
    Double3 n_ij = (q_j - q_i) / sqrt(1 + eps * pow((q_j - q_i).Norm(), 2));
    u_i_a += n_ij * Phi_a(Norm_sig(q_j - q_i));

    // add consensus term
    double z2 = Norm_sig(boid_->perception_radius_);
    double r_a2 = (sqrt(1 + eps * z2 * z2) - 1) / eps;
    double a_ij = phi_h(Norm_sig(q_j - q_i) / r_a2);
    u_i_a += (p_j - p_i) * a_ij;
  }
}

Double3 CalculateNeighborData2::GetProtocol1() {
  return u_i_a;
  // return boid_->UpperLimit(u_i_a, boid_->max_force_);
}

Double3 CalculateNeighborData2::GetProtocol2() {
  double c_1 = 0.01, c_2 = 0.01;
  /////////////////////////////////////////////// 2do2dod2odo2doo
  Double3 q_r = {750, 750, 1500}, p_r = {0, 0, 0};
  Double3 u_i_y = (q_i - q_r) * (-c_1) + (p_i - p_r) * (-c_2);
  return u_i_a + u_i_y;
}

double CalculateNeighborData2::Norm_sig(Double3 z) {
  return (sqrt(1 + eps * pow(z.Norm(), 2)) - 1) / eps;
}

double CalculateNeighborData2::Norm_sig(double z) {
  double result = (sqrt(1 + eps * z * z) - 1) / eps;
  return result;
}

double CalculateNeighborData2::Phi_a(double z) {
  // std::cout << z / r_a << std::endl;
  // std::cout << r_a << std::endl;
  double z2 = Norm_sig(boid_->perception_radius_);
  double r_a2 = (sqrt(1 + eps * z2 * z2) - 1) / eps;
  return phi_h(z / r_a2) * Phi(z - d_a);
}

double CalculateNeighborData2::Phi(double z) {
  // 0 < a <= b
  double a = 0.5;
  double b = 0.5;
  double c = abs(a - b) / sqrt(4 * a * b);
  return ((a + b) * sigma_1(z + c) + (a - b)) / 2;
}

double CalculateNeighborData2::phi_h(double z) {
  double h = 0.2;
  if (z >= 0 && z < h) {
    return 1;
  }
  if (z >= h && z <= 1) {
    return (1 + cos(M_PI * (z - h) / (1 - h))) / 2;
  }
  return 0;
}

double CalculateNeighborData2::sigma_1(double z) { return z / sqrt(1 + z * z); }

}  // namespace bdm