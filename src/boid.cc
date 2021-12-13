#include "boid.h"
#include <cnpy.h>
#include "core/container/math_array.h"
#include "sim_param.h"

namespace bdm {

// ---------------------------------------------------------------------------
// Double3 Methods
double NormSq(Double3 vector) {
  double res = 0;
  for (size_t i = 0; i < 3; i++)
    res += vector[i] * vector[i];
  return res;
}

Double3 UpperLimit(Double3 vector, double upper_limit) {
  double length = vector.Norm();
  if (length == 0) {
    return {0, 0, 0};
  }
  if (length > upper_limit) {
    vector = (vector / length) * upper_limit;
  }
  return vector;
}

Double3 LowerLimit(Double3 vector, double lower_limit) {
  double length = vector.Norm();
  if (length == 0) {
    return {0, 0, 0};
  }
  if (length < lower_limit) {
    vector = (vector / length) * lower_limit;
  }
  return vector;
}

Double3 ClampUpperLower(Double3 vector, double upper_limit,
                        double lower_limit) {
  double length = vector.Norm();
  if (length == 0) {
    return {0, 0, 0};
  }
  if (length > upper_limit) {
    vector = (vector / length) * upper_limit;
  }
  if (length < lower_limit) {
    vector = (vector / length) * lower_limit;
  }
  return vector;
}

Double3 GetNormalizedArray(Double3 vector) {
  if (vector.Norm() == 0)
    return {0, 0, 0};
  else
    return vector.GetNormalizedArray();
}

Double3 GetRandomVectorInUnitSphere() {
  auto* random = Simulation::GetActive()->GetRandom();

  double phi = random->Uniform(0, 2 * M_PI);
  double costheta = random->Uniform(-1, 1);
  double u = random->Uniform(0, 1);

  double theta = acos(costheta);
  double r = sqrt(u);

  double x_coord = r * sin(theta) * cos(phi);
  double y_coord = r * sin(theta) * sin(phi);
  double z_coord = r * cos(theta);

  Double3 vec = {x_coord, y_coord, z_coord};
  return vec;

  // bool found = false;
  // while (!found) {
  //   double x = random->Uniform(-1, 1);
  //   double y = random->Uniform(-1, 1);
  //   double z = random->Uniform(-1, 1);
  //   Double3 vec = {x, y, z};

  //   if (vec.Norm() <= 1) {
  //     found = true;
  //     return vec;
  //   }
  // }
}

////////////////////////////////////////////////////////////////////////////////
//----------------------------------------------------------------------------//
// Boid Class                                                                 //
//----------------------------------------------------------------------------//
////////////////////////////////////////////////////////////////////////////////

void Boid::InitializeMembers() {
  const auto* param = Simulation::GetActive()->GetParam();
  const auto* sparam = param->Get<SimParam>();

  SetNewPosition(GetPosition());
  SetNewVelocity(GetVelocity());

  actual_diameter_ = sparam->actual_diameter;

  SetBoidPerceptionRadius(sparam->boid_perception_radius);
  boid_interaction_radius_ = sparam->boid_interaction_radius;
  obstacle_perception_radius_ = sparam->obstacle_perception_radius;

  SetPerceptionAngle((sparam->perception_angle_deg / 180) * M_PI);

  boid_distance_ = sparam->neighbor_distance;
  obstacle_distance_ = sparam->obstacle_distance;

  max_accel_ = sparam->max_accel;
  max_speed_ = sparam->max_speed;

  obstacles_obstruct_view_ = sparam->obstacles_obstruct_view;

  navig_ = gGeoManager->AddNavigator();

  // Flocking constants
  c_a_1_ = sparam->c_a_1;
  c_a_2_ = sparam->c_a_1;
  c_b_1_ = sparam->c_b_1;
  c_b_2_ = sparam->c_b_2;
  c_y_ = sparam->c_y;
  c_a_3_ = sparam->c_a_3;
  eps_ = sparam->eps;
  h_a_ = sparam->h_a;
  h_b_ = sparam->h_b;

  pos_gamma_ = sparam->pos_gamma;
};

// ---------------------------------------------------------------------------
// Define necessary virtual functions of Base class

Shape Boid::GetShape() const { return Shape::kSphere; }

Double3 Boid::CalculateDisplacement(const InteractionForce* force,
                                    double squared_radius, double dt) {
  Double3 zero = {0, 0, 0};
  return zero;
};

void Boid::ApplyDisplacement(const Double3& displacement) { ; };

const Double3& Boid::GetPosition() const { return position_; };

void Boid::SetPosition(const Double3& pos) { position_ = pos; };

double Boid::GetDiameter() const { return diameter_; };

void Boid::SetDiameter(double diameter) { diameter_ = diameter; };

// ---------------------------------------------------------------------------
// Important Setter that have to update other variables as well

Double3 Boid::GetVelocity() const { return velocity_; }

void Boid::SetVelocity(Double3 velocity) {
  velocity_ = velocity;
  if (velocity_.Norm() != 0) {
    heading_direction_ = velocity_.GetNormalizedArray();
  }
}

void Boid::SetNewPosition(Double3 position) { new_position_ = position; }

void Boid::SetNewVelocity(Double3 velocity) { new_velocity_ = velocity; }

void Boid::SetBoidPerceptionRadius(double perception_radius) {
  boid_perception_radius_ = perception_radius;
  SetDiameter(boid_perception_radius_ * 2);
}

void Boid::SetPerceptionAngle(double angle) {
  perception_angle_ = angle;
  cos_perception_angle_ = std::cos(angle);
}

void Boid::SetHeadingDirection(Double3 dir) {
  if (dir.Norm() != 0) {
    heading_direction_ = GetNormalizedArray(dir);
  }
}
// ---------------------------------------------------------------------------

bool Boid::CheckIfVisible(Double3 point) {
  if ((point - GetPosition()).Norm() == 0) {
    // identical points
    return true;
  }

  Double3 cone_normal = heading_direction_;
  Double3 direction_normal = (point - GetPosition()).GetNormalizedArray();
  double cos_angle = cone_normal * direction_normal;

  // check if obstacle obstructs view to point
  bool is_unobstructed = true;
  if (obstacles_obstruct_view_) {
    is_unobstructed = DirectionIsUnobstructed(
        point - GetPosition(), GetPosition(), (point - GetPosition()).Norm());
  }

  if (is_unobstructed && cos_angle >= cos_perception_angle_) {
    return true;
  } else {
    return false;
  }
}

Double3 Boid::SteerTowards(Double3 vector) {
  if (vector.Norm() == 0) {
    return {0, 0, 0};
  }
  Double3 steer = vector.GetNormalizedArray() * max_speed_ - velocity_;
  return steer;
  return UpperLimit(steer, max_accel_);
}

bool Boid::DirectionIsUnobstructed(Double3 direction, Double3 position,
                                   double distance) {
  navig_->InitTrack(position[0], position[1], position[2], direction[0],
                    direction[1], direction[2]);
  navig_->FindNextBoundary(distance);
  if (navig_->GetStep() == distance) {
    return true;
  } else {
    return false;
  }
}

// ---------------------------------------------------------------------------
// Data Updates

void Boid::UpdateNewPosition() {
  const auto* param = Simulation::GetActive()->GetParam();
  const auto* sparam = param->Get<SimParam>();

  new_position_ += new_velocity_ * sparam->d_t;
}

void Boid::UpdateNewVelocity() {
  const auto* param = Simulation::GetActive()->GetParam();
  const auto* sparam = param->Get<SimParam>();

  acceleration_ = UpperLimit(acceleration_, max_accel_);
  new_velocity_ += acceleration_ * sparam->d_t;
  if (sparam->limit_speed) {
    new_velocity_ = UpperLimit(new_velocity_, max_speed_);
  }
}

void Boid::ResetAcceleration() {
  acceleration_ = {0, 0, 0};
  acc_scalar_ = 0;
}

void Boid::UpdateData() {
  SetVelocity(new_velocity_);
  SetPosition(new_position_);
}

void Boid::AccelerationAccumulator(Double3 acceleration_to_add) {
  size_t nr = 2;

  switch (nr) {
    case 0:
      acceleration_ += acceleration_to_add;
    case 1:
      if (acceleration_to_add.Norm() == 0)
        return;
      else if ((acceleration_ + acceleration_to_add).Norm() <= max_accel_) {
        acceleration_ += acceleration_to_add;
      } else {
        // top up acceleration_ with acceleration_to_add until length equals
        // max_accel_
        double a = acceleration_to_add * acceleration_to_add,
               b = acceleration_ * acceleration_to_add * 2,
               c = acceleration_ * acceleration_ - max_accel_ * max_accel_;
        double lambda = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);

        acceleration_ += acceleration_to_add * lambda;
      }
    case 2:
      if (acc_scalar_ + acceleration_to_add.Norm() <= max_accel_) {
        acc_scalar_ += acceleration_to_add.Norm();
        acceleration_ += acceleration_to_add;
      } else {
        double s = max_accel_ - acc_scalar_;
        acc_scalar_ = max_accel_;
        acceleration_ += acceleration_to_add * s;
      }
  }
}

// ---------------------------------------------------------------------------
// Flocking Algorithm

Double3 Boid::GetFlockingForce() {
  auto* ctxt = Simulation::GetActive()->GetExecutionContext();
  CalculateNeighborData NeighborData(this);
  ctxt->ForEachNeighbor(NeighborData, *this, pow(boid_perception_radius_, 2));

  Double3 force = {0, 0, 0};
  force += NeighborData.GetU_a();
  force += GetExtendedCohesionTerm(NeighborData.GetCentreOfMass());

  return force;
}

Double3 Boid::GetObstacleAvoidanceForce() {
  Double3 force = {0, 0, 0};

  for (auto sphere : SphereObstacle::sphere_obstacles) {
    force += GetSphereInteractionTerm(&sphere);
  }

  for (auto cuboid : CuboidObstacle::cuboid_obstacles) {
    force += GetCuboidInteractionTerm(&cuboid);
  }

  return force;
}

Double3 Boid::GetNavigationalFeedbackForce() {
  // Double3 target_vel = {0, 0, 0};
  // double c_1 = 1, c_2 = 0;
  // Double3 u_y =
  //     (target_pos - GetPosition()) * c_1 + (target_vel - GetVelocity()) *
  //     c_2;
  // return u_y;
  return SteerTowards(pos_gamma_ - GetPosition()) * c_y_;
}

Double3 Boid::GetExtendedCohesionTerm(Double3 centre_of_mass) {
  double ratio =
      (centre_of_mass - GetPosition()).Norm() / boid_perception_radius_;
  double h_1 = boid_interaction_radius_ / boid_perception_radius_;
  double h_2 = h_1 * 1.2;

  double scale = zeta(ratio, h_1, h_2);

  Double3 result =
      GetNormalizedArray(centre_of_mass - GetPosition()) * scale * c_a_3_;
  return result;
}

Double3 Boid::GetBoidInteractionTerm(const Boid* boid) {
  Double3 u_a = {0, 0, 0};

  // add gradient-based term to u_a
  // double temp = (boid->GetPosition() - GetPosition()).Norm();
  // Double3 temp2 = (boid->GetPosition() - GetPosition());
  // Double3 n_ij = temp2 / sqrt(1 + eps_ * pow(temp, 2));
  Double3 n_ij = GetNormalizedArray(boid->GetPosition() - GetPosition());

  u_a += n_ij * Phi_a(Norm_sig(boid->GetPosition() - GetPosition())) * c_a_1_;

  // add consensus term
  double r_a = Norm_sig(boid_interaction_radius_);
  double a_ij =
      rho_h(Norm_sig(boid->GetPosition() - GetPosition()) / r_a, h_a_);

  u_a += (boid->GetVelocity() - GetVelocity()) * a_ij * c_a_2_;

  return u_a;
}

Double3 Boid::GetSphereInteractionTerm(SphereObstacle* sphere) {
  Double3 u_b = {0, 0, 0};
  Double3 q_ik = GetProjectedPosition(sphere);

  // test if sphere is in obstacle_perception_radius_ and boid has not clipped
  // into it and is not heading away from projected position
  if ((GetPosition() - q_ik).Norm() <= obstacle_perception_radius_ &&
      (GetPosition() - sphere->centre_).Norm() >= sphere->radius_ &&
      IsHeadingTowards(q_ik)) {
    // ---------------------------------------------------------------------------
    // distance term
    // Double3 n_ik = (q_ik - GetPosition()) /
    //                sqrt(1 + eps_ * pow((q_ik - GetPosition()).Norm(), 2));
    Double3 n_ik = GetNormalizedArray(q_ik - GetPosition());
    u_b += n_ik * Phi_b(Norm_sig(q_ik - GetPosition())) * c_b_1_;

    // ---------------------------------------------------------------------------
    // velocity term
    double b_ik = rho_h(
        Norm_sig(q_ik - GetPosition()) / Norm_sig(obstacle_perception_radius_),
        h_b_);
    u_b += (GetProjectedVelocity(sphere) - GetVelocity()) * b_ik * c_b_2_;
  }

  return u_b;
}

Double3 Boid::GetCuboidInteractionTerm(CuboidObstacle* cuboid) {
  Double3 u_b = {0, 0, 0};
  Double3 q_ik = GetProjectedPosition(cuboid);

  // test if cuboid is in obstacle_perception_radius_ and boid has not clipped
  // into it and is not heading away from projected position
  if ((GetPosition() - q_ik).Norm() <= obstacle_perception_radius_ &&
      (GetPosition() - q_ik).Norm() != 0 && IsHeadingTowards(q_ik)) {
    // ---------------------------------------------------------------------------
    // distance term
    // Double3 n_ik = (q_ik - GetPosition()) /
    //                sqrt(1 + eps_ * pow((q_ik - GetPosition()).Norm(), 2));
    Double3 n_ik = GetNormalizedArray((q_ik - GetPosition()));
    u_b += n_ik * Phi_b(Norm_sig(q_ik - GetPosition())) * c_b_1_;

    // ---------------------------------------------------------------------------
    // velocity term
    // obstacle_perception_radius_ or obstacle_distance_????
    double b_ik = rho_h(
        Norm_sig(q_ik - GetPosition()) / Norm_sig(obstacle_perception_radius_),
        h_b_);
    u_b += (GetProjectedVelocity(cuboid) - GetVelocity()) * b_ik * c_b_2_;
  }

  return u_b;
}

Double3 Boid::GetProjectedPosition(SphereObstacle* sphere) {
  double dist = (GetPosition() - sphere->centre_).Norm();

  if (dist > sphere->radius_) {
    // outside of sphere
    double my = sphere->radius_ / dist;
    return GetPosition() * my + sphere->centre_ * (1 - my);
  } else {
    // inside of sphere
    return GetPosition();
  }
}

Double3 Boid::GetProjectedPosition(CuboidObstacle* cuboid) {
  Double3 projected_position;

  for (int i = 0; i < 3; i++) {
    if (GetPosition()[i] < (cuboid->lower_bound_)[i])
      projected_position[i] = (cuboid->lower_bound_)[i];
    else if (GetPosition()[i] > (cuboid->upper_bound_)[i])
      projected_position[i] = (cuboid->upper_bound_)[i];
    else
      projected_position[i] = GetPosition()[i];
  }

  return projected_position;
}

Double3 Boid::GetProjectedVelocity(SphereObstacle* sphere) {
  // double my = sphere->radius_ / (GetPosition() - sphere->centre_).Norm();
  Double3 a = GetPosition() - sphere->centre_;
  a = GetNormalizedArray(a);

  Double3 projected_normal = a * (a * velocity_);
  Double3 projected_velocity = (velocity_ - projected_normal);  // * my;

  return projected_velocity;
}

Double3 Boid::GetProjectedVelocity(CuboidObstacle* cuboid) {
  Double3 a = GetPosition() - GetProjectedPosition(cuboid);
  a = GetNormalizedArray(a);

  Double3 projected_normal = a * (a * velocity_);
  Double3 projected_velocity = (velocity_ - projected_normal);

  return projected_velocity;
}

bool Boid::IsHeadingTowards(Double3 point) {
  return (((point - GetPosition()) * velocity_) > 0);
}

double Boid::Norm_sig(Double3 z) {
  return (std::sqrt(1 + eps_ * NormSq(z)) - 1) / eps_;
}

double Boid::Norm_sig(double z) {
  double result = (std::sqrt(1 + eps_ * z * z) - 1) / eps_;
  return result;
}

double Boid::Phi(double z) {
  // 0 < a <= b
  // "a" controls a max for attaction scaling, "b" min for repelling
  double a = 1;
  double b = 2.5;
  double c = std::abs(a - b) / std::sqrt(4 * a * b);
  return ((a + b) * sigmoid_2(z + c) + (a - b)) / 2;
}

double Boid::rho_h(double z, double h) {
  if (z >= 0 && z < h) {
    return 1;
  }
  if (z >= h && z <= 1) {
    return (1 + cos(M_PI * (z - h) / (1 - h))) / 2;
  }
  return 0;
}

double Boid::rho_h_a(double z, double h) {
  if (z >= 0 && z < h) {
    return 1;
  }
  if (z >= h && z <= 1) {
    double scale = exp(-5 * (z - h) * (z - h));
    return scale * (1 + cos(M_PI * (z - h) / (1 - h))) / 2;
  }
  return 0;
}

double Boid::zeta(double z, double h_onset, double h_maxeff) {
  if (z < h_onset) {
    return 0;
  } else if (z >= h_onset && z <= h_maxeff) {
    return (1 + cos(M_PI * (h_maxeff - z) / (h_maxeff - h_onset))) / 2;
  } else {
    return 1;
  }
  // } else if (z >= h_maxeff && z <= 1) {
  //   return 1;
  // } else if (z >= h_offset && z <= 1) {
  //   return (1 + cos(M_PI * (z - h_offset) / (1 - h_offset))) / 2;
  // } else {
  //   return 0;
  // }
}

double Boid::sigmoid_1(double z) { return z / std::sqrt(1 + z * z); }

double Boid::sigmoid_2(double z) { return z / (1 + std::abs(z)); }

double Boid::Phi_a(double z) {
  double r_a = Norm_sig(boid_interaction_radius_);
  double d_a = Norm_sig(boid_distance_);

  return rho_h_a(z / r_a, h_a_) * Phi(z - d_a);
}

double Boid::Phi_b(double z) {
  double d_b = Norm_sig(obstacle_distance_);

  return rho_h(z / d_b, h_b_) * (sigmoid_2(z - d_b) - 1) * 0.5;
}

// ---------------------------------------------------------------------------

cnpy::NpyArray arr = cnpy::npy_load("src/pywind/data/wind.npy");
double* wind_data = arr.data<double>();
size_t xdim_wind = arr.shape[0];
size_t ydim_wind = arr.shape[1];
size_t zdim_wind = arr.shape[2];
size_t vector_dim_wind = arr.shape[3];

Double3 Boid::CalculateWindForce() {
  auto* sparam = bdm::Simulation::GetActive()->GetParam()->Get<SimParam>();
  Double3 v_wind = GetWindVelocity();
  Double3 v_rel = v_wind - velocity_;

  Double3 f_wind = v_rel * v_rel.Norm() * sparam->c_wind_force;
  return f_wind;
}

Double3 Boid::GetWindVelocity() {
  auto* param = bdm::Simulation::GetActive()->GetParam();
  auto* sparam = param->Get<SimParam>();
  double max_bound = param->max_bound;
  double min_bound = param->min_bound;
  double d = max_bound - min_bound;

  auto factor_1 = ydim_wind * zdim_wind * vector_dim_wind;
  auto factor_2 = zdim_wind * vector_dim_wind;
  auto factor_3 = vector_dim_wind;

  Double3 coords_shifted = GetPosition();
  coords_shifted -= min_bound;
  coords_shifted[0] /= d / xdim_wind;
  coords_shifted[1] /= d / ydim_wind;
  coords_shifted[2] /= d / zdim_wind;

  // find corresponding grid index
  // we use floor instead of ceil as in the thesis since idices start at 0
  auto x_idx = (int)coords_shifted[0];
  auto y_idx = (int)coords_shifted[1];
  auto z_idx = (int)coords_shifted[2];
  // std::cout << x_idx << ',' << y_idx << ',' << z_idx << std::endl;
  // std::cout << GetPosition() << std::endl;

  Double3 wind_turb;
  for (int l = 0; l < 3; l++) {
    wind_turb[l] =
        wind_data[factor_1 * x_idx + factor_2 * y_idx + factor_3 * z_idx + l];
  }
  // std::cout << "iteration complete" << std::endl;

  // combine turbulence and mean wind field
  Double3 wind =
      wind_turb * sparam->c_wind_turb + sparam->wind_mean * sparam->c_wind_mean;

  return wind;
}

////////////////////////////////////////////////////////////////////////////////
//----------------------------------------------------------------------------//
// Flocking Behaviour                                                         //
//----------------------------------------------------------------------------//
////////////////////////////////////////////////////////////////////////////////

void Flocking::Run(Agent* agent) {
  auto* boid = dynamic_cast<Boid*>(agent);
  Double3 flocking_force = boid->GetFlockingForce();
  Double3 flocking_obstacle_avoidance_force = boid->GetObstacleAvoidanceForce();
  Double3 flocking_navigational_feedback_force =
      boid->GetNavigationalFeedbackForce();

  boid->AccelerationAccumulator(flocking_obstacle_avoidance_force);
  boid->AccelerationAccumulator(flocking_force);
  boid->AccelerationAccumulator(flocking_navigational_feedback_force);
}

void FreeSpaceFlocking::Run(Agent* agent) {
  auto* boid = dynamic_cast<Boid*>(agent);

  Double3 flocking_force = boid->GetFlockingForce();
  Double3 flocking_navigational_feedback_force =
      boid->GetNavigationalFeedbackForce();

  boid->AccelerationAccumulator(flocking_force);
  boid->AccelerationAccumulator(flocking_navigational_feedback_force);
}

void CalculateNeighborData::operator()(Agent* neighbor,
                                       double squared_distance) {
  auto* neighbor_boid = bdm_static_cast<const Boid*>(neighbor);
  double dist = (boid_->GetPosition() - neighbor_boid->GetPosition()).Norm();
  bool is_visible = boid_->CheckIfVisible(neighbor_boid->GetPosition());

  if (is_visible && dist <= boid_->boid_interaction_radius_) {
    u_a += boid_->GetBoidInteractionTerm(neighbor_boid);
  }

  // For getting the avarage position for all visible boids within
  // boid_perception_radius_
  if (is_visible && dist <= boid_->boid_perception_radius_) {
    sum_pos += neighbor_boid->GetPosition();
    n++;
  }
}

Double3 CalculateNeighborData::GetCentreOfMass() {
  if (n != 0)
    return (sum_pos / n);
  else
    return boid_->GetPosition();
}

void FlockingNeighborAnalysis::operator()(Agent* neighbor,
                                          double squared_distance) {
  auto* neighbor_boid = bdm_static_cast<const Boid*>(neighbor);
  double dist = (boid_->GetPosition() - neighbor_boid->GetPosition()).Norm();

  if (dist <= boid_->boid_interaction_radius_ &&
      boid_->CheckIfVisible(neighbor_boid->GetPosition())) {
    sum_dist_interacion_r += dist;
    n++;
  }
}

double FlockingNeighborAnalysis::GetAvgDist_InteractionR() {
  if (n != 0) {
    return sum_dist_interacion_r / n;
  } else {
    // no neighbor boids within boid_interaction_radius_
    // we mark it as -1 and discard this agent later in matlab when computing
    // the average over all agents for each simulationstep
    return -1;
  }
}

}  // namespace bdm
