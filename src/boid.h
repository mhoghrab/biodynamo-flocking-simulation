#ifndef BOID_H_
#define BOID_H_

#include "TGeoManager.h"
#include "core/behavior/behavior.h"
#include "core/container/math_array.h"
#include "core/functor.h"
#include "world_geometry.h"

namespace bdm {

// ---------------------------------------------------------------------------
// Double3 Methods

Double3 UpperLimit(Double3 vector, double upper_limit);

Double3 LowerLimit(Double3 vector, double lower_limit);

Double3 ClampUpperLower(Double3 vector, double upper_limit, double lower_limit);

Double3 GetNormalizedArray(Double3 vector);

////////////////////////////////////////////////////////////////////////////////
//----------------------------------------------------------------------------//
// Boid Class                                                                 //
//----------------------------------------------------------------------------//
////////////////////////////////////////////////////////////////////////////////

class Boid : public Agent {
  BDM_AGENT_HEADER(Boid, Agent, 1);

 private:
  Double3 position_;
  double diameter_;

 public:
  Boid() {}
  explicit Boid(const Double3& position)
      : position_(position), diameter_(1.0) {}
  virtual ~Boid() {}

  // Initializes Boid parameters with given SimParam
  void InitializeMembers();

  // ---------------------------------------------------------------------------
  // Define necessary virtual functions of Base class. Those functions are
  // called from BioDynaMo's main engine but we don't need that here. Thus,
  // the function return zero or are defined as an empty call.

  Shape GetShape() const override;

  Double3 CalculateDisplacement(const InteractionForce* force,
                                double squared_radius, double dt) override;

  void ApplyDisplacement(const Double3& displacement) override;

  const Double3& GetPosition() const override;

  void SetPosition(const Double3& pos) override;

  double GetDiameter() const override;

  void SetDiameter(double diameter) override;

  // ---------------------------------------------------------------------------
  // Important setter that have to update/initialize other variables in the
  // process as well

  Double3 GetVelocity() const;
  void SetVelocity(Double3 velocity);

  void SetNewPosition(Double3 position);

  void SetNewVelocity(Double3 velocity);

  void SetBoidPerceptionRadius(double perception_radius);

  void SetPerceptionAngle(double angle);

  // ---------------------------------------------------------------------------

  // Returns bool wether given point is visible by the boid
  // It checks if the point is inside the viewing cone defined by
  // heading_direction_ and perception_angle_
  bool CheckIfVisible(Double3 point);

  // Returns a Steering-Force in order to steer velocity towards
  // (GetNormalizedArray(vector) * crusing_speed_)
  // Force is limited by max_force_
  Double3 SteerTowards(Double3 vector);

  // Returns bool weather a (root) obstacle is in the given direction and within
  // distance from the postition vector
  bool DirectionIsUnobstructed(Double3 direction, Double3 position,
                               double distance);

  // ---------------------------------------------------------------------------
  // Data Updates

  // Update new_position_ by adding new_velocity_
  void UpdateNewPosition();

  // Update new_velocity_ by adding acceleration_ and clamping it by
  // max_speed_ and min_speed_
  void UpdateNewVelocity();

  // Sets acceleration_ to {0,0,0}
  void ResetAcceleration();

  // Sets the actual position / velocity to new_position_ / new_velocity_
  void UpdateData();

  // Right now simply adds acc2add to the stored acceleration_
  void AccelerationAccumulator(Double3 acceleration_to_add);

  // Returns the position vector, but if a coordinate exceeds the boundarys it
  // will get set to the opposite site of the domain
  Double3 UpdatePositionTorus(Double3 position);

  // ---------------------------------------------------------------------------
  // Flocking Algorithm

  // iterates over all neightbor boids and adds the interaction terms;
  // returns a flocking force that produces a-latices in free space
  Double3 GetFlockingForce();

  // iterates over all spherical and cuboid obstacles and adds the interaction
  // terms; returns a force to avoid them and keep the desired distance
  Double3 GetObstacleAvoidanceForce();

  // To do
  Double3 GetNavigationalFeedbackForce();

  // To do
  Double3 GetExtendedCohesionTerm(Double3 centre_of_mass);

  // returns the interaction term for a given boid
  Double3 GetBoidInteractionTerm(const Boid* boid);

  // returns the interaction term for a given sphere
  Double3 GetSphereInteractionTerm(SphereObstacle* sphere);

  // returns the interaction term for a given cuboid
  Double3 GetCuboidInteractionTerm(CuboidObstacle* cuboid);

  // funcions that project the boids position / velocity onto a obstacele sphere
  // or cuboid (orthogonal projection)
  Double3 GetProjectedPosition(SphereObstacle* sphere);

  Double3 GetProjectedPosition(CuboidObstacle* cuboid);

  Double3 GetProjectedVelocity(SphereObstacle* sphere);

  Double3 GetProjectedVelocity(CuboidObstacle* cuboid);

  bool IsHeadingTowards(Double3 point);

  // functions needed to calculate the interaction terms
  double Norm_sig(Double3 z);

  double Norm_sig(double z);

  double Phi(double z);

  double rho_h(double z, double h);

  double rho_h_a(double z, double h);

  double rho_h_inv(double z, double h);

  double sigmoid_1(double z);

  double sigmoid_2(double z);

  double Phi_a(double z);

  double Phi_b(double z);

  // ---------------------------------------------------------------------------
  Double3 new_position_, new_velocity_;
  Double3 acceleration_, velocity_, heading_direction_;
  double actual_diameter_;
  double boid_perception_radius_, boid_interaction_radius_,
      obstacle_perception_radius_;
  double perception_angle_, cos_perception_angle_;
  double boid_distance_, obstacle_distance_;
  double max_force_, min_speed_, crusing_speed_, max_speed_;
  bool obstacles_obstruct_view_ = true;

  TGeoNavigator* navig_;

  // Flocking constants
  double c_a_1_;
  double c_a_2_;
  double c_b_1_;
  double c_b_2_;
  double c_y_;
  double eps_ = 0.1;
  double h_a_ = 0.2, h_b_ = 0.4;

  // this vector stores the average distance to all boids within
  // boid_interaction_radius_ for each timestep
  std::vector<double> avg_dist_r_i_;
};

////////////////////////////////////////////////////////////////////////////////
//----------------------------------------------------------------------------//
// Flocking Behaviour                                                         //
//----------------------------------------------------------------------------//
////////////////////////////////////////////////////////////////////////////////

struct Flocking : public Behavior {
  BDM_BEHAVIOR_HEADER(Flocking, Behavior, 1);

  void Run(Agent* agent) override;
};

struct FreeFlocking : public Behavior {
  BDM_BEHAVIOR_HEADER(FreeFlocking, Behavior, 1);

  void Run(Agent* agent) override;
};

// Functor class needed to calculate neighbor data in Flocking
// ForEachNeighbor call
class CalculateNeighborData : public Functor<void, Agent*, double> {
 public:
  CalculateNeighborData(Boid* boid) : boid_(boid) {}
  virtual ~CalculateNeighborData() {}

  void operator()(Agent* neighbor, double squared_distance) override;

  Double3 GetU_a() { return u_a; };

  Double3 GetCentreOfMass();

  Boid* boid_;
  Double3 u_a = {0, 0, 0}, sum_pos = {0, 0, 0};
  int n = 0;
};

// Functor class to calculate various Data for Analysis / Export
class FlockingNeighborAnalysis : public Functor<void, Agent*, double> {
 public:
  FlockingNeighborAnalysis(Boid* boid) : boid_(boid) {}
  virtual ~FlockingNeighborAnalysis() {}

  void operator()(Agent* neighbor, double squared_distance) override;

  // returns the avg dist or -1 if there are no neighbors
  double GetAvgDist_InteractionR();

  Boid* boid_;
  double sum_dist_interacion_r = 0;
  int n = 0;
};

}  // namespace bdm

#endif  // BOID_H_
