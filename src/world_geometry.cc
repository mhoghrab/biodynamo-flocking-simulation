#include "world_geometry.h"
#include "sim_param.h"

namespace bdm {

void CreateRootObstacles() {
  const auto *param = bdm::Simulation::GetActive()->GetParam();
  double max_bound = param->max_bound;
  double min_bound = param->min_bound;
  double centre = (max_bound - min_bound) / 2;
  double max_length = std::max(std::abs(max_bound), std::abs(min_bound));

  new TGeoManager();

  TGeoMaterial *mat = new TGeoMaterial("Vacuum", 0, 0, 0);
  TGeoMedium *med = new TGeoMedium("Vacuum", 1, mat);

  //// ToDo: creating TopVolumne corresponding to the bdm domain
  TGeoVolume *top =
      gGeoManager->MakeBox("Top", med, max_length, max_length, max_length);
  gGeoManager->SetTopVolume(top);

  // creating a boundary box for the bdm domain
  TGeoTranslation *tr1 = new TGeoTranslation(centre, centre, centre);
  TGeoVolume *boundary_box =
      gGeoManager->MakeBox("boundary_box", med, centre, centre, centre);
  top->AddNode(boundary_box, 1, tr1);

  // -----------------------------------------------------------------------------
  // creating a simple geometry
  double width = 100;
  double height = 1400;
  double gap = 400;

  // creating box_1
  TGeoTranslation *tr_1 =
      new TGeoTranslation(-(gap + width) / 2, 0, centre - height / 2);
  TGeoVolume *box_1 =
      gGeoManager->MakeBox("box_1", med, width / 2, centre, height / 2);
  boundary_box->AddNode(box_1, 1, tr_1);

  // creating box_2
  TGeoTranslation *tr_2 =
      new TGeoTranslation((gap + width) / 2, 0, -centre + height / 2);
  TGeoVolume *box_2 =
      gGeoManager->MakeBox("box_2", med, width / 2, centre, height / 2);
  boundary_box->AddNode(box_2, 1, tr_2);

  gGeoManager->CloseGeometry();

  // gGeoManager->Export("world_geometry.gdml");
}

////////////////////////////////////////////////////////////////////////////////
// Sphere and Cuboid Obstacles
////////////////////////////////////////////////////////////////////////////////

void InitializeRootGeometry() {
  const auto *param = bdm::Simulation::GetActive()->GetParam();
  double max_bound = param->max_bound;
  double min_bound = param->min_bound;
  double max_length = std::max(std::abs(max_bound), std::abs(min_bound));

  new TGeoManager();

  TGeoMaterial *mat = new TGeoMaterial("Vacuum", 0, 0, 0);
  TGeoMedium *med = new TGeoMedium("Vacuum", 1, mat);

  TGeoVolume *top =
      gGeoManager->MakeBox("Top", med, max_length, max_length, max_length);
  gGeoManager->SetTopVolume(top);

  // add spheres to root geometry
  for (auto sphere : SphereObstacle::sphere_obstacles) {
    TGeoTranslation *translation = new TGeoTranslation(
        (sphere.centre_)[0], (sphere.centre_)[1], (sphere.centre_)[2]);

    TGeoVolume *sphere_root = gGeoManager->MakeSphere(
        "sphere", med, 0, sphere.radius_, 0, 180, 0, 180);

    top->AddNode(sphere_root, 1, translation);
  }

  // add cuboids to root geometry
  for (auto cuboid : CuboidObstacle::cuboid_obstacles) {
    TGeoTranslation *translation = new TGeoTranslation(
        (cuboid.upper_bound_[0] + cuboid.lower_bound_[0]) / 2,
        (cuboid.upper_bound_[1] + cuboid.lower_bound_[1]) / 2,
        (cuboid.upper_bound_[2] + cuboid.lower_bound_[2]) / 2);

    TGeoVolume *cuboid_root = gGeoManager->MakeBox(
        "cuboid", med, (cuboid.upper_bound_[0] - cuboid.lower_bound_[0]) / 2,
        (cuboid.upper_bound_[1] - cuboid.lower_bound_[1]) / 2,
        (cuboid.upper_bound_[2] - cuboid.lower_bound_[2]) / 2);

    top->AddNode(cuboid_root, 1, translation);
  }

  gGeoManager->CloseGeometry();
}

// ---------------------------------------------------------------------------

SphereObstacle::SphereObstacle(Double3 centre, double radius) {
  centre_ = centre;
  radius_ = radius;
}

std::vector<SphereObstacle> SphereObstacle::sphere_obstacles;

void CreateSphereObstacles() {
  auto sphere_0 = new SphereObstacle({1000, 1000, 1000}, 200);
  SphereObstacle::sphere_obstacles.push_back(*sphere_0);
}

// ---------------------------------------------------------------------------

CuboidObstacle::CuboidObstacle(Double3 lower_bound, Double3 upper_bound) {
  lower_bound_ = lower_bound;
  upper_bound_ = upper_bound;
}

std::vector<CuboidObstacle> CuboidObstacle::cuboid_obstacles;

void CreateCuboidObstacles() {
  auto cuboid_0 = new CuboidObstacle({650, 950, 950}, {750, 1050, 1050});
  CuboidObstacle::cuboid_obstacles.push_back(*cuboid_0);
}

}  // namespace bdm