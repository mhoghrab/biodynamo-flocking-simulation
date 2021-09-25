#include "world_geometry.h"
#include "sim_param.h"

namespace bdm {

void WorldGeometry::CreateCentreBox() {
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

}  // namespace bdm