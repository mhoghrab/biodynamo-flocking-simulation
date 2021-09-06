#include "world_geometry.h"
#include "sim_param.h"

void WorldGeometry::CreateCentreBox() {
  const auto *param = bdm::Simulation::GetActive()->GetParam();
  double max_bound = param->max_bound;
  double min_bound = param->min_bound;
  double centre = (max_bound - min_bound) / 2;

  new TGeoManager();

  TGeoMaterial *mat = new TGeoMaterial("Vacuum", 0, 0, 0);
  TGeoMedium *med = new TGeoMedium("Vacuum", 1, mat);

  //// ToDo: creating TopVolumne corresponding to the bdm domain
  TGeoVolume *top = gGeoManager->MakeBox("Top", med, 4000, 4000, 4000);
  gGeoManager->SetTopVolume(top);

  // creating a boundary box for the bdm domain
  TGeoTranslation *tr1 = new TGeoTranslation(centre, centre, centre);
  TGeoVolume *boundary_box =
      gGeoManager->MakeBox("boundary_box", med, centre, centre, centre);
  top->AddNode(boundary_box, 1, tr1);

  // creating a simple obstacle box
  // TGeoTranslation *tr_obst_1 = new TGeoTranslation(0, 0, 0);
  TGeoVolume *obst_1 = gGeoManager->MakeBox("obst_1", med, 100, 100, 100);
  boundary_box->AddNode(obst_1, 1);

  gGeoManager->CloseGeometry();
}