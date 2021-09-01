#include "world_geometry.h"
#include "sim_param.h"

void WorldGeometry::CreateCentreBox() {
  // gSystem->Load("libGeom");
  const auto *param = bdm::Simulation::GetActive()->GetParam();
  // const auto *sparam = param->Get<bdm::SimParam>();
  double centre = (param->max_bound - param->min_bound) / 2;

  new TGeoManager();

  TGeoMaterial *mat = new TGeoMaterial("Vacuum", 0, 0, 0);
  TGeoMedium *med = new TGeoMedium("Vacuum", 1, mat);

  // ToDo: creating TopVolumne corresponding to the bdm domain
  TGeoVolume *top = gGeoManager->MakeBox("Top", med, 2000, 2000, 2000);
  gGeoManager->SetTopVolume(top);

  // creating a box with half length 100 (200x200x200) in the centre of the
  // bdm domain
  TGeoTranslation *tr1 = new TGeoTranslation(centre, centre, centre);
  TGeoVolume *box_1 = gGeoManager->MakeBox("box_1", med, 100, 100, 100);
  top->AddNode(box_1, 1, tr1);
  gGeoManager->CloseGeometry();
}