import pyg4ometry
reader = pyg4ometry.gdml.Reader("world_geometry.gdml")
logical = reader.getRegistry().getWorldVolume()
freg = pyg4ometry.convert.geant4Logical2Fluka(logical)
w = pyg4ometry.fluka.Writer()
w.addDetector(freg)
w.write("world_geometry.inp")