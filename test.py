from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib

print("TopoDS has Face_s:", hasattr(TopoDS, "Face_s"))
print("TopoDS has Face  :", hasattr(TopoDS, "Face"))