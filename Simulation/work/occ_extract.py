"""
Reads a STEP file given on the command line with OpenCascade XCAF (names + colours), tessellates every solid
and writes one OBJ per (solid, colour) with the colour exactly as set in the CAD.

Colour priority per face: face colour > solid colour > part colour > inherited from the
assembly component that places it. Faces with no colour anywhere keep Fusion's default.
"""
import os, json
import numpy as np
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool, XCAFDoc_ColorType, XCAFDoc_ColorTool
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.Quantity import Quantity_Color, Quantity_TOC_sRGB
from OCP.TopLoc import TopLoc_Location
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE, TopAbs_REVERSED
from OCP.TopoDS import TopoDS
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp

HERE = os.path.dirname(os.path.abspath(__file__))
import sys
STEP = sys.argv[1]
OUT = os.path.join(HERE, sys.argv[2], "parts_occ")
os.makedirs(OUT, exist_ok=True)
DEFAULT = None  # faces with no CAD colour anywhere

doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
r = STEPCAFControl_Reader()
r.SetColorMode(True)
r.SetNameMode(True)
assert r.ReadFile(STEP) == 1, "read failed"
r.Transfer(doc)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
TYPES = [XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen]


def rgb(c):
    r_, g_, b_ = c.Values(Quantity_TOC_sRGB)
    return (round(r_, 4), round(g_, 4), round(b_, 4))


def label_color(lab):
    c = Quantity_Color()
    for t in TYPES:
        if XCAFDoc_ColorTool.GetColor_s(lab, t, c):
            return rgb(c)
    return None


def shape_color(shp):
    c = Quantity_Color()
    for t in TYPES:
        if ct.GetColor(shp, t, c):
            return rgb(c)
    return None


def name_of(lab):
    a = TDataStd_Name()
    if lab.FindAttribute(TDataStd_Name.GetID_s(), a):
        return a.Get().ToExtString()
    return ""


solids = []


def visit(lab, loc, inherited, path):
    if XCAFDoc_ShapeTool.IsReference_s(lab):
        ref = TDF_Label()
        XCAFDoc_ShapeTool.GetReferredShape_s(lab, ref)
        col = label_color(lab) or label_color(ref) or inherited
        loc2 = loc.Multiplied(XCAFDoc_ShapeTool.GetLocation_s(lab))
        visit(ref, loc2, col, path + "/" + name_of(lab))
        return
    if XCAFDoc_ShapeTool.IsAssembly_s(lab):
        comps = TDF_LabelSequence()
        XCAFDoc_ShapeTool.GetComponents_s(lab, comps)
        col = label_color(lab) or inherited
        for i in range(1, comps.Length() + 1):
            visit(comps.Value(i), loc, col, path)
        return
    part = XCAFDoc_ShapeTool.GetShape_s(lab)
    pcol = label_color(lab) or inherited
    ex = TopExp_Explorer(part, TopAbs_SOLID)
    k = 0
    while ex.More():
        solid = ex.Current()
        solids.append((solid, shape_color(solid) or pcol, loc, path + "/" + name_of(lab), k))
        ex.Next()
        k += 1
    if k == 0:                       # part stored as shells / loose faces (no SOLID): keep it anyway
        fx0 = TopExp_Explorer(part, TopAbs_FACE)
        if fx0.More():
            print("  part without solid kept as faces:", path + "/" + name_of(lab))
            solids.append((part, pcol, loc, path + "/" + name_of(lab), 0))


roots = TDF_LabelSequence()
st.GetFreeShapes(roots)
for i in range(1, roots.Length() + 1):
    visit(roots.Value(i), TopLoc_Location(), None, "")
print("solids:", len(solids))

meta = []
for idx, (solid, scol, loc, name, k) in enumerate(solids):
    BRepMesh_IncrementalMesh(solid, 0.25, False, 0.35, True)
    trsf = loc.Transformation()
    pieces = {}
    fx = TopExp_Explorer(solid, TopAbs_FACE)
    while fx.More():
        face = TopoDS.Face_s(fx.Current())
        col = shape_color(face) or scol or DEFAULT
        fl = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, fl)
        if tri is not None:
            ft = fl.Transformation()
            pts = []
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(ft).Transformed(trsf)
                pts.append((p.X(), p.Y(), p.Z()))
            tris = np.array([tri.Triangle(i).Get() for i in range(1, tri.NbTriangles() + 1)]) - 1
            if face.Orientation() == TopAbs_REVERSED:
                tris = tris[:, ::-1]
            v, f = pieces.setdefault(col, ([], []))
            base = sum(len(x) for x in v)
            v.append(np.array(pts))
            f.append(tris + base)
        fx.Next()
    if not pieces:
        continue
    files = []
    allv = []
    for j, (col, (v, f)) in enumerate(pieces.items()):
        V = np.vstack(v) / 1000.0
        F = np.vstack(f)
        allv.append(V)
        fn = f"s{idx:03d}_{j}.obj"
        with open(os.path.join(OUT, fn), "w") as fh:
            fh.write("".join(f"v {a:.6f} {b:.6f} {c:.6f}\n" for a, b, c in V))
            fh.write("".join(f"f {a+1} {b+1} {c+1}\n" for a, b, c in F))
        files.append({"file": fn, "rgb": col})
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(solid.Moved(loc), props)
    if abs(props.Mass()) < 1e-9:
        BRepGProp.SurfaceProperties_s(solid.Moved(loc), props)
    c = props.CentreOfMass()
    A = np.vstack(allv)
    meta.append({"id": idx, "name": name, "com_mm": [c.X(), c.Y(), c.Z()],
                 "bounds": [A.min(0).tolist(), A.max(0).tolist()], "pieces": files})

json.dump(meta, open(os.path.join(HERE, sys.argv[2], "parts_occ.json"), "w"), indent=1)
from collections import Counter
cc = Counter(tuple(p["rgb"]) if p["rgb"] else None for m in meta for p in m["pieces"])
print("written", len(meta), "solids; colours:", cc.most_common(25))
