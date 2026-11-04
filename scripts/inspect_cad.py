#!/usr/bin/env python3
"""Read-only STEP/STL audit and tessellation, using OpenCascade XCAF assembly locations.

Run with .venv/bin/python. Original files are never modified. All STEP lengths
are converted by OCCT to millimetres, then written in metres in mesh archives.
"""
import hashlib
import json
from pathlib import Path
import re

import numpy as np
from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Sphere
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFDoc import XCAFDoc_DocumentTool

ROOT = Path(__file__).resolve().parents[1]


def appearances(step_text):
    result = []
    for entity, value in re.findall(r"#(\d+)=COLOUR_RGB\(([^;]+)\);", step_text, re.S):
        match = re.match(r"'([^']*)'\s*,(.*)", value, re.S)
        if match:
            result.append({"step_entity": int(entity), "name": match[1],
                           "rgb": [float(v) for v in match[2].split(",")]})
    return result


def label_name(label):
    name = TDataStd_Name()
    return name.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), name) else "unnamed"


def bounds(shape):
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    return [list(box.CornerMin().Coord()), list(box.CornerMax().Coord())]


def triangulate(shape):
    BRepMesh_IncrementalMesh(shape, 0.25, False, 0.25, True).Perform()
    vertices, triangles, surfaces = [], [], []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face(exp.Current())
        surface = BRepAdaptor_Surface(face)
        st = surface.GetType()
        if st in (GeomAbs_Cylinder, GeomAbs_Sphere):
            geom = surface.Cylinder() if st == GeomAbs_Cylinder else surface.Sphere()
            entry = {"type": "cylinder" if st == GeomAbs_Cylinder else "sphere",
                     "origin_mm": list(geom.Location().Coord()), "radius_mm": geom.Radius()}
            if st == GeomAbs_Cylinder:
                entry["axis"] = list(geom.Axis().Direction().Coord())
            surfaces.append(entry)
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri:
            offset = len(vertices)
            vertices.extend([list(tri.Node(i).Transformed(loc.Transformation()).Coord())
                             for i in range(1, tri.NbNodes() + 1)])
            for i in range(1, tri.NbTriangles() + 1):
                ids = list(tri.Triangle(i).Get())
                if face.Orientation() == TopAbs_REVERSED:
                    ids.reverse()
                triangles.append([offset + j - 1 for j in ids])
        exp.Next()
    return np.array(vertices, dtype=np.float32) * 0.001, np.array(triangles, dtype=np.int32), surfaces


def main():
    source = ROOT / "Delta+Robot"
    out = ROOT / "assets/generated"
    audit = ROOT / "artifacts/inspection"
    out.mkdir(parents=True, exist_ok=True)
    audit.mkdir(parents=True, exist_ok=True)
    doc = TDocStd_Document(TCollection_ExtendedString("delta"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    assert reader.ReadFile(str(source / "Delta_assembly.step")) == IFSelect_RetDone
    assert reader.Transfer(doc)
    shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    shapes.GetFreeShapes(roots)
    records = []
    tree = []

    def walk(label, parent_location, path):
        loc = parent_location.Multiplied(shapes.GetLocation_s(label))
        original_name = label_name(label)
        referred = TDF_Label()
        if shapes.IsReference_s(label):
            shapes.GetReferredShape_s(label, referred)
            label = referred
        name = label_name(label)
        path = path + [original_name if original_name != "unnamed" else name]
        children = TDF_LabelSequence()
        if shapes.GetComponents_s(label, children):
            tree.append({"path": "/".join(path), "name": name, "children": children.Length()})
            for i in range(1, children.Length() + 1):
                walk(children.Value(i), loc, path)
        else:
            shape = shapes.GetShape_s(label).Located(loc)
            v, t, surfaces = triangulate(shape)
            if not len(v):
                return
            idx = len(records)
            mesh = f"part_{idx:03d}.npz"
            np.savez_compressed(out / mesh, vertices=v, triangles=t)
            records.append({"id": idx, "name": name, "path": "/".join(path), "mesh": mesh,
                            "bounds_mm": bounds(shape), "triangles": len(t), "surfaces": surfaces})

    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [])
    original = []
    import trimesh
    for f in sorted(source.iterdir()):
        item = {"name": f.name, "bytes": f.stat().st_size,
                "sha256": hashlib.sha256(f.read_bytes()).hexdigest()}
        if f.suffix == ".stl":
            mesh = trimesh.load_mesh(f)
            item.update(bounds_mm=mesh.bounds.tolist(), triangles=len(mesh.faces), watertight=mesh.is_watertight)
        original.append(item)
    step_text = (source / "Delta_assembly.step").read_text()
    (audit / "source_materials.json").write_text(json.dumps(appearances(step_text), indent=2))
    result = {"source_units": "mm (STEP SI_UNIT(.MILLI.,.METRE.))", "mesh_units": "m",
              "original_files": original, "assembly_tree": tree, "parts": records,
              "step_entity_counts": {key: len(re.findall(r"\b" + key + r"\(", step_text)) for key in
                                     ["PRODUCT", "NEXT_ASSEMBLY_USAGE_OCCURRENCE", "COLOUR_RGB", "REVOLUTE_PAIR"]}}
    (audit / "cad_inventory.json").write_text(json.dumps(result, indent=2))
    for r in records:
        print(r["id"], r["path"], np.round(r["bounds_mm"], 2).tolist())
    print(f"Exported {len(records)} positioned STEP leaf instances")


if __name__ == "__main__":
    main()
