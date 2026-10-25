# NOVEXA: CAD to MuJoCo

This folder turns the NOVEXA STEP CAD into a MuJoCo robot model and opens it in the MuJoCo viewer. The CAD is the source of truth. The scripts regenerate the mesh and XML files from it, so you don't edit the XML by hand.

## What's here

| Path | What it does |
|---|---|
| `../Mechanical/Design/Assembly/novexa_assembly_01.step` | The CAD assembly |
| `work/occ_extract.py` | STEP -> one OBJ per solid, plus `parts_occ.json` |
| `scripts/build_model.py` | OBJ parts -> `meshes/`, `work/rig.json`, `work/mesh_index.json`, `mjcf/robot_bodies.xml` |
| `scripts/mjcf_robot.py` | Writes `mjcf/robot_test.xml`, the robot model that loads in MuJoCo |

## Requirements

Python 3 with:

```bash
pip install mujoco numpy trimesh cadquery-ocp
```

`cadquery-ocp` provides the `OCP` module used by the STEP reader.

## Editing the CAD

Edit the assembly in Fusion, then export it as **STEP** (File > Export > STEP). The scripts read STEP only. They can't read `.f3d` or `.f3z` directly.

Re-run the steps below after every export.

## Build the model

Run from the `Simulation/` folder:

```bash
mkdir -p meshes mjcf
python work/occ_extract.py ../Mechanical/Design/Assembly/novexa_assembly_01.step .
python scripts/build_model.py
python scripts/mjcf_robot.py
```

The `.` argument tells `occ_extract.py` to write into `work/`.

## Open the model in MuJoCo

```bash
python -m mujoco.viewer --mjcf=mjcf/robot_test.xml
```

Or from Python:

```python
import mujoco

model = mujoco.MjModel.from_xml_path("mjcf/robot_test.xml")
data = mujoco.MjData(model)
```

Joints are named `<LEG>_{coxa,femur,tibia}_joint`, where LEG is one of `FL ML BL BR MR FR`.

## Limits

`build_model.py` is written for this robot. It recognises parts by size and position, for example the servo pin dimensions and the height bands that separate coxa and femur servos. If a CAD edit changes the part sizes or layout, the classification may need updating in that script.

Frame convention, in metres: x forward (the 4-hole ultrasonic face), y left, z up.
