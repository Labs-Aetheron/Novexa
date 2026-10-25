"""Generates the NOVEXA <asset> meshes and <body> tree (robot frame, standing pose = zero joint pose)."""
import os, json, math
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
WORK = os.path.join(SIM, "work")
LEGS = ["FL", "ML", "BL", "BR", "MR", "FR"]
RIG = json.load(open(os.path.join(WORK, "rig.json")))
IDX = json.load(open(os.path.join(WORK, "mesh_index.json")))

# joint limits (rad) - generous; gait stays well inside
RANGE = {"coxa": (-0.9, 0.9), "femur": (-1.9, 1.9), "tibia": (-2.2, 2.2)}
BODY_MASS = 2.6
LINK_MASS = {"coxa": 0.16, "femur": 0.12, "tibia": 0.07}


def v3(a):
    return " ".join(f"{x:.6f}" for x in a)


def mesh_assets(meshdir_prefix="meshes"):
    out = []
    for key, fn in IDX.items():
        name = fn[:-4]
        out.append(f'<mesh name="{name}" file="{fn}"/>')
    return "\n    ".join(out)


def geoms(link):
    s = []
    for mat in ("mid", "tip", "logo"):
        k = f"{link}|{mat}"
        if k in IDX:
            n = IDX[k][:-4]
            s.append(f'<geom class="vis" mesh="{n}" material="mat_{mat}"/>')
    return "\n".join(s)


def body_xml(root_pos="0 0 0.0836", with_free=True, indent="    "):
    b = []
    b.append(f'<body name="novexa_base" pos="{root_pos}">')
    if with_free:
        b.append('  <freejoint name="root"/>')
    b.append(f'  <inertial pos="0 0 0.03" mass="{BODY_MASS}" diaginertia="0.012 0.016 0.02"/>')
    b.append(geoms("body"))
    b.append('  <site name="imu" pos="0 0 0.04" size="0.004"/>')
    for n in LEGS:
        r = RIG[n]
        hip = np.array(r["hip"])
        fem = np.array(r["femur"])
        tib = np.array(r["tibia"])
        foot = np.array(r["foot"])
        psi = r["psi"]
        ax = (-math.sin(psi), math.cos(psi), 0.0)
        b.append(f'  <body name="{n}_coxa" pos="{v3(hip)}">')
        b.append(f'    <inertial pos="0 0 0" mass="{LINK_MASS["coxa"]}" diaginertia="2e-4 2e-4 2e-4"/>')
        b.append(f'    <joint name="{n}_coxa_joint" type="hinge" axis="0 0 1" range="{RANGE["coxa"][0]} {RANGE["coxa"][1]}" damping="0.05"/>')
        b.append(geoms(f"{n}_coxa"))
        b.append(f'    <body name="{n}_femur" pos="{v3(fem - hip)}">')
        b.append(f'      <inertial pos="0 0 0" mass="{LINK_MASS["femur"]}" diaginertia="2e-4 2e-4 2e-4"/>')
        b.append(f'      <joint name="{n}_femur_joint" type="hinge" axis="{v3(ax)}" range="{RANGE["femur"][0]} {RANGE["femur"][1]}" damping="0.05"/>')
        b.append(geoms(f"{n}_femur"))
        b.append(f'      <body name="{n}_tibia" pos="{v3(tib - fem)}">')
        b.append(f'        <inertial pos="0 0 0" mass="{LINK_MASS["tibia"]}" diaginertia="1e-4 1e-4 1e-4"/>')
        b.append(f'        <joint name="{n}_tibia_joint" type="hinge" axis="{v3(ax)}" range="{RANGE["tibia"][0]} {RANGE["tibia"][1]}" damping="0.05"/>')
        b.append(geoms(f"{n}_tibia"))
        b.append(f'        <site name="foot_{n}" pos="{v3(foot - tib)}" size="0.0055" rgba="1 0 0 0"/>')
        b.append('      </body>')
        b.append('    </body>')
        b.append('  </body>')
    b.append('</body>')
    return "\n".join(indent + l for l in b)


def default_xml():
    return ('<default><default class="vis"><geom type="mesh" contype="0" conaffinity="0" group="1"/></default></default>')


# colours come straight from the V2 CAD: near-black body (0.098), pale blue-white foot tips, grey logo letters
MATERIALS = ('<material name="mat_mid" rgba="0.098 0.098 0.098 1" specular="0.5" shininess="0.6" reflectance="0.0"/>\n'
             '    <material name="mat_tip" rgba="0.733 0.8 0.933 1" specular="0.35" shininess="0.4"/>\n'
             '    <material name="mat_logo" rgba="0.6275 0.6275 0.6275 1" specular="0.4" shininess="0.5"/>')


def standalone_xml():
    return f"""<mujoco model="novexa_test">
  <compiler meshdir="../meshes" angle="radian"/>
  <visual><global offwidth="1920" offheight="1080"/><quality shadowsize="2048"/></visual>
  {default_xml()}
  <asset>
    {mesh_assets()}
    {MATERIALS}
    <texture name="grid" type="2d" builtin="checker" rgb1=".12 .12 .14" rgb2=".18 .18 .2" width="256" height="256"/>
    <material name="floor" texture="grid" texrepeat="12 12" texuniform="true"/>
  </asset>
  <worldbody>
    <light pos="1 -1 2" dir="-1 1 -2" diffuse="1 1 1"/>
    <geom type="plane" size="3 3 .1" material="floor"/>
{body_xml()}
  </worldbody>
</mujoco>"""


if __name__ == "__main__":
    out = os.path.join(SIM, "mjcf", "robot_test.xml")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    open(out, "w").write(standalone_xml())
    print("wrote", out)
