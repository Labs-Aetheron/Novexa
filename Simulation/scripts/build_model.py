"""
Builds the NOVEXA robot meshes + MJCF from the OCC-extracted STEP parts.

Input : simulation/work/parts_occ.json + parts_occ/*.obj   (STEP frame, metres)
Output: simulation/meshes/<link>__<mat>.obj  (robot frame: x forward, y left, z up, metres)
        simulation/mjcf/robot_bodies.xml     (asset + body tree fragments, used by build_scene.py)
        simulation/work/rig.json             (hips, joint points, leg angles, foot offsets)

STEP -> robot frame: the 4-hole (ultrasonic) face is STEP -y, which becomes robot +x (forward):
    x' = -y,  y' = x,  z' = z   (proper rotation, mm -> m already done by occ_extract)
Zero joint pose == the standing pose in the CAD.
"""
import os, json, math, collections
import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
WORK = os.path.join(SIM, "work")
MESH = os.path.join(SIM, "meshes")
MESH_LITE = os.path.join(SIM, "meshes_lite")
os.makedirs(MESH, exist_ok=True)
os.makedirs(MESH_LITE, exist_ok=True)

parts = json.load(open(os.path.join(WORK, "parts_occ.json")))


def rot(p):  # STEP(m) -> robot(m)
    p = np.asarray(p, float)
    return np.array([-p[..., 1], p[..., 0], p[..., 2]]).T if p.ndim > 1 else np.array([-p[1], p[0], p[2]])


def base_name(path):
    return path.split("/")[-1].split(" v")[0].strip()


def inst_key(path):
    segs = [s for s in path.split("/") if s]
    return "/".join(segs[:-1]) if len(segs) > 1 else segs[0]


# ---- group COM per instance (servos classified per whole servo) -------------------------
grp = collections.defaultdict(list)
for p in parts:
    grp[inst_key(p["name"])].append(np.array(p["com_mm"]) / 1000.0)
grp_com = {k: rot(np.mean(v, 0)) for k, v in grp.items()}   # robot frame, metres
grp_com_step_z = {k: np.mean(v, 0)[2] for k, v in grp.items()}

# ---- hips: BearingMountV2 sitting at z~47 mm (bolted to body), robot frame ---------------
hips = []
for p in parts:
    if base_name(p["name"]) == "BearingMountV2" and 0.040 < p["com_mm"][2] / 1000 < 0.055:
        hips.append(rot(np.array(p["com_mm"]) / 1000.0))
# BearingMounts at z47: 6 body-mounted + duplicates in the instance list; dedupe by xy
uniq = []
for h in hips:
    if not any(np.linalg.norm(h[:2] - u[:2]) < 0.01 for u in uniq):
        uniq.append(h)
hips = uniq
assert len(hips) == 6, len(hips)


def leg_name(xy):
    fb = "F" if xy[0] > 0.05 else ("B" if xy[0] < -0.05 else "M")
    lr = "L" if xy[1] > 0 else "R"
    return fb + lr


hip_by_leg = {leg_name(h[:2]): h for h in hips}
assert len(hip_by_leg) == 6, hip_by_leg.keys()
LEGS = ["FL", "ML", "BL", "BR", "MR", "FR"]


def nearest_leg(c):
    """leg whose hip direction (angle about origin) is nearest to the part's angle."""
    a = math.atan2(c[1], c[0])
    best, bd = None, 9
    for n, h in hip_by_leg.items():
        ha = math.atan2(h[1], h[0])
        d = abs((a - ha + math.pi) % (2 * math.pi) - math.pi)
        # mid hips are at angle 0/180; corner hips at ~±45/135
        if d < bd:
            best, bd = n, d
    return best


def oriented_dims(p):
    """rotation-invariant (PCA) extents in mm of a solid, sorted ascending"""
    V = []
    for pc in p["pieces"]:
        for l in open(os.path.join(WORK, "parts_occ", pc["file"])):
            if l.startswith("v "):
                V.append([float(x) for x in l.split()[1:4]])
    V = np.array(V)
    V = V - V.mean(0)
    P = V @ np.linalg.svd(V, full_matrices=False)[2].T
    return np.sort(P.max(0) - P.min(0)) * 1000


def mat_of(rgb):
    """material from the CAD colour of a piece (V2 CAD: near-black body, pale blue-white foot tips, grey logo letters)"""
    if rgb is None:
        return "mid"
    r, g, b = rgb
    if b - r > 0.12 and r > 0.5:
        return "tip"
    if r > 0.45 and abs(r - b) < 0.05:
        return "logo"
    return "mid"


def classify(p):
    """returns (link, material); link = 'body' or ('FL','coxa'|'femur'|'tibia')"""
    nm = base_name(p["name"])
    com = rot(np.array(p["com_mm"]) / 1000.0)
    zmm = p["com_mm"][2]
    mat = "mid"
    if nm.startswith("Tibia"):
        kind = "tibia"
    elif nm.startswith("Femur"):
        kind = "femur"
    elif nm.startswith("Coxa"):
        kind = "coxa"
    elif nm in ("BearingMountV2", "novexa_bearing"):
        # z~47-55 mm: at the hip, fixed in the chassis; z~3 mm: femur-axis mount/bearing (coxa link);
        # z~74-76 mm: tibia-axis mount/bearing (femur link)
        kind = "femur" if zmm > 65 else ("body" if zmm > 40 else "coxa")
    elif nm == "ServoMountWithServo":
        z = grp_com_step_z[inst_key(p["name"])] * 1000
        dims = oriented_dims(p)
        is_pin = abs(dims[0] - 5.9) < 0.5 and abs(dims[1] - 6.6) < 0.5 and abs(dims[2] - 7.1) < 0.5   # servo output-shaft pin
        if 22 < z < 40:        # coxa servo: body is fixed in the chassis, its output pin turns the coxa link
            kind = "coxa" if is_pin else "body"
        elif z < 22:           # femur servo: body is bolted between the coxa brackets, output pin turns the femur link
            kind = "femur" if is_pin else "coxa"
        else:                  # tibia servo: body is bolted between the tibia blades (moves WITH the tibia),
            kind = "femur" if is_pin else "tibia"      # only its output pin belongs to the femur
        com = grp_com[inst_key(p["name"])]
    else:
        return "body", mat
    if kind == "body":
        return "body", mat
    return (nearest_leg(com), kind), mat


# ---- bucket the pieces -------------------------------------------------------------------
buckets = collections.defaultdict(list)       # (link, mat) -> [(V,F)]
tipcom = {}
seen = set()
dropped = 0
for p in parts:
    # the CAD contains one leg twice, exactly coincident -> drop exact duplicates (same name, centre and bounds)
    key = (base_name(p["name"]), tuple(np.round(p["com_mm"], 1)), tuple(np.round(np.array(p["bounds"]).ravel() * 1000, 1)))
    if key in seen:
        dropped += 1
        continue
    seen.add(key)
    link, mat = classify(p)
    for pc in p["pieces"]:
        lines = open(os.path.join(WORK, "parts_occ", pc["file"])).read().split("\n")
        V = np.array([[float(x) for x in l.split()[1:4]] for l in lines if l.startswith("v ")])
        F = np.array([[int(x) - 1 for x in l.split()[1:4]] for l in lines if l.startswith("f ")])
        V = rot(V)
        mat = mat_of(pc["rgb"])
        buckets[(link, mat)].append((V, F))
        if mat == "tip":
            tipcom.setdefault(link[0], []).append(V)

print("duplicate solids dropped:", dropped)
# ---- rig geometry ---------------------------------------------------------------------------
rig = {}
BM_f, BM_t = {}, {}
for p in parts:
    if base_name(p["name"]) == "BearingMountV2":
        z = p["com_mm"][2]
        c = rot(np.array(p["com_mm"]) / 1000.0)
        if z < 30:
            BM_f[nearest_leg(c)] = c
        elif z > 60:
            BM_t[nearest_leg(c)] = c
for n in LEGS:
    tip = np.vstack(tipcom[n])
    lo = tip[np.argmin(tip[:, 2])]
    ctr = tip.mean(0)
    h = hip_by_leg[n]
    d = ctr[:2] - h[:2]
    psi = math.atan2(d[1], d[0])
    foot_r = 0.0055                      # tip sphere radius (11 mm tip)
    foot_c = np.array([ctr[0], ctr[1], tip[:, 2].min() + foot_r])
    rig[n] = dict(
        hip=[h[0], h[1], 0.0],           # coxa axis (vertical) passes through hip xy
        femur=BM_f[n].tolist(),          # point on femur axis
        tibia=BM_t[n].tolist(),          # point on tibia axis
        psi=psi,
        foot=foot_c.tolist(),
        foot_r=foot_r,
    )
json.dump(rig, open(os.path.join(WORK, "rig.json"), "w"), indent=1)
for n in LEGS:
    r = rig[n]
    print(n, "psi=%.1f" % math.degrees(r["psi"]), "hip", np.round(r["hip"], 3), "femur", np.round(r["femur"], 3),
          "tibia", np.round(r["tibia"], 3), "foot", np.round(r["foot"], 3))

# ---- write meshes (link-local coordinates, crease-angle smooth normals) ------------------------
origin = {"body": np.zeros(3)}
for n in LEGS:
    r = rig[n]
    origin[(n, "coxa")] = np.array([r["hip"][0], r["hip"][1], 0.0])
    origin[(n, "femur")] = np.array(r["femur"])
    origin[(n, "tibia")] = np.array(r["tibia"])


def lname(link):
    return "body" if link == "body" else f"{link[0]}_{link[1]}"


written = {}
for (link, mat), lst in buckets.items():
    off = origin[link]
    meshes = []
    for V, F in lst:
        meshes.append(trimesh.Trimesh(V - off, F, process=False))
    m = trimesh.util.concatenate(meshes)
    m.merge_vertices(merge_tex=True, merge_norm=True)
    m.update_faces(m.nondegenerate_faces())
    sm = trimesh.graph.smooth_shade(m, angle=math.radians(38))
    fn = f"{lname(link)}__{mat}.obj"
    nrm = sm.vertex_normals
    with open(os.path.join(MESH, fn), "w") as fh:
        fh.write("".join(f"v {a:.6f} {b:.6f} {c:.6f}\n" for a, b, c in sm.vertices))
        fh.write("".join(f"vn {a:.5f} {b:.5f} {c:.5f}\n" for a, b, c in nrm))
        fh.write("".join(f"f {a+1}//{a+1} {b+1}//{b+1} {c+1}//{c+1}\n" for a, b, c in sm.faces))
    written[(link, mat)] = (fn, len(sm.faces))
    # low-poly copy for the interactive viewer on weak GPUs (quadric decimation, same crease-angle shading)
    try:
        import fast_simplification
        if len(m.faces) > 6000:
            pts, fcs = fast_simplification.simplify(np.asarray(m.vertices, np.float32), np.asarray(m.faces, np.int32),
                                                    target_reduction=0.62, agg=3)
            ml = trimesh.Trimesh(pts, fcs, process=False)
        else:
            ml = m
        sl = trimesh.graph.smooth_shade(ml, angle=math.radians(38))
        with open(os.path.join(MESH_LITE, fn), "w") as fh:
            fh.write("".join(f"v {a:.6f} {b:.6f} {c:.6f}\n" for a, b, c in sl.vertices))
            fh.write("".join(f"vn {a:.5f} {b:.5f} {c:.5f}\n" for a, b, c in sl.vertex_normals))
            fh.write("".join(f"f {a+1}//{a+1} {b+1}//{b+1} {c+1}//{c+1}\n" for a, b, c in sl.faces))
        lite_tris = globals().setdefault("lite_tris", [0]); lite_tris[0] += len(sl.faces)
    except Exception as ex:
        print("lite mesh failed for", fn, ex)
print("meshes:", len(written), "tris:", sum(v[1] for v in written.values()), "lite tris:", globals().get("lite_tris", [0])[0])
json.dump({f"{lname(k[0])}|{k[1]}": v[0] for k, v in written.items()}, open(os.path.join(WORK, "mesh_index.json"), "w"), indent=1)
