# How far the right palm reaches, with and without the waist.
#
# The plan's cuRobo config (end2end/curobo_assets/g1_right_arm.yml) roots the
# chain at torso_link and lists only the right arm's 7 joints, so waist_yaw /
# waist_roll / waist_pitch are frozen at 0 and the reachable set is whatever
# those 7 joints alone can cover. This samples both cases off the URDF so the
# cost of that choice is a number rather than an opinion.
#
#   python map/measure_reach.py [n_samples]
import sys
import xml.etree.ElementTree as ET

import numpy as np

URDF = ("/home/sehoon/Projects/GR00T-WholeBodyControl/gear_sonic/data/"
        "robots/g1/g1_29dof_with_hand_rev_1_0.urdf")
ROOT = "pelvis"
TIP = "right_hand_palm_link"
REF = "pelvis"          # the fixed base in this setup: what the waist can actually move against
WAIST = ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint")
N = int(sys.argv[1]) if len(sys.argv) > 1 else 40000

tree = ET.parse(URDF)
joints = {}
for j in tree.getroot().findall("joint"):
    o = j.find("origin")
    xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0").split())]) if o is not None else np.zeros(3)
    rpy = np.array([float(v) for v in (o.get("rpy", "0 0 0").split())]) if o is not None else np.zeros(3)
    ax = j.find("axis")
    axis = np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else np.array([0., 0., 1.])
    lim = j.find("limit")
    lo, hi = (float(lim.get("lower")), float(lim.get("upper"))) if lim is not None else (0.0, 0.0)
    joints[j.get("name")] = dict(
        parent=j.find("parent").get("link"), child=j.find("child").get("link"),
        xyz=xyz, rpy=rpy, axis=axis, type=j.get("type"), lo=lo, hi=hi)

parent_of = {v["child"]: k for k, v in joints.items()}


def chain_to(link):
    out = []
    while link != ROOT:
        jn = parent_of[link]
        out.append(jn)
        link = joints[jn]["parent"]
    return out[::-1]


def rpy_R(r, p, y):
    cr, sr, cp, sp, cy, sy = (np.cos(r), np.sin(r), np.cos(p),
                              np.sin(p), np.cos(y), np.sin(y))
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def axis_R(axis, q):
    a = axis / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)


def fk(chain, qmap):
    T = np.eye(4)
    for jn in chain:
        j = joints[jn]
        L = np.eye(4)
        L[:3, :3] = rpy_R(*j["rpy"])
        L[:3, 3] = j["xyz"]
        T = T @ L
        if j["type"] in ("revolute", "continuous"):
            R = np.eye(4)
            R[:3, :3] = axis_R(j["axis"], qmap.get(jn, 0.0))
            T = T @ R
    return T


tip_chain = chain_to(TIP)
ref_chain = chain_to(REF)
arm = [j for j in tip_chain if joints[j]["type"] == "revolute"
       and "hand" not in j and j not in WAIST]
print(f"[reach] pelvis -> {TIP}: {len(arm)} arm joints, waist {list(WAIST)}")
print(f"[reach] {', '.join(arm)}")

rng = np.random.default_rng(0)


def sweep(free_waist):
    names = arm + (list(WAIST) if free_waist else [])
    lo = np.array([joints[n]["lo"] for n in names])
    hi = np.array([joints[n]["hi"] for n in names])
    Q = rng.uniform(lo, hi, size=(N, len(names)))
    pts = np.empty((N, 3))
    for i in range(N):
        qmap = dict(zip(names, Q[i]))
        T_ref = fk(ref_chain, qmap)
        T_tip = fk(tip_chain, qmap)
        pts[i] = (np.linalg.inv(T_ref) @ T_tip)[:3, 3]
    return pts


for tag, fw in [("팔 7관절만 (지금)", False), ("+ 허리 3관절", True)]:
    p = sweep(fw)
    print(f"\n=== {tag} ===  샘플 {N}")
    print(f"  손바닥 최저 높이 {p[:, 2].min():+.3f}  (pelvis 기준)")
    print("  높이      앞으로 최대   좌우 폭")
    for z in (0.15, 0.20, 0.25, 0.30, 0.35, 0.40):
        m = np.abs(p[:, 2] - z) < 0.02
        if m.sum() < 20:
            print(f"  {z:+.2f}      (표본 부족)")
            continue
        print(f"  {z:+.2f}      {p[m, 0].max():.3f}        "
              f"{p[m, 1].min():+.3f} ~ {p[m, 1].max():+.3f}")
