"""Find the thing the sentence names, in the picture the robot just took.

Up to now the capture was told which prims are pickable (`obj_1`, `obj_plan`)
by the simulator's own semantic labels, and GraspGenX was pointed at one of
them by name. Nothing understood "box". This is the step in front of that:
the robot is told, in words, what to pick up, and the words are grounded in
the RGB frame by NVIDIA C-RADIO -- the same model and the same text matching
cuRobo's own perception tutorial uses
(curobo/examples/getting_started/feature_mapping.py: CRadioInference, its
``extract_patch_features``, ``project_features`` and ``encode_text``). The
tutorial fuses the features into a TSDF and scores map blocks against a
prompt; here there is one frame, so the same score is taken per image patch.

What comes out is written back into the capture in the one form GraspGenX
already reads (graspgenx.utils.scene_loaders.load_realworld_scene): a new
``obj_<label>`` entry in ``label_map`` and its pixels in ``seg.png``. The
grasp side stays untouched; it is simply pointed at the object the sentence
picked.

    python grasp/find_by_text.py CAPTURE_DIR "box" [--on "table"] \
        [--label obj_lang] [--scale 2] [--adaptor clip]

--on names the surface the object has to be resting on ("the box on the
desk"): components matching the object prompt are kept only if they sit just
above a component matching the support prompt. Without it the strongest
match wins.

Prints ``FOUND <label> <n_px> <x> <y> <z>`` (object centre in the cell frame)
or ``NOT FOUND``, and exits 0 / 2 accordingly, so a search loop can turn and
look again.
"""
import json
import os
import sys

import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image

sys.path.insert(0, "/home/sehoon/Projects/GraspGenX")
from graspgenx.utils.scene_loaders import (  # noqa: E402
    depth_to_camera_xyz, transform_xyz)


def _arg(flag, default):
    return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default


# Prompts the query is scored AGAINST. A cosine score on its own is not a
# decision -- SigLIP/CLIP similarities sit in a narrow band for every patch --
# so each patch is asked which of these it is most like, and the query wins
# or loses per patch. The list is the cell's own furniture, nothing else.
BACKGROUND = ["floor", "wall", "table", "shelf", "robot arm", "ceiling"]


def score_map(model, rgb, prompts, scale):
    """Per-pixel probability of each prompt, (P, H, W), from C-RADIO patches."""
    H, W = rgb.shape[:2]
    img = torch.from_numpy(rgb).to(model.device)
    if scale != 1:
        # A small object far away covers one or two 16 px patches. Feeding
        # the frame enlarged gives it several; RADIO takes any resolution.
        t = img.permute(2, 0, 1).float().unsqueeze(0)
        t = F.interpolate(t, scale_factor=scale, mode="bilinear",
                          align_corners=False)
        img = t[0].permute(1, 2, 0).round().clamp(0, 255).to(torch.uint8)
    feats = model.extract_patch_features(img)              # (Hp, Wp, D)
    Hp, Wp, D = feats.shape
    proj = model.project_features(feats.reshape(-1, D))    # (Hp*Wp, Dt)
    text = model.encode_text(prompts)                      # (P, Dt)
    sim = proj.float() @ text.float().T                    # (Hp*Wp, P)
    # 100 is CLIP's own logit scale; the same temperature the teachers were
    # trained with.
    prob = torch.softmax(100.0 * sim, dim=-1).T.reshape(len(prompts), Hp, Wp)
    prob = F.interpolate(prob.unsqueeze(0), size=(H, W), mode="bilinear",
                         align_corners=False)[0]
    return prob.cpu().numpy()


def components(mask, min_px, depth=None, tol=0.15):
    """Connected regions of `mask`, each trimmed to one depth.

    A 16 px patch is coarser than a small object far away, so a region that
    wins the prompt spills onto whatever is behind it -- measured, 474 px
    won 'box' where the box itself is 416, and the spill sat on the wall
    three metres further back. Keep only the pixels within `tol` of the
    region's median depth; the object is at one distance, the spill is not.
    """
    from scipy import ndimage
    lab, n = ndimage.label(mask)
    out = []
    for i in range(1, n + 1):
        m = lab == i
        if depth is not None and m.any():
            med = float(np.median(depth[m]))
            m = m & (np.abs(depth - med) < tol)
        if m.sum() >= min_px:
            out.append(m)
    return out


def resting_gap(m, supports, xyz, above=None):
    """How far the region's underside is above the nearest support surface.

    'On the desk' is a claim about geometry as much as words: the thing's
    bottom sits at the desk's top. So the support prompt is used to find the
    desk as a region, and the gap between that region's top (near the
    object's footprint) and the object's bottom is what decides. Measured on
    the far frame: box on the desk +0.01 m, crate on the rack +0.36 m above
    the nearest 'table' pixels, box on the floor -0.65 m below the desk top.
    Asking what the pixels around the object look like instead read the desk
    legs as 'table' under the box on the floor.

    Returns the gap in metres, or None if no support region is near.
    """
    p = xyz[m]
    bottom = float(np.percentile(p[:, 2], 5))
    cx, cy = float(p[:, 0].mean()), float(p[:, 1].mean())
    best = None
    for s in supports:
        q = xyz[s]
        near = (np.abs(q[:, 0] - cx) < 0.25) & (np.abs(q[:, 1] - cy) < 0.25)
        if above is not None:
            # A table top is not at floor level. Around a box on the floor
            # the floor itself and the feet of the desk legs read as 'table'
            # -- measured on the near frame, 8326 px of floor box passed as
            # "on the table" with a gap of 0.00 m -- so pixels down at the
            # floor cannot be the surface something rests on.
            near &= q[:, 2] > above
        if near.sum() < 10:
            continue
        top = float(np.percentile(q[near, 2], 95))
        gap = bottom - top
        if best is None or abs(gap) < abs(best[0]):
            best = (gap, top)
    return best


def grow_above(seed, xyz, valid, top, radius=0.12):
    """Everything standing on the support around the seed is the object.

    The words find the thing; they do not outline it. C-RADIO's 16 px patches
    give back a piece of it -- measured on the near frame, 118 px of a box
    the camera sees 1037 px of, the rest of it voted 'table' against the desk
    top behind it. Its extent is a geometric question once the support is
    known: the points above the desk top within reach of the seed, connected
    in the image, are the box. The tabletop convention every real-world
    segmentation starts from; GraspGenX's own scene loaders assume it.
    """
    from scipy import ndimage
    c = xyz[seed].mean(axis=0)
    r = np.hypot(xyz[..., 0] - c[0], xyz[..., 1] - c[1])
    cand = valid & (xyz[..., 2] > top + 0.005) & (r < radius)
    lab, n = ndimage.label(cand | seed)
    ids = set(np.unique(lab[seed])) - {0}
    return np.isin(lab, list(ids)) & cand | seed


RESTING = (-0.05, 0.10)     # metres between underside and support top


def main():
    cap, query = sys.argv[1], sys.argv[2]
    on = _arg("--on", None)
    label = _arg("--label", "obj_lang")
    scale = float(_arg("--scale", "2"))
    adaptor = _arg("--adaptor", "clip")
    min_px = int(_arg("--min-px", "60"))

    meta = json.load(open(os.path.join(cap, "meta_data.json")))
    rgb = np.asarray(Image.open(os.path.join(cap, "rgb.png")).convert("RGB"))
    depth = np.load(os.path.join(cap, "depth.npy"))
    # Start from the capture as the camera wrote it, every time: the first
    # run keeps a copy, later runs (a different sentence, a fix) do not pile
    # a second answer on top of the first.
    orig = os.path.join(cap, "seg_orig.png")
    if not os.path.exists(orig):
        os.rename(os.path.join(cap, "seg.png"), orig)
        Image.open(orig).save(os.path.join(cap, "seg.png"))
    seg = np.asarray(Image.open(orig), dtype=np.int32)
    meta["label_map"] = {k: v for k, v in meta["label_map"].items()
                         if k != meta.get("language", {}).get("label")}
    meta.pop("language", None)
    K = np.asarray(meta["intrinsics"], float)
    cam_pose = np.asarray(meta["camera_pose"], float)
    valid = depth > 0

    from curobo.examples.getting_started.feature_mapping import CRadioInference
    model = CRadioInference(text_adaptor_name=adaptor)
    prompts = [query] + ([on] if on else []) + \
        [b for b in BACKGROUND if b not in (query, on)]
    prob = score_map(model, rgb, prompts, scale)
    best = prob.argmax(axis=0)
    obj_mask = (best == 0) & valid
    print(f"[text] '{query}' wins {int(obj_mask.sum())} px against "
          f"{prompts[1:]}")

    xyz = transform_xyz(depth_to_camera_xyz(depth, K), cam_pose)
    cands = components(obj_mask, min_px, depth=depth)
    if not cands:
        print("NOT FOUND")
        sys.exit(2)
    supports = components((best == 1) & valid, 200) if on else []
    # Where the floor is, from the pixels that look like floor -- so that a
    # support surface can be required to be above it.
    floor_z = None
    if on and "floor" in prompts:
        fl = (best == prompts.index("floor")) & valid
        if fl.sum() > 500:
            floor_z = float(np.median(xyz[fl][:, 2]))
            print(f"[text] floor seen at z {floor_z:+.3f} ({int(fl.sum())} px)")
    kept = []
    for i, m in enumerate(cands):
        c = xyz[m].mean(axis=0)
        line = (f"[text]   '{query}' region {int(m.sum()):5d} px at "
                f"{np.round(c, 2)} p={float(prob[0][m].mean()):.2f}")
        if on:
            got = resting_gap(m, supports, xyz,
                              above=None if floor_z is None else floor_z + 0.10)
            if got is not None:
                # Grow first, judge after. The words may light up only the
                # top of the thing -- measured, 64 px of a box seen edge-on,
                # whose underside then read 0.11 m above the desk and failed
                # the test. What stands on the surface is the whole grown
                # region, so that is what the gap is measured on.
                m = grow_above(m, xyz, valid, got[1])
                got = resting_gap(m, supports, xyz,
                                  above=None if floor_z is None else floor_z + 0.10)
            gap = None if got is None else got[0]
            rests = gap is not None and RESTING[0] < gap < RESTING[1]
            line += (f", {'on' if rests else 'NOT on'} '{on}'"
                     + (f" (gap {gap:+.2f} m, {int(m.sum())} px grown)"
                        if gap is not None else " (none near)"))
            if rests:
                kept.append(m)
        print(line)

    if on:
        if not kept:
            print(f"[text] {len(cands)} '{query}' region(s), none on '{on}'")
            print("NOT FOUND")
            sys.exit(2)
        cands = kept

    # Strongest match by mean probability, when more than one is left.
    chosen = max(cands, key=lambda m: float(prob[0][m].mean()))
    centre = xyz[chosen].mean(axis=0)

    new_id = int(max(meta["label_map"].values())) + 1
    seg = seg.copy()
    seg[chosen] = new_id
    Image.fromarray(seg.astype(np.int32), mode="I").save(
        os.path.join(cap, "seg.png"))
    meta["label_map"][label] = new_id
    meta["language"] = {"query": query, "on": on, "label": label,
                        "pixels": int(chosen.sum()),
                        "centre": [float(v) for v in centre]}
    json.dump(meta, open(os.path.join(cap, "meta_data.json"), "w"), indent=2)

    # What it saw, for the record: the frame with the chosen pixels outlined.
    over = rgb.copy()
    over[chosen] = (0.4 * over[chosen] + 0.6 * np.array([0, 255, 255])).astype(np.uint8)
    Image.fromarray(over).save(os.path.join(cap, f"{label}_overlay.png"))
    print(f"FOUND {label} {int(chosen.sum())} "
          f"{centre[0]:.3f} {centre[1]:.3f} {centre[2]:.3f}")


if __name__ == "__main__":
    main()
