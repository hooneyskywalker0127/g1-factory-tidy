"""Which part of the thing the sentence means: "the handle of the hammer".

find_by_text.py grounds the object; this grounds a PART inside it, the same
way (cuRobo's C-RADIO feature_mapping example, clip text adaptor), scored only
over the object's own pixels against the object's other parts. The part's
pixels are written as a second label so GraspGenX can be pointed at them
(--capture_object obj_part): the grasps then come out on the handle, which is
where a person takes a hammer from the floor, not on the head.

    python grasp/find_part.py CAPTURE_DIR "handle" --of obj_lang \
        [--others "hammer head"] [--label obj_part] [--scale 3] [--min-px 40]

Prints FOUND <label> <n_px> <x> <y> <z> or NOT FOUND (exit 2).
"""
import json
import os
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, "/home/sehoon/Projects/GraspGenX")
from find_by_text import score_map, components, _arg  # noqa: E402
from graspgenx.utils.scene_loaders import depth_to_camera_xyz, transform_xyz  # noqa: E402


def main():
    cap, part = sys.argv[1], sys.argv[2]
    of = _arg("--of", "obj_lang")
    label = _arg("--label", "obj_part")
    others = [o.strip() for o in _arg("--others", "head").split(",") if o.strip()]
    scale = float(_arg("--scale", "3"))
    min_px = int(_arg("--min-px", "40"))
    meta = json.load(open(os.path.join(cap, "meta_data.json")))
    rgb = np.asarray(Image.open(os.path.join(cap, "rgb.png")).convert("RGB"))
    depth = np.load(os.path.join(cap, "depth.npy"))
    seg = np.asarray(Image.open(os.path.join(cap, "seg.png")), dtype=np.int32).copy()
    if label in meta["label_map"]:                       # redo, not pile up
        old = meta["label_map"].pop(label)
        seg[seg == old] = meta["label_map"][of]
    obj = seg == meta["label_map"][of]
    if not obj.any():
        print("NOT FOUND"); sys.exit(2)
    # The object alone, enlarged: a 16 px patch is most of a handle seen from
    # a metre away, so the object's box is cut out and scored on its own at a
    # scale that gives the handle several patches.
    ys, xs = np.where(obj)
    pad = 24
    y0, y1 = max(0, ys.min() - pad), min(rgb.shape[0], ys.max() + pad + 1)
    x0, x1 = max(0, xs.min() - pad), min(rgb.shape[1], xs.max() + pad + 1)
    crop = rgb[y0:y1, x0:x1]
    from curobo.examples.getting_started.feature_mapping import CRadioInference
    model = CRadioInference(text_adaptor_name="clip")
    prompts = [part] + others + ["floor"]
    prob = score_map(model, np.ascontiguousarray(crop), prompts, scale)   # (P, h, w)
    full = np.zeros((len(prompts),) + rgb.shape[:2], np.float32)
    full[:, y0:y1, x0:x1] = prob
    best = full.argmax(axis=0)
    mask = (best == 0) & obj
    print(f"[part] '{part}' wins {int(mask.sum())} of the object's {int(obj.sum())} px against {prompts[1:]}")
    xyz = transform_xyz(depth_to_camera_xyz(depth, np.asarray(meta["intrinsics"], float)),
                        np.asarray(meta["camera_pose"], float))
    cands = components(mask, min_px, depth=depth)
    if not cands:
        print("NOT FOUND"); sys.exit(2)
    seed = max(cands, key=lambda m: float(full[0][m].sum()))
    # The words light up a piece of the handle (147 of 1718 px on the lying
    # hammer, results/fable6/near/obj_part_overlay.png). The handle is a
    # geometric thing once the seed says which end: the thin run of the
    # object along its long axis on the seed's side of the thick part.
    # First the object itself is completed -- the grown label stopped 12 cm
    # from the words' centre and lost the end of the handle -- with the
    # points standing on the floor near it.
    valid = depth > 0
    floor_z = float(np.percentile(xyz[valid][:, 2], 3))
    c0 = xyz[obj].mean(axis=0)
    near = valid & (xyz[..., 2] > floor_z + 0.006) & (np.hypot(xyz[..., 0] - c0[0], xyz[..., 1] - c0[1]) < 0.30)
    from scipy import ndimage
    lab, _ = ndimage.label(near | obj)
    obj_full = np.isin(lab, list(set(np.unique(lab[obj])) - {0})) & near
    P = xyz[obj_full]
    ctr = P.mean(axis=0)
    axis = np.linalg.svd(P[:, :2] - ctr[:2], full_matrices=False)[2][0]     # long axis, in the floor plane
    t = (P[:, :2] - ctr[:2]) @ axis
    perp = np.abs((P[:, :2] - ctr[:2]) @ np.array([-axis[1], axis[0]]))
    bins = np.linspace(t.min(), t.max(), 25)
    idx = np.clip(np.digitize(t, bins) - 1, 0, len(bins) - 2)
    width = np.array([np.percentile(perp[idx == b], 90) if (idx == b).sum() > 3 else 0.0 for b in range(len(bins) - 1)])
    thin = width < 0.6 * width.max()
    ts = (xyz[seed][:, :2] - ctr[:2]) @ axis
    b_seed = int(np.clip(np.digitize(np.median(ts), bins) - 1, 0, len(bins) - 2))
    if not thin[b_seed]:                                  # words landed on the thick part: take the thin side nearest
        cand = [b for b in range(len(thin)) if thin[b]]
        b_seed = min(cand, key=lambda b: abs(b - b_seed)) if cand else b_seed
    lo = hi = b_seed
    while lo - 1 >= 0 and thin[lo - 1]:
        lo -= 1
    while hi + 1 < len(thin) and thin[hi + 1]:
        hi += 1
    keep_bins = np.zeros(len(thin), bool); keep_bins[lo:hi + 1] = True
    chosen = np.zeros_like(obj)
    chosen[obj_full] = keep_bins[idx]
    print(f"[part] object completed to {int(obj_full.sum())} px; long axis {np.round(axis, 2)}, widths "
          f"{np.round(width * 100, 1).tolist()} cm; thin run bins {lo}..{hi} of {len(thin)} -> {int(chosen.sum())} px")
    centre = xyz[chosen].mean(axis=0)
    new_id = int(max(meta["label_map"].values())) + 1
    seg[chosen] = new_id
    Image.fromarray(seg.astype(np.int32), mode="I").save(os.path.join(cap, "seg.png"))
    meta["label_map"][label] = new_id
    meta.setdefault("language_labels", {})[label] = {
        "query": part, "of": of, "label": label, "pixels": int(chosen.sum()),
        "centre": [float(v) for v in centre], "p": float(full[0][chosen].mean())}
    json.dump(meta, open(os.path.join(cap, "meta_data.json"), "w"), indent=2)
    over = rgb.copy()
    over[obj] = (0.6 * over[obj] + 0.4 * np.array([255, 255, 0])).astype(np.uint8)
    over[chosen] = (0.3 * over[chosen] + 0.7 * np.array([255, 0, 255])).astype(np.uint8)
    Image.fromarray(over[max(0, y0 - 40):y1 + 40, max(0, x0 - 40):x1 + 40]).resize(
        ((x1 - x0 + 80) * 3, (y1 - y0 + 80) * 3), Image.NEAREST).save(os.path.join(cap, f"{label}_overlay.png"))
    print(f"FOUND {label} {int(chosen.sum())} {centre[0]:.3f} {centre[1]:.3f} {centre[2]:.3f} p={float(full[0][chosen].mean()):.2f}")


if __name__ == "__main__":
    main()
