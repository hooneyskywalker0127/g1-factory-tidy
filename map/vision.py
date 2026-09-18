# Head camera -> where the thing on the floor is.
#
# The only input is the depth image. Depth is unprojected into world points, the
# floor is subtracted, and what stands on it is the object. No pose is read from
# the stage and none is passed in.
#
# Two things here ARE prior knowledge of the room rather than of the object, and
# they should be named: the floor is taken as z = 0 instead of being fitted, and
# points outside the cell's own footprint are dropped so the walls and the racks
# do not count as objects. A robot in an unmapped room would have to earn both.
import math

import numpy as np
import torch
from isaaclab.sensors.camera.utils import create_pointcloud_from_depth
from isaaclab.utils.math import convert_camera_frame_orientation_convention
from pxr import UsdGeom

NEAR_DEPTH = 0.40       # the depth sensor returns nothing closer than this
FLOOR_EPS = 0.03        # points this near z=0 are the floor
MAX_OBJ_Z = 0.80        # taller than anything that can sit on this floor
SELF_R = 0.55           # this close to the lens is the robot's own arms


def camera_pose(stage, prim_path):
    """Read the camera's pose off the stage, in the ros convention.

    The sensor's own pos_w/quat_w buffers read all zeros on a cpu device --
    Fabric is disabled there and that is the path they are filled through. An
    all-zero quaternion turns every unprojected point into NaN, which silently
    empties the cloud instead of raising, so the pose is taken from the stage.
    """
    m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(prim_path))
    t = m.ExtractTranslation()
    r = m.ExtractRotationQuat()
    pos = np.array([t[0], t[1], t[2]], dtype=np.float32)
    q_gl = torch.tensor([[r.GetReal(), *r.GetImaginary()]], dtype=torch.float32)
    quat = convert_camera_frame_orientation_convention(
        q_gl, origin="opengl", target="ros")[0].numpy()
    return pos, quat


def find_object(cam, pos, quat, floor_half):
    """The object standing on the floor, from one depth frame.

    Returns None when nothing is in view. `floor_half` is the cell's (x, y)
    half-extent, used only to drop the walls and racks.
    """
    depth = cam.data.output["distance_to_image_plane"][0].cpu().numpy().squeeze()
    d = depth.copy()
    d[~np.isfinite(d)] = 0.0
    d[d < NEAR_DEPTH] = 0.0
    K = cam.data.intrinsic_matrices[0].cpu().numpy()
    pts = np.asarray(create_pointcloud_from_depth(K, d, position=pos, orientation=quat))
    pts = pts[np.isfinite(pts).all(axis=1)]
    if not len(pts):
        return None

    keep = ((pts[:, 2] > FLOOR_EPS) & (pts[:, 2] < MAX_OBJ_Z) &
            (np.abs(pts[:, 0]) < floor_half[0] - 0.12) &
            (np.abs(pts[:, 1]) < floor_half[1] - 0.12) &
            (np.linalg.norm(pts - pos, axis=1) > SELF_R))
    obj = pts[keep]
    if len(obj) < 30:
        return None

    lo, hi = obj.min(axis=0), obj.max(axis=0)
    return {"points": obj,
            "centre": (lo + hi) / 2.0,
            "size": hi - lo,
            "top_z": float(hi[2])}


def grip_frame(obj, from_xy):
    """Where the two hands go, given where the robot is standing.

    The cloud gives an axis-aligned box, not the object's own yaw, so the grip
    is built off the approach instead: the hands close along the axis across the
    line of approach, and how far apart they start is the object's own width
    measured along that same axis -- from the points, not from a known size.
    """
    c = obj["centre"]
    a = np.array([c[0] - from_xy[0], c[1] - from_xy[1]])
    n = np.linalg.norm(a)
    if n < 1e-6:
        return None
    a = a / n
    lat = np.array([-a[1], a[0]])                  # across the approach
    proj = (obj["points"][:, 0] - c[0]) * lat[0] + (obj["points"][:, 1] - c[1]) * lat[1]
    half = float(np.percentile(np.abs(proj), 98))  # ignore a few stray points
    return {"approach": a, "lateral": lat, "half_width": half,
            "centre": c, "grip_z": float(obj["top_z"] * 0.5)}
