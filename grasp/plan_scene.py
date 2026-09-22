"""The plan's scene, rebuilt in the cell.

A GraspGenX plan carries the surface it assumed under the object and the
object itself, both as offsets from the robot's torso (see
grasp/traj_from_graspgen.py). Both grasp/play_in_cell.py and
grasp/capture_rgbd.py have to put them back in the same place, so they live
here rather than in either one.

Import only after the Isaac app is running -- pxr and trimesh come with it.
"""
import os

import numpy as np
import trimesh
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

# Contact values GraspGenX validates its own grasps under, from
# end2end/e2e_grasp_demo.py: --object_mu default 10.0, --finger_mu default 3.0.
# PhysX's own default is 0.5, and a grasp generated for mu 10 slips straight
# out at 0.5 -- which is what every replay here was showing.
OBJECT_MU = 10.0
FINGER_MU = 3.0

# torso_link sits at a fixed offset from the pelvis: the waist joints are at 0
# in the default pose and a plan that only moves the arm never touches them.
# Measured off the G1 URDF.
PELVIS_TO_TORSO = np.array([-0.00396, 0.0, 0.044])


def torso_pose(pelvis_xyz, yaw_rad):
    """Where torso_link stands, given where the robot's root is put."""
    T = np.eye(4)
    c, s = np.cos(yaw_rad), np.sin(yaw_rad)
    T[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    T[:3, 3] = np.asarray(pelvis_xyz, dtype=float) + T[:3, :3] @ PELVIS_TO_TORSO
    return T


def _tight_contact(mesh_prim):
    """Zero the collider's rest offset.

    PhysX keeps bodies apart by the collider's rest offset, and the default
    left the box hovering 14 mm over the table -- measured, not guessed:
    spawned with its underside 0.7 mm above the top, it was pushed to 13.7 mm
    on the first physics step and stayed there.
    """
    api = PhysxSchema.PhysxCollisionAPI.Apply(mesh_prim)
    api.CreateRestOffsetAttr(0.0)
    api.CreateContactOffsetAttr(0.002)


def spawn_target(stage, mesh_path, T_torso, T_in_torso):
    """Put the object the plan reached for at the same offset from the torso.

    The plan was made in the arm-only robot's torso frame; the cell puts that
    torso somewhere else, so the object moves with it. Visual only -- this
    replays a plan, it does not re-simulate the grasp.
    """
    m = trimesh.load(mesh_path, force="mesh")
    T = T_torso @ np.asarray(T_in_torso, dtype=np.float64)
    prim = UsdGeom.Xform.Define(stage, "/World/GraspTarget").GetPrim()
    # translate + orient, not a single matrix op: PhysX writes a simulated
    # body's pose back through those two ops, and a body carrying only a
    # matrix op never moves on the stage no matter what the sim does.
    q = Gf.Matrix4d(T.T.tolist()).ExtractRotationQuat()
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*T[:3, 3]))
    xf.AddOrientOp().Set(Gf.Quatf(q))
    mesh = UsdGeom.Mesh.Define(stage, "/World/GraspTarget/Mesh")
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in m.vertices.astype(float)])
    mesh.CreateFaceVertexCountsAttr([3] * len(m.faces))
    mesh.CreateFaceVertexIndicesAttr(m.faces.reshape(-1).tolist())
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.45, 0.15)])
    # A rigid body, not scenery: the point of replaying here is to see whether
    # the hand holds it. Same treatment map/props.py gives the cartons.
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(0.2)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(
        "convexHull")
    # Friction, from the values GraspGenX validates its own grasps under
    # (end2end/e2e_grasp_demo.py --object_mu, default 10.0). Left unset the
    # stage runs on PhysX's 0.5, and a grasp generated for mu 10 slips out the
    # moment the fingers touch it -- which is what the replays were showing.
    mat = UsdShade.Material.Define(stage, "/World/GraspTarget/PhysicsMaterial")
    UsdPhysics.MaterialAPI.Apply(mat.GetPrim()).CreateStaticFrictionAttr(OBJECT_MU)
    UsdPhysics.MaterialAPI(mat.GetPrim()).CreateDynamicFrictionAttr(OBJECT_MU)
    UsdPhysics.MaterialAPI(mat.GetPrim()).CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
        mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    _tight_contact(mesh.GetPrim())
    bottom = (T @ np.append(m.bounds[0], 1.0))[2]
    print(f"[scene] target at {np.round(T[:3, 3], 3)} bottom z {bottom:.3f} "
          f"from {os.path.basename(mesh_path)}")


def spawn_support(stage, name, mesh_path, T_torso, T_in_torso):
    """The surface the plan assumed under the object -- static, with colliders.

    Without it the object is spawned in mid-air in the cell and just falls.
    """
    m = trimesh.load(mesh_path, force="mesh")
    T = T_torso @ np.asarray(T_in_torso, dtype=np.float64)
    path = f"/World/PlanSupport_{name}"
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    UsdGeom.Xformable(prim).AddTransformOp().Set(Gf.Matrix4d(T.T.tolist()))
    mesh = UsdGeom.Mesh.Define(stage, path + "/Mesh")
    mesh.CreatePointsAttr([Gf.Vec3f(*v) for v in m.vertices.astype(float)])
    mesh.CreateFaceVertexCountsAttr([3] * len(m.faces))
    mesh.CreateFaceVertexIndicesAttr(m.faces.reshape(-1).tolist())
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.72, 0.66, 0.52)])
    # Collide against a box matching the mesh's extent, not a decomposition of
    # the mesh. The decomposed hulls sat 14 mm proud of the tabletop and held
    # the object in mid-air (measured: object pushed from 0.7647 to 0.7781
    # against a 0.7640 top). cuRobo plans against this table as a
    # `cuboid_from_extents` too, so the box is also what the plan assumed.
    lo, hi = m.bounds
    box = UsdGeom.Cube.Define(stage, path + "/Collider")
    box.CreateSizeAttr(2.0)
    bxf = UsdGeom.Xformable(box.GetPrim())
    bxf.AddTranslateOp().Set(Gf.Vec3d(*((lo + hi) / 2.0)))
    bxf.AddScaleOp().Set(Gf.Vec3f(*((hi - lo) / 2.0)))
    UsdGeom.Imageable(box.GetPrim()).CreateVisibilityAttr("invisible")
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    _tight_contact(box.GetPrim())
    top = (T @ np.append(m.bounds[1], 1.0))[2]
    print(f"[scene] support '{name}' origin {np.round(T[:3, 3], 3)} top z {top:.3f}")


def build(stage, app, meta, T_torso):
    """Put the plan's support surfaces and its target into the stage.

    Call before the first ``sim.reset()``: PhysX builds its scene there, and a
    rigid body added afterwards is never simulated -- it hangs frozen in
    mid-air.
    """
    if "object" not in meta:
        return
    for sup in meta.get("support", []):
        spawn_support(stage, sup["name"], sup["mesh"], T_torso,
                      sup["transform_in_torso"])
    spawn_target(stage, meta["object"]["mesh"], T_torso,
                 meta["object"]["transform_in_torso"])
    for _ in range(3):
        app.update()
