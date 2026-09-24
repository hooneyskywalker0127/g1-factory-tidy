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
# OBJECT_MU env: GraspGenX validates with 10.0 on the object, but there the
# finger effort limit is 1000 Nm-equivalent (g1_right_arm.yaml finger_effort
# limit) and the fingers drag anything. With the real 1.4 Nm the same mu glues
# a 0.2 kg cylinder to the floor (20 N of friction) and the thumb stalls on
# contact without closing (measured: thumb_1 at 0.69 of 0.72 open on every
# candidate). A floor and a tool are not mu 10.
OBJECT_MU = float(os.environ.get("OBJECT_MU", "10.0"))
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
    # Continuous collision detection on the target: pressed into the floor by
    # a hand whose body is placed rather than simulated, a 0.2 kg tin went
    # through the floor in one 1 ms step (box at close -0.146 .. -0.281 on a
    # third of the candidates, and two "held" verdicts that were the tin
    # being flung back up from below the floor). CCD is what PhysX has for
    # exactly this; the scene enables it too (SimulationCfg physx.enable_ccd).
    from pxr import PhysxSchema
    PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateEnableCCDAttr(True)
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
    #
    # But one box the size of the whole desk makes the space under the top
    # solid, and the robot stands with its shins in that space: at the stand
    # the arm reaches from, four leg links sit inside the block and the
    # contact solver throws them out every step -- measured, the leg joints
    # moved 384 mrad between rendered frames for the whole pick (0.08 with
    # the legs straight down, clear of the block), which is the jittering
    # in every delivered video. Raising the pelvis 2 cm did not change it;
    # the floor was never the problem. Using the desk's own triangle mesh
    # instead freed the legs but the box slid off the grasp (LOST): the
    # tabletop contact the grasp was tuned against is this box's flat top.
    #
    # So keep the box for the tabletop and give the legs their own: the
    # slab is the mesh's top down to its underside, the legs are the four
    # clusters of vertices below it, each in its own bounding box. Same top
    # face as before, and air where the desk has air.
    lo, hi = m.bounds
    v = m.vertices
    top = hi[2]
    near_top = v[v[:, 2] > top - 0.10]
    slab_bottom = float(near_top[:, 2].min())
    below = v[v[:, 2] < slab_bottom - 0.01]
    parts = [((lo[0], lo[1], slab_bottom), (hi[0], hi[1], top))]
    if len(below):
        c = (lo[:2] + hi[:2]) / 2.0
        for qx in (-1.0, 1.0):
            for qy in (-1.0, 1.0):
                q = below[((below[:, 0] - c[0]) * qx > 0)
                          & ((below[:, 1] - c[1]) * qy > 0)]
                if len(q):
                    parts.append(((q[:, 0].min(), q[:, 1].min(), lo[2]),
                                  (q[:, 0].max(), q[:, 1].max(), slab_bottom)))
    for k, (a, b) in enumerate(parts):
        a, b = np.asarray(a, float), np.asarray(b, float)
        box = UsdGeom.Cube.Define(stage, f"{path}/Collider{k}")
        box.CreateSizeAttr(2.0)
        bxf = UsdGeom.Xformable(box.GetPrim())
        bxf.AddTranslateOp().Set(Gf.Vec3d(*((a + b) / 2.0)))
        bxf.AddScaleOp().Set(Gf.Vec3f(*((b - a) / 2.0)))
        UsdGeom.Imageable(box.GetPrim()).CreateVisibilityAttr("invisible")
        UsdPhysics.CollisionAPI.Apply(box.GetPrim())
        _tight_contact(box.GetPrim())
    print(f"[scene] support '{name}' collides as a slab "
          f"{top - slab_bottom:.3f} m thick and {len(parts) - 1} legs")
    top = (T @ np.append(m.bounds[1], 1.0))[2]
    print(f"[scene] support '{name}' origin {np.round(T[:3, 3], 3)} top z {top:.3f}")


def floor_slab(stage, z_top=0.0, thickness=0.5, half=6.0):
    """A thick static box just under the floor.

    The cell's floor is a mesh collider, one triangle thick: a 0.2 kg tin
    pressed on it by the hand went through in one step (box at close -0.146
    .. -0.281 on a third of the candidates, and "held" verdicts that were the
    tin flung back up from below), and continuous collision detection did
    not stop it -- it is a slow push, not a fast one. A box has an inside.
    """
    from pxr import UsdGeom as _UG, UsdPhysics as _UP, Gf as _Gf
    prim = _UG.Cube.Define(stage, "/World/FloorSlab")
    prim.CreateSizeAttr(1.0)
    xf = _UG.Xformable(prim.GetPrim())
    xf.AddTranslateOp().Set(_Gf.Vec3d(0.0, 0.0, z_top - thickness / 2.0))
    xf.AddScaleOp().Set(_Gf.Vec3d(2 * half, 2 * half, thickness))
    prim.CreatePurposeAttr(_UG.Tokens.guide)                      # never rendered
    _UP.CollisionAPI.Apply(prim.GetPrim())
    return prim.GetPrim()


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


def keep_only_hand_collisions(stage, robot_path="/World/G1", keep=("right_hand", "right_wrist"),
                              cell_paths=("/World/Cell", "/World/GraspTarget", "/World/Props", "/World/Plan")):
    """Let nothing of the robot but the right hand collide with the cell.

    The body is written into place every substep (walk, kneel and reach are
    references replayed kinematically), and a kneel puts the knees and feet a
    few millimetres into the floor. PhysX answers those penetrations with
    contact impulses through the whole articulation, and the only joints free
    to take them are the simulated fingers: measured on a kneeling reach, the
    right fingers sat at their limits (index_0 at 0, thumb_1 at 1.01) with
    reported velocities of -2.5..-7.7 rad/s before the close had begun, and
    no drive at 1.4 Nm moved them. The robot's USD is instanceable, so its
    colliders cannot be edited one by one; PhysX collision groups filter by
    path instead: group "body" (the robot minus the hand) never collides
    with group "cell" (floor, props, the target). The hand is left out of
    "body" and collides with everything. Returns the link paths kept.
    """
    from pxr import Usd, UsdPhysics
    root = stage.GetPrimAtPath(robot_path)
    kept = [str(c.GetPath()) for c in root.GetChildren() if any(k in c.GetName() for k in keep)]
    body = UsdPhysics.CollisionGroup.Define(stage, "/World/ColGroupBody")
    cell = UsdPhysics.CollisionGroup.Define(stage, "/World/ColGroupCell")
    bc = body.GetCollidersCollectionAPI()
    bc.CreateIncludesRel().AddTarget(robot_path)
    for k in kept:
        bc.CreateExcludesRel().AddTarget(k)
    cc = cell.GetCollidersCollectionAPI()
    for c in cell_paths:
        if stage.GetPrimAtPath(c):
            cc.CreateIncludesRel().AddTarget(c)
    body.CreateFilteredGroupsRel().AddTarget(cell.GetPath())
    cell.CreateFilteredGroupsRel().AddTarget(body.GetPath())
    return kept
