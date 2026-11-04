"""USD workcell authoring for installed Isaac Sim 6.0 / PhysX.

The robot is a network of ordinary rigid-body joints (maximal coordinates),
not a tree articulation. All 12 spherical joints participate in the solver.
Only three revolute drive targets are actuated. No platform pose is commanded.
"""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, PhysxSchema

ROOT = Path(__file__).resolve().parents[1]


def quat(rotation):
    x, y, z, w = Rotation.from_matrix(np.asarray(rotation)).as_quat()
    return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))


def orient_axis(direction):
    """Rotation that maps +Z to a direction (for capsule geometry)."""
    d = np.asarray(direction, float)
    d /= np.linalg.norm(d)
    if d[2] < -0.999999:
        return Gf.Quatf(0, Gf.Vec3f(1, 0, 0))
    q = np.r_[np.cross([0, 0, 1], d), 1+d[2]]
    q /= np.linalg.norm(q)
    return Gf.Quatf(float(q[3]), Gf.Vec3f(*q[:3].tolist()))


class CellScene:
    def __init__(self, stage, cfg, geometry):
        self.stage, self.cfg, self.g = stage, cfg, geometry
        self.zbase = cfg["robot"]["base_height"]
        self.offset = np.array([0, 0, self.zbase])
        self.drives = []
        self.cap_paths, self.cap_colors = [], []
        self.body_origins = {}
        self.materials = {}

    def material(self, name, color, metallic=0.0, roughness=0.5):
        if name in self.materials:
            return self.materials[name]
        m = UsdShade.Material.Define(self.stage, f"/World/Materials/{name}")
        shader = UsdShade.Shader.Define(self.stage, f"{m.GetPath()}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        m.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        self.materials[name] = m
        return m

    def bind(self, prim, material):
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)

    def collision(self, prim, friction=0.5):
        UsdPhysics.CollisionAPI.Apply(prim)
        api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        api.CreateContactOffsetAttr(0.001)
        api.CreateRestOffsetAttr(0.0)
        mat = UsdShade.Material.Define(self.stage, f"/World/Materials/Physics_{int(friction*1000)}")
        phys = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        phys.CreateStaticFrictionAttr(friction)
        phys.CreateDynamicFrictionAttr(friction)
        phys.CreateRestitutionAttr(0.03)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, materialPurpose="physics")

    def box(self, path, position, size, material, collide=True):
        shape = UsdGeom.Cube.Define(self.stage, path)
        shape.CreateSizeAttr(1)
        shape.AddTranslateOp().Set(Gf.Vec3d(*np.asarray(position, float).tolist()))
        shape.AddScaleOp().Set(Gf.Vec3f(*np.asarray(size, float).tolist()))
        self.bind(shape.GetPrim(), material)
        if collide:
            self.collision(shape.GetPrim())
        return shape.GetPrim()

    def cylinder(self, path, position, radius, height, material, collide=True):
        shape = UsdGeom.Cylinder.Define(self.stage, path)
        shape.CreateRadiusAttr(radius)
        shape.CreateHeightAttr(height)
        shape.CreateAxisAttr("Z")
        shape.AddTranslateOp().Set(Gf.Vec3d(*np.asarray(position, float).tolist()))
        self.bind(shape.GetPrim(), material)
        if collide:
            self.collision(shape.GetPrim())
        return shape.GetPrim()

    def capsule(self, path, a, b, radius, material, collide=True):
        a, b = np.asarray(a), np.asarray(b)
        shape = UsdGeom.Capsule.Define(self.stage, path)
        shape.CreateRadiusAttr(radius)
        shape.CreateHeightAttr(float(np.linalg.norm(b-a)))
        shape.CreateAxisAttr("Z")
        shape.AddTranslateOp().Set(Gf.Vec3d(*((a+b)/2).tolist()))
        shape.AddOrientOp().Set(orient_axis(b-a))
        self.bind(shape.GetPrim(), material)
        if collide:
            self.collision(shape.GetPrim())
        return shape.GetPrim()

    def rigid(self, path, position, mass, inertia):
        prim = UsdGeom.Xform.Define(self.stage, path).GetPrim()
        UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*np.asarray(position, float).tolist()))
        UsdGeom.Xformable(prim).AddOrientOp().Set(Gf.Quatf(1))
        UsdPhysics.RigidBodyAPI.Apply(prim)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(mass)
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        m.CreateCenterOfMassAttr(Gf.Vec3f(0))
        api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        api.CreateSolverPositionIterationCountAttr(self.cfg["simulation"]["solver_position_iterations"])
        api.CreateSolverVelocityIterationCountAttr(self.cfg["simulation"]["solver_velocity_iterations"])
        api.CreateEnableCCDAttr(True)
        api.CreateSleepThresholdAttr(0.00001)
        self.body_origins[path] = np.asarray(position)
        return prim

    def ball_joint(self, name, body0, body1, anchor):
        joint = UsdPhysics.SphericalJoint.Define(self.stage, "/World/Robot/Joints/"+name)
        joint.CreateBody0Rel().SetTargets([body0])
        joint.CreateBody1Rel().SetTargets([body1])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*(anchor-self.body_origins[body0]).tolist()))
        joint.CreateLocalPos1Attr(Gf.Vec3f(*(anchor-self.body_origins[body1]).tolist()))
        joint.CreateExcludeFromArticulationAttr(True)
        joint.CreateCollisionEnabledAttr(False)

    def mesh(self, group, parent, origin, material, visuals):
        mesh = UsdGeom.Mesh.Define(self.stage, parent + "/CAD")
        vertices = visuals[group+"_vertices"] - origin
        triangles = visuals[group+"_triangles"]
        mesh.CreatePointsAttr(vertices.tolist())
        mesh.CreateFaceVertexCountsAttr([3]*len(triangles))
        mesh.CreateFaceVertexIndicesAttr(triangles.ravel().tolist())
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        self.bind(mesh.GetPrim(), material)

    def build_robot(self):
        g = self.g
        visuals = np.load(ROOT / "assets/generated/visuals.npz")
        steel = self.material("robot_steel", (0.30, 0.34, 0.38), 0.7, 0.3)
        paint = self.material("robot_paint", (0.72, 0.74, 0.76), 0.2)
        rubber = self.material("vacuum_rubber", (0.022, 0.026, 0.029))
        base = UsdGeom.Xform.Define(self.stage, "/World/Robot/Base")
        base.AddTranslateOp().Set(Gf.Vec3d(0, 0, self.zbase))
        self.mesh("base", str(base.GetPath()), np.zeros(3), steel, visuals)
        shoulders = np.array(g["shoulders"])
        elbows = np.array(g["elbows_reference"])
        p0 = np.array(g["platform_reference"])
        offsets = np.array(g["platform_offsets"])
        platform = "/World/Robot/Platform"
        self.rigid(platform, p0+self.offset, 0.16, [0.00008, 0.00008, 0.00012])
        self.mesh("platform", platform, p0, paint, visuals)
        self.cylinder(platform+"/PlateCollision", [0, 0, 0], 0.026, 0.008, paint)
        # Collision proxy is hidden while the original CAD plate remains visible.
        UsdGeom.Imageable(self.stage.GetPrimAtPath(platform+"/PlateCollision")).MakeInvisible()
        self.cylinder(platform+"/VacuumStem", [0, 0, -0.019], 0.004, 0.030, steel)
        self.cylinder(platform+"/VacuumCup", [0, 0, -0.035], 0.007, 0.006, rubber)
        for i in range(3):
            upper = f"/World/Robot/Upper_{i}"
            b = shoulders[i]
            self.rigid(upper, b+self.offset, 0.22, [0.0012, 0.0012, 0.0012])
            self.mesh(f"upper_{i}", upper, b, paint, visuals)
            proxy = self.capsule(upper+"/Collision", np.zeros(3), elbows[i].mean(0)-b, 0.0075, steel)
            UsdGeom.Imageable(proxy).MakeInvisible()
            joint = UsdPhysics.RevoluteJoint.Define(self.stage, f"/World/Robot/Joints/shoulder_{i}")
            joint.CreateBody1Rel().SetTargets([upper])
            joint.CreateAxisAttr("X")
            axis = np.array(g["axes"][i])
            # X of the joint frame is the measured tangent shoulder axis.
            frame = np.column_stack([axis, [0, 0, 1], np.cross(axis, [0, 0, 1])])
            joint.CreateLocalRot0Attr(quat(frame))
            joint.CreateLocalRot1Attr(quat(frame))
            joint.CreateLocalPos0Attr(Gf.Vec3f(*(b+self.offset).tolist()))
            joint.CreateLocalPos1Attr(Gf.Vec3f(0))
            joint.CreateLowerLimitAttr(float(np.rad2deg(g["joint_limits"][0]-g["q_reference"][i])))
            joint.CreateUpperLimitAttr(float(np.rad2deg(g["joint_limits"][1]-g["q_reference"][i])))
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(self.cfg["robot"]["drive_stiffness"])
            drive.CreateDampingAttr(self.cfg["robot"]["drive_damping"])
            drive.CreateMaxForceAttr(self.cfg["robot"]["max_torque"])
            drive.CreateTargetPositionAttr(0.0)
            self.drives.append(drive)
            for j in range(2):
                rod = f"/World/Robot/Rod_{i}_{j}"
                a, b2 = elbows[i, j], p0+offsets[i, j]
                origin = (a+b2)/2
                self.rigid(rod, origin+self.offset, 0.022, [0.00012]*3)
                self.mesh(f"rod_{i}_{j}", rod, origin, steel, visuals)
                proxy = self.capsule(rod+"/Collision", a-origin, b2-origin, 0.002, steel)
                UsdGeom.Imageable(proxy).MakeInvisible()
                self.ball_joint(f"elbow_{i}_{j}", upper, rod, a+self.offset)
                self.ball_joint(f"wrist_{i}_{j}", platform, rod, b2+self.offset)
        # Adjacent and crossing CAD proxies must not fight the ball constraints.
        paths = ["/World/Robot/Platform"] + [f"/World/Robot/Upper_{i}" for i in range(3)]
        paths += [f"/World/Robot/Rod_{i}_{j}" for i in range(3) for j in range(2)]
        for p in paths:
            UsdPhysics.FilteredPairsAPI.Apply(self.stage.GetPrimAtPath(p)).CreateFilteredPairsRel().SetTargets(
                [x for x in paths if x != p])

    def build(self, scenario="normal"):
        stage, cfg = self.stage, self.cfg
        world = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(world.GetPrim())
        UsdGeom.SetStageUpAxis(stage, "Z")
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        physics = UsdPhysics.Scene.Define(stage, "/World/Physics")
        physics.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        physics.CreateGravityMagnitudeAttr(9.81)
        api = PhysxSchema.PhysxSceneAPI.Apply(physics.GetPrim())
        api.CreateTimeStepsPerSecondAttr(240)
        api.CreateSolverTypeAttr("TGS")
        api.CreateEnableCCDAttr(True)
        api.CreateEnableGPUDynamicsAttr(False)
        api.CreateBroadphaseTypeAttr("MBP")
        concrete = self.material("concrete", (0.18, 0.21, 0.24))
        aluminium = self.material("aluminium", (0.45, 0.5, 0.56), 0.65, 0.35)
        dark = self.material("belt", (0.045, 0.048, 0.054))
        guard = self.material("guard", (0.20, 0.23, 0.27), 0.5)
        safety = self.material("safety", (0.95, 0.45, 0.015))
        self.box("/World/Floor", [-1, 0, -0.025], [6, 3, 0.05], concrete)
        for x in [-0.36, 0.36]:
            for y in [-0.32, 0.32]:
                self.box(f"/World/Frame/Post_{x}_{y}".replace("-", "m").replace(".", "p"),
                         [x, y, 0.64], [0.04, 0.04, 1.28], aluminium)
        for y in [-0.32, 0.32]:
            self.box(f"/World/Frame/Beam_{'a' if y < 0 else 'b'}", [0, y, 1.25], [0.76, 0.04, 0.04], aluminium)
        for x in [-0.22, 0.22]:
            self.box(f"/World/Frame/Cross_{'a' if x < 0 else 'b'}", [x, 0, 1.25], [0.03, 0.67, 0.03], aluminium)
        # Mounting standoffs join the original T-slot base to the support frame.
        for i, b in enumerate(np.array(self.g["shoulders"])):
            self.box(f"/World/Frame/Mount_{i}", [b[0], b[1], 1.228], [0.025, 0.025, 0.045], aluminium)
        length = cfg["conveyor"]["length"]
        centre_x = 0.3-length/2
        belt = self.box("/World/Conveyor/Belt", [centre_x, 0, cfg["conveyor"]["top"]-0.018],
                        [length, cfg["conveyor"]["width"], 0.036], dark)
        UsdPhysics.RigidBodyAPI.Apply(belt).CreateKinematicEnabledAttr(True)
        self.belt_surface = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(belt)
        self.belt_surface.CreateSurfaceVelocityAttr(Gf.Vec3f(0))
        self.belt_surface.CreateSurfaceVelocityLocalSpaceAttr(False)
        self.belt_surface.CreateSurfaceVelocityEnabledAttr(True)
        for sign in [-1, 1]:
            self.box(f"/World/Conveyor/Rail_{'a' if sign < 0 else 'b'}", [centre_x, sign*0.075, 0.78],
                     [length, 0.017, 0.045], aluminium)
        for i, x in enumerate([-2.8, -1.6, -0.4, 0.22]):
            for j, y in enumerate([-0.08, 0.08]):
                self.box(f"/World/Conveyor/Leg_{i}_{j}", [x, y, 0.38], [0.025, 0.025, 0.76], aluminium)
        for name, xy in cfg["bins"].items():
            mat = self.material("bin_"+name, tuple(np.array(cfg["caps"]["colors"][name])*0.55), 0.1)
            half = cfg["bin_geometry"]["inner_width"]/2
            bottom, top = cfg["bin_geometry"]["floor"], cfg["bin_geometry"]["rim"]
            self.box(f"/World/Bins/{name}/Floor", [*xy, bottom-0.002], [2*half+0.008, 2*half+0.008, 0.004], mat)
            for axis in [0, 1]:
                for side in [-1, 1]:
                    pos = [*xy, (top+bottom)/2]
                    pos[axis] += side*(half+0.002)
                    size = [2*half+0.008, 2*half+0.008, top-bottom]
                    size[axis] = 0.004
                    self.box(f"/World/Bins/{name}/Wall_{axis}_{'a' if side < 0 else 'b'}", pos, size, mat)
            self.box(f"/World/Bins/{name}/Support", [*xy, bottom/2], [0.035, 0.035, bottom], aluminium)
        # Guard perimeter outside the robot's swept envelope; belt passes through end slots.
        for side in [-1, 1]:
            for i, x in enumerate(np.linspace(-0.38, 0.38, 17)):
                self.box(f"/World/Guard/Bar_{side+1}_{i}", [float(x), side*0.35, 0.92], [0.003, 0.003, 0.55], guard)
            for j, z in enumerate(np.linspace(0.67, 1.2, 12)):
                self.box(f"/World/Guard/Rail_{side+1}_{j}", [0, side*0.35, float(z)], [0.8, 0.003, 0.003], guard)
            self.box(f"/World/Guard/Top_{side+1}", [0, side*0.35, 1.22], [0.8, 0.015, 0.025], safety)
        dome = UsdLux.DomeLight.Define(stage, "/World/Lighting/Dome")
        dome.CreateIntensityAttr(900)
        lamp = UsdLux.RectLight.Define(stage, "/World/Lighting/TaskLight")
        lamp.CreateIntensityAttr(1800)
        lamp.CreateWidthAttr(0.6)
        lamp.CreateHeightAttr(0.6)
        lamp.AddTranslateOp().Set(Gf.Vec3d(0.1, -0.1, 1.65))
        camera = UsdGeom.Camera.Define(stage, "/World/Camera")
        camera.AddTranslateOp().Set(Gf.Vec3d(*cfg["camera"]["position"]))
        camera.CreateFocalLengthAttr(cfg["camera"]["focal_length_mm"])
        camera.CreateHorizontalApertureAttr(cfg["camera"]["horizontal_aperture_mm"])
        camera.CreateVerticalApertureAttr(cfg["camera"]["horizontal_aperture_mm"] * cfg["camera"]["height"]/cfg["camera"]["width"])
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 20))
        overview = UsdGeom.Camera.Define(stage, "/World/Overview")
        view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(1.1, -1.5, 1.75), Gf.Vec3d(-0.16, 0, 0.78), Gf.Vec3d(0, 0, 1))
        overview.AddTransformOp().Set(view.GetInverse())
        overview.CreateFocalLengthAttr(22)
        overview.CreateHorizontalApertureAttr(24)
        overview.CreateVerticalApertureAttr(13.5)
        overview.CreateClippingRangeAttr(Gf.Vec2f(0.01, 30))
        self.box("/World/CameraHousing", np.array(cfg["camera"]["position"])+[0, 0, 0.014], [0.03, 0.03, 0.02], dark, False)
        self.capsule("/World/CameraMount", [0.33, 0, 1.13], [0, 0, 1.13], 0.006, aluminium, False)
        self.build_robot()
        rng = np.random.default_rng(cfg["caps"]["seed"])
        colors = list(cfg["caps"]["colors"])
        count = 0 if scenario == "empty" else cfg["caps"]["count"]
        for i in range(count):
            color = colors[i % len(colors)]
            path = f"/World/Caps/Cap_{i:03d}"
            x = -0.035 - i*cfg["caps"]["spacing"]
            y = float(rng.uniform(-0.03, 0.03))
            pos = [x, y, cfg["conveyor"]["top"]+cfg["caps"]["height"]/2+0.002]
            cap = self.rigid(path, pos, cfg["caps"]["mass"], [1.5e-7, 1.5e-7, 2.2e-7])
            q = Rotation.from_euler("xyz", [rng.uniform(-0.025, 0.025), rng.uniform(-0.025, 0.025), rng.uniform(-np.pi, np.pi)]).as_quat()
            UsdGeom.Xformable(cap).GetOrderedXformOps()[1].Set(Gf.Quatf(float(q[3]), Gf.Vec3f(*q[:3].tolist())))
            rgb = cfg["caps"]["colors"][color] if scenario != "uncertain" else [0.35, 0.35, 0.35]
            mat = self.material("cap_"+color, rgb, 0.0, 0.55)
            solid = self.cylinder(path+"/Shell", [0, 0, 0], cfg["caps"]["radius"], cfg["caps"]["height"], mat)
            self.collision(solid, cfg["caps"]["friction"])
            # A thin lip makes the cap readable; physics uses a convex cylindrical proxy.
            self.cylinder(path+"/Top", [0, 0, cfg["caps"]["height"]/2-0.0005], cfg["caps"]["radius"]*0.9, 0.001, mat, False)
            self.cap_paths.append(path)
            self.cap_colors.append(color)
        return self
