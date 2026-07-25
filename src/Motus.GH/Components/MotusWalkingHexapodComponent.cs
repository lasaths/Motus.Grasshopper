using Grasshopper.Kernel;
using Motus.Core;
using Motus.Geometry;
using Motus.GH.Data;
using Motus.GH.Params;
using Motus.GH.Planning;
using Motus.GH.Preview;
using Motus.GH.Rhino;
using Rhino.Display;
using Rhino.Geometry;
using System.Drawing;

namespace Motus.GH.Components;

/// <summary>
/// Walking / mobile hexapod (6×3-DOF legs: coxa, femur, tibia) — NOT Stewart/Gough.
/// Motus.NET has no dedicated walking-hex stack; this builds a branching
/// <see cref="KinematicTree"/> for TreeFK preview. Plan uses one tip-path leg only
/// (side legs are preview-only), same contract as Motus Joint Table.
/// </summary>
public sealed class MotusWalkingHexapodComponent : RobotSourceComponentBase
{
    private List<Color> _previewColors = [];
    private readonly Dictionary<Color, DisplayMaterial> _matCache = new();

    public static readonly Guid Id = new("b8e2c4f1-8a3d-4c7e-9f1b-5d6e7a8b9c0d");

    /// <summary>Leg order (mithi-style labels).</summary>
    public static readonly string[] LegNames =
    [
        "right-middle", "right-front", "left-front",
        "left-middle", "left-back", "right-back",
    ];

    public MotusWalkingHexapodComponent()
        : base(
            "Motus Walking Hex",
            "WalkHex",
            "Walking hexapod (6× coxa/femur/tibia). Wire Path for tripod gait Trajectory → Preview; Plan = tip-path one leg only.",
            "polygon")
    {
    }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddNumberParameter("BodyR", "Br", "Body hex radius to hip (m)", GH_ParamAccess.item, 0.12);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Coxa", "Cx", "Coxa length (m)", GH_ParamAccess.item, 0.06);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Femur", "Fm", "Femur length (m)", GH_ParamAccess.item, 0.17);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Tibia", "Tb", "Tibia length (m)", GH_ParamAccess.item, 0.19);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("HipStance", "Hs", "Coxa stance angle (rad, signed by leg side)", GH_ParamAccess.item, 7.5 * Math.PI / 180.0);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("FemurStance", "Fs", "Femur stance angle (rad)", GH_ParamAccess.item, 30.0 * Math.PI / 180.0);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("TibiaStance", "Ts", "Tibia stance angle (rad)", GH_ParamAccess.item, -30.0 * Math.PI / 180.0);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("BodyZ", "Bz", "Body height above ground (m)", GH_ParamAccess.item, 0.12);
        p[p.ParamCount - 1].Optional = true;
        p.AddCurveParameter("Path", "P", "Walk path (Curve or Planes list, m) — gait Trajectory when wired", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddPlaneParameter("Planes", "Pl", "Optional path as plane list (origins, m)", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Speed", "Spd", "Walk speed along path (m/s)", GH_ParamAccess.item, 0.08);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Step", "St", "Nominal step length (m)", GH_ParamAccess.item, 0.06);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Lift", "Lf", "Swing foot lift (m)", GH_ParamAccess.item, 0.03);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Q", "Q", "Optional full driver q (18): leg-major coxa,femur,tibia × 6", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddGenericParameter("Robot", "Rb", "Robot (gait=18-DOF; Plan tip-path = right-middle 3-DOF only)", GH_ParamAccess.item);
        p.AddParameter(new Param_MotusJointState(), "State", "Js", "Full 18-DOF stance (TreeFK / documentation)", GH_ParamAccess.item);
        p.AddParameter(new Param_MotusTrajectory(), "Trajectory", "Tr", "Gait trajectory when Path/Planes wired (18-DOF q + mobile base)", GH_ParamAccess.item);
        p.AddCurveParameter("PathCurve", "Pc", "Resolved walk path curve (m)", GH_ParamAccess.item);
        p.AddPlaneParameter("PathPlanes", "Pp", "Body planes sampled along path", GH_ParamAccess.list);
        p.AddMeshParameter("Meshes", "M", "Preview meshes (body + legs + COG)", GH_ParamAccess.list);
        p.AddCurveParameter("Support", "Sp", "Support polygon (foot tips)", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var br = 0.12;
        var cx = 0.06;
        var fm = 0.17;
        var tb = 0.19;
        var hs = 7.5 * Math.PI / 180.0;
        var fs = 30.0 * Math.PI / 180.0;
        var ts = -30.0 * Math.PI / 180.0;
        var bz = 0.12;
        var speed = 0.08;
        var stepLen = 0.06;
        var lift = 0.03;
        var qIn = new List<double>();
        Curve? pathCurve = null;
        var pathPlanes = new List<Plane>();
        da.GetData(0, ref br);
        da.GetData(1, ref cx);
        da.GetData(2, ref fm);
        da.GetData(3, ref tb);
        da.GetData(4, ref hs);
        da.GetData(5, ref fs);
        da.GetData(6, ref ts);
        da.GetData(7, ref bz);
        da.GetData(8, ref pathCurve);
        da.GetDataList(9, pathPlanes);
        da.GetData(10, ref speed);
        da.GetData(11, ref stepLen);
        da.GetData(12, ref lift);
        da.GetDataList(13, qIn);

        var hasPath = (pathCurve is not null && pathCurve.IsValid) || pathPlanes.Count >= 2;

        if (!double.IsFinite(br) || !double.IsFinite(cx) || !double.IsFinite(fm) || !double.IsFinite(tb)
            || !double.IsFinite(hs) || !double.IsFinite(fs) || !double.IsFinite(ts) || !double.IsFinite(bz)
            || !double.IsFinite(speed) || !double.IsFinite(stepLen) || !double.IsFinite(lift))
        {
            ClearPreview();
            _previewColors = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "BodyR / Coxa / Femur / Tibia / stance / BodyZ / Speed / Step / Lift must be finite (no NaN/Inf).");
            return;
        }

        if (qIn.Count is > 0 and < 18)
        {
            ClearPreview();
            _previewColors = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Q must be empty (stance defaults) or exactly 18 driver values — partial lists are rejected.");
            return;
        }

        if (qIn.Any(v => !double.IsFinite(v)))
        {
            ClearPreview();
            _previewColors = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Q contains non-finite values (NaN/Inf).");
            return;
        }

        if (br <= 0 || cx <= 0 || fm <= 0 || tb <= 0)
        {
            ClearPreview();
            _previewColors = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "BodyR / Coxa / Femur / Tibia must be > 0.");
            return;
        }

        try
        {
            var q = BuildStanceQ(hs, fs, ts, qIn);
            var desc = BuildDescription(br, cx, fm, tb, bz);
            var tree = desc.ToKinematicTree();
            const string tipLink = "right-middle_tibia";
            var tip = tree.ExtractSerialTip("body", tipLink);
            var driverNames = DriverNames(tree);
            var allLimits = LimitsAllDrivers(tree);

            RobotPreset preset;
            SerialJointChain? chain;
            JointState previewHome;
            JointState? treeDriverHome;
            if (hasPath)
            {
                preset = new RobotPreset
                {
                    Manufacturer = RobotManufacturer.Unknown,
                    ModelName = "walking_hexapod_gait",
                    Family = "serial",
                    AxisCount = tree.DriverCount,
                    JointLimits = allLimits,
                    BaseFrame = BaseFrame.Identity,
                    ToolFrame = ToolFrame.Identity,
                    Notes = "Walking hex gait (18-DOF). Use Trajectory → Preview — not UR MoveJ / not Plan tip-path.",
                    SourceNote = "Motus Walking Hex gait",
                };
                chain = null;
                previewHome = new JointState(q);
                treeDriverHome = null;
            }
            else
            {
                var tipLimits = LimitsAlongTip(tree, tip.JointNames);
                preset = new RobotPreset
                {
                    Manufacturer = RobotManufacturer.Unknown,
                    ModelName = "walking_hexapod",
                    Family = "serial",
                    AxisCount = tip.Chain.Joints.Length,
                    JointLimits = tipLimits,
                    BaseFrame = BaseFrame.Identity,
                    ToolFrame = tip.TipToolOffset is { } off
                        ? new ToolFrame(off, "foot")
                        : ToolFrame.Identity,
                    Notes = "Walking hexapod (NOT Stewart). Plan/Joint State = tip path (one leg). Wire Path for gait Trajectory.",
                    SourceNote = "Motus Walking Hex",
                };
                chain = tip.Chain;
                var tipHome = new double[preset.AxisCount];
                for (var i = 0; i < tipHome.Length && i < 3; i++)
                    tipHome[i] = q[i];
                previewHome = new JointState(tipHome);
                treeDriverHome = new JointState(q);
            }

            var model = new RobotModel(preset, jointNames: driverNames);
            var goo = new RobotModelGoo(model)
            {
                Chain = chain,
                Tree = tree,
                PreviewHome = previewHome,
                TreeDriverHome = treeDriverHome,
                PreviewGeometry = MechanismPreviewGeometry.Build(desc),
            };

            TrajectoryGoo? trajGoo = null;
            Curve? pathCurveOut = null;
            List<Plane> pathPlanesOut = [];
            if (hasPath)
            {
                if (!WalkingHexGait.TryBuild(
                        pathCurve, pathPlanes, speed, stepLen, lift, hs, fs, ts, model,
                        out var gait, out var gaitErr))
                {
                    ClearPreview();
                    _previewColors = [];
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, gaitErr);
                    return;
                }

                trajGoo = new TrajectoryGoo(gait!.Trajectory)
                {
                    Tree = tree,
                    PreviewGeometry = goo.PreviewGeometry,
                    PreviewMeshColors = goo.PreviewMeshColors,
                    TreeDriverHome = new JointState(q),
                    BasePath = gait.BasePath,
                };
                pathCurveOut = gait.PathCurve;
                pathPlanesOut = gait.PathPlanes.ToList();
                if (gait.Warning is { } warn)
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, warn);
            }
            else
            {
                AddRuntimeMessage(
                    GH_RuntimeMessageLevel.Remark,
                    "Wire Path (curve) or Planes (≥2) for tripod gait Trajectory → Preview. Plan moves one tip leg only.");
            }

            var preview = WalkingHexPreview.Build(br, cx, fm, tb, bz, q);
            _previewMeshes = preview.Meshes.ToList();
            _previewWires = preview.Wires.ToList();
            _previewColors = preview.Colors.ToList();
            ExpirePreview(true);

            if (!hasPath && tree.DriverCount != tip.Chain.Joints.Length)
            {
                AddRuntimeMessage(
                    GH_RuntimeMessageLevel.Remark,
                    $"Walking hex: Plan tip-path '{tipLink}' = {tip.Chain.Joints.Length} axes; TreeFK has {tree.DriverCount} drivers. Not Stewart.");
            }

            da.SetData(0, goo);
            da.SetData(1, new JointStateGoo(new JointState(q)));
            da.SetData(2, trajGoo);
            da.SetData(3, pathCurveOut);
            da.SetDataList(4, pathPlanesOut);
            da.SetDataList(5, _previewMeshes);
            da.SetData(6, preview.SupportPolygon);
        }
        catch (Exception ex)
        {
            ClearPreview();
            _previewColors = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
        }
    }

    public override void DrawViewportMeshes(IGH_PreviewArgs args)
    {
        if (Locked || _previewMeshes.Count == 0) return;
        for (var i = 0; i < _previewMeshes.Count; i++)
        {
            var c = i < _previewColors.Count ? _previewColors[i] : Color.White;
            if (!_matCache.TryGetValue(c, out var mat))
            {
                mat = new DisplayMaterial(c) { Transparency = 0.2 };
                _matCache[c] = mat;
            }
            args.Display.DrawMeshShaded(_previewMeshes[i], mat);
        }
    }

    public override void DrawViewportWires(IGH_PreviewArgs args)
    {
        if (Locked) return;
        if (_previewWires.Count > 0)
        {
            foreach (var line in _previewWires)
                args.Display.DrawLine(line, Color.FromArgb(200, 255, 140, 40), 2);
            return;
        }
        base.DrawViewportWires(args);
    }

    public override Guid ComponentGuid => Id;

    private static double[] BuildStanceQ(double hip, double femur, double tibia, List<double> overrideQ)
    {
        var q = new double[18];
        if (overrideQ.Count >= 18)
        {
            for (var i = 0; i < 18; i++) q[i] = overrideQ[i];
            return q;
        }

        for (var leg = 0; leg < 6; leg++)
        {
            var side = LegIsLeft(leg) ? 1.0 : -1.0;
            q[leg * 3 + 0] = leg * (Math.PI / 3.0) + side * hip;
            q[leg * 3 + 1] = femur;
            q[leg * 3 + 2] = tibia;
        }
        return q;
    }

    private static bool LegIsLeft(int legIndex) =>
        LegNames[legIndex].StartsWith("left", StringComparison.Ordinal);

    /// <summary>Hip yaw angles (rad) around +Z for each leg, body frame.</summary>
    private static double LegYaw(int legIndex) =>
        legIndex * (Math.PI / 3.0);

    private static RobotDescription BuildDescription(
        double bodyR, double coxa, double femur, double tibia, double bodyZ)
    {
        var links = new List<UrdfLink>
        {
            new("body",
                [UrdfGeometry.Cylinder(bodyR * 0.85, 0.03, new Frame(0, 0, bodyZ))],
                r: 1, g: 0.4, b: 0.7, a: 0.85),
        };
        var joints = new List<UrdfJoint>();

        for (var leg = 0; leg < 6; leg++)
        {
            var name = LegNames[leg];
            var yaw = LegYaw(leg);
            var hx = bodyR * Math.Cos(yaw);
            var hy = bodyR * Math.Sin(yaw);
            var coxaLink = $"{name}_coxa";
            var femurLink = $"{name}_femur";
            var tibiaLink = $"{name}_tibia";

            links.Add(new UrdfLink(coxaLink, [UrdfGeometry.Cylinder(0.012, coxa, new Frame(coxa * 0.5, 0, 0))], r: 1, g: 0.55, b: 0.15, a: 1));
            links.Add(new UrdfLink(femurLink, [UrdfGeometry.Cylinder(0.012, femur, new Frame(femur * 0.5, 0, 0))], r: 1, g: 0.55, b: 0.15, a: 1));
            links.Add(new UrdfLink(tibiaLink, [UrdfGeometry.Cylinder(0.010, tibia, new Frame(tibia * 0.5, 0, 0))], r: 1, g: 0.55, b: 0.15, a: 1));

            // Hip: revolute about body Z at hip socket. Axis Z; child coxa extends along local X after joint.
            joints.Add(new UrdfJoint($"{name}_hip", "revolute", "body", coxaLink,
                hx, hy, bodyZ, 0, 0, 1, -Math.PI, Math.PI));
            // Femur: pitch about Y at end of coxa
            joints.Add(new UrdfJoint($"{name}_femur", "revolute", coxaLink, femurLink,
                coxa, 0, 0, 0, 1, 0, -Math.PI, Math.PI));
            // Tibia: pitch about Y at end of femur
            joints.Add(new UrdfJoint($"{name}_tibia", "revolute", femurLink, tibiaLink,
                femur, 0, 0, 0, 1, 0, -Math.PI, Math.PI));
        }

        if (!RobotDescription.TryAssemble("walking_hexapod", links, joints, tipLink: "right-middle_tibia",
                out var desc, out var diag, homeQ: null) || desc is null)
            throw new InvalidOperationException(string.Join("; ", diag.Errors));

        return desc;
    }

    private static List<JointLimit> LimitsAllDrivers(KinematicTree tree)
    {
        var limits = new List<JointLimit>(tree.DriverCount);
        for (var di = 0; di < tree.DriverCount; di++)
        {
            var j = tree.Joints[tree.DriverJointIndices[di]];
            var vel = j.Velocity ?? Math.PI;
            limits.Add(new JointLimit(j.Lower, j.Upper, vel, vel * 2));
        }
        return limits;
    }

    private static string[] DriverNames(KinematicTree tree)
    {
        var names = new string[tree.DriverCount];
        for (var di = 0; di < tree.DriverCount; di++)
            names[di] = tree.Joints[tree.DriverJointIndices[di]].Name;
        return names;
    }

    private static List<JointLimit> LimitsAlongTip(KinematicTree tree, IReadOnlyList<string> tipJointNames)
    {
        var byName = new Dictionary<string, KinematicJoint>(StringComparer.OrdinalIgnoreCase);
        foreach (var j in tree.Joints)
            byName[j.Name] = j;

        var limits = new List<JointLimit>(tipJointNames.Count);
        foreach (var name in tipJointNames)
        {
            if (!byName.TryGetValue(name, out var j))
                throw new InvalidOperationException($"Tip joint '{name}' missing.");
            var vel = j.Velocity ?? Math.PI;
            limits.Add(new JointLimit(j.Lower, j.Upper, vel, vel * 2));
        }
        return limits;
    }
}

/// <summary>Mithi-style walking-hex viewport meshes (body hex, orange legs, COG, support poly).</summary>
internal static class WalkingHexPreview
{
    public readonly record struct Result(
        IReadOnlyList<Mesh> Meshes,
        IReadOnlyList<Color> Colors,
        IReadOnlyList<Line> Wires,
        Curve? SupportPolygon);

    public static Result Build(
        double bodyR, double coxa, double femur, double tibia, double bodyZ, double[] q18)
    {
        var meshes = new List<Mesh>();
        var colors = new List<Color>();
        var wires = new List<Line>();
        var feet = new List<Point3d>();

        var bodyColor = Color.FromArgb(200, 255, 105, 180);
        var legColor = Color.FromArgb(220, 255, 140, 40);
        var jointColor = Color.FromArgb(230, 255, 160, 70);
        var cogColor = Color.FromArgb(230, 0, 200, 100);

        // Body hex plate
        var bodyPts = new Point3d[6];
        for (var i = 0; i < 6; i++)
        {
            var a = i * (Math.PI / 3.0) + Math.PI / 6.0;
            bodyPts[i] = new Point3d(bodyR * 0.9 * Math.Cos(a), bodyR * 0.9 * Math.Sin(a), bodyZ);
        }
        if (HexSlab(bodyPts, 0.02) is { } bodyMesh)
        {
            meshes.Add(bodyMesh);
            colors.Add(bodyColor);
        }

        var cog = new Point3d(0, 0, bodyZ);
        if (Mesh.CreateFromSphere(new Sphere(cog, 0.025), 10, 8) is { } cogMesh)
        {
            meshes.Add(cogMesh);
            colors.Add(cogColor);
        }

        const double segR = 0.012;
        const double jointR = 0.018;
        for (var leg = 0; leg < 6; leg++)
        {
            var yaw0 = leg * (Math.PI / 3.0);
            var hip = new Point3d(bodyR * Math.Cos(yaw0), bodyR * Math.Sin(yaw0), bodyZ);
            var coxaA = q18[leg * 3 + 0];
            var femurA = q18[leg * 3 + 1];
            var tibiaA = q18[leg * 3 + 2];

            // Coxa direction: q hip channel already includes mount yaw φᵢ
            var coxaDir = new Vector3d(Math.Cos(coxaA), Math.Sin(coxaA), 0);
            var knee = hip + coxaDir * coxa;

            // Femur / tibia pitch in the vertical plane of coxaDir
            var up = Vector3d.ZAxis;
            var lat = Vector3d.CrossProduct(up, coxaDir);
            if (!lat.Unitize()) lat = Vector3d.YAxis;
            var femurDir = coxaDir * Math.Cos(femurA) + up * Math.Sin(femurA);
            femurDir.Unitize();
            var ankle = knee + femurDir * femur;

            var tibiaDir = coxaDir * Math.Cos(femurA + tibiaA) + up * Math.Sin(femurA + tibiaA);
            tibiaDir.Unitize();
            var foot = ankle + tibiaDir * tibia;
            feet.Add(foot);

            AddSeg(meshes, colors, wires, hip, knee, segR, jointR, legColor, jointColor);
            AddSeg(meshes, colors, wires, knee, ankle, segR, jointR, legColor, jointColor);
            AddSeg(meshes, colors, wires, ankle, foot, segR * 0.85, jointR * 0.85, legColor, jointColor);
            if (Mesh.CreateFromSphere(new Sphere(foot, jointR * 0.7), 8, 6) is { } footMesh)
            {
                meshes.Add(footMesh);
                colors.Add(jointColor);
            }
        }

        Curve? support = null;
        if (feet.Count >= 3)
        {
            var poly = new Polyline(feet);
            poly.Add(feet[0]);
            support = poly.ToNurbsCurve();
            // Support fill on ground
            var ground = feet.Select(f => new Point3d(f.X, f.Y, 0)).ToArray();
            if (HexSlab(ground, 0.004) is { } supportMesh)
            {
                meshes.Add(supportMesh);
                colors.Add(Color.FromArgb(90, 80, 120, 160));
            }
        }

        return new Result(meshes, colors, wires, support);
    }

    private static void AddSeg(
        List<Mesh> meshes, List<Color> colors, List<Line> wires,
        Point3d a, Point3d b, double r, double jr, Color leg, Color joint)
    {
        wires.Add(new Line(a, b));
        var len = a.DistanceTo(b);
        if (len > 1e-9)
        {
            var dir = b - a;
            dir.Unitize();
            var plane = new Plane(a, dir);
            if (Mesh.CreateFromCylinder(new Cylinder(new Circle(plane, r), len), 10, 1) is { } cyl)
            {
                meshes.Add(cyl);
                colors.Add(leg);
            }
        }
        if (Mesh.CreateFromSphere(new Sphere(a, jr), 8, 6) is { } ja)
        {
            meshes.Add(ja);
            colors.Add(joint);
        }
    }

    private static Mesh? HexSlab(Point3d[] ring, double thickness)
    {
        if (ring.Length < 3) return null;
        var c = Point3d.Origin;
        foreach (var p in ring) c += p;
        c /= ring.Length;
        var mesh = new Mesh();
        mesh.Vertices.Add(c);
        foreach (var p in ring) mesh.Vertices.Add(p);
        for (var i = 0; i < ring.Length; i++)
            mesh.Faces.AddFace(0, i + 1, i + 2 <= ring.Length ? i + 2 : 1);
        mesh.Normals.ComputeNormals();

        var top = mesh.DuplicateMesh();
        top.Transform(Transform.Translation(0, 0, thickness * 0.5));
        var bottom = mesh.DuplicateMesh();
        bottom.Transform(Transform.Translation(0, 0, -thickness * 0.5));
        bottom.Flip(true, true, true);
        var slab = new Mesh();
        slab.Append(top);
        slab.Append(bottom);
        slab.Normals.ComputeNormals();
        slab.Compact();
        return slab.IsValid ? slab : null;
    }
}
