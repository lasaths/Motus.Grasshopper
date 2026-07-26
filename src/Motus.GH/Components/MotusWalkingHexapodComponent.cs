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
/// Thin hex wrapper over Motus.NET <see cref="LeggedLayout"/> / <see cref="LeggedGait"/> / <see cref="LegIk3R"/>.
/// Plan uses one tip-path leg only (side legs preview-only), same contract as Motus Joint Table.
/// </summary>
public sealed class MotusWalkingHexapodComponent : RobotSourceComponentBase
{
    /// <summary>Alias for <see cref="Units.LeggedFamily"/>.</summary>
    public const string LeggedFamily = Units.LeggedFamily;

    private List<Color> _previewColors = [];
    private List<Circle> _previewContactCircles = [];
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
            "Walking hexapod (6× coxa/femur/tibia, Family=legged). Wire Path → foot-target IK gait Trajectory → Preview (no Motus Plan). Tip-path Plan = one leg only.",
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
        p.AddNumberParameter("Step", "St", "Nominal step length along path (m) — sets gait cadence", GH_ParamAccess.item, 0.06);
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
            _previewContactCircles = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "BodyR / Coxa / Femur / Tibia / stance / BodyZ / Speed / Step / Lift must be finite (no NaN/Inf).");
            return;
        }

        if (qIn.Count is > 0 and < 18)
        {
            ClearPreview();
            _previewColors = [];
            _previewContactCircles = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Q must be empty (stance defaults) or exactly 18 driver values — partial lists are rejected.");
            return;
        }

        if (qIn.Any(v => !double.IsFinite(v)))
        {
            ClearPreview();
            _previewColors = [];
            _previewContactCircles = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Q contains non-finite values (NaN/Inf).");
            return;
        }

        if (br <= 0 || cx <= 0 || fm <= 0 || tb <= 0)
        {
            ClearPreview();
            _previewColors = [];
            _previewContactCircles = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "BodyR / Coxa / Femur / Tibia must be > 0 (m).");
            return;
        }

        try
        {
            var layout = LeggedLayout.HexMithi(br, cx, fm, tb, bz);
            var q = BuildStanceQ(layout, hs, fs, ts, qIn);
            var desc = BuildDescription(layout);
            var tree = desc.ToKinematicTree();
            var tipLink = layout.TipLinkName;
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
                    Family = Units.LeggedFamily,
                    AxisCount = tree.DriverCount,
                    JointLimits = allLimits,
                    BaseFrame = BaseFrame.Identity,
                    ToolFrame = ToolFrame.Identity,
                    Notes = "Walking hex gait (18-DOF, Family=legged). Use Trajectory → Preview — not UR MoveJ / not Plan tip-path. Q = joint radians.",
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
                    Family = Units.LeggedFamily,
                    AxisCount = tip.Chain.Joints.Length,
                    JointLimits = tipLimits,
                    BaseFrame = BaseFrame.Identity,
                    ToolFrame = tip.TipToolOffset is { } off
                        ? new ToolFrame(off, "foot")
                        : ToolFrame.Identity,
                    Notes = "Walking hexapod Family=legged (NOT Stewart). Plan/Joint State = tip path (one leg). Wire Path for gait Trajectory. Q = joint radians.",
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
                if (!LeggedGaitRhino.TryBuild(
                        layout, pathCurve, pathPlanes, speed, stepLen, lift,
                        hs, fs, ts, model,
                        out var gait, out var gaitErr))
                {
                    ClearPreview();
                    _previewColors = [];
                    _previewContactCircles = [];
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
                    "Wire Path (curve) or Planes (≥2) for foot-target IK gait Trajectory → Preview. No Motus Plan on this path — Plan moves one tip leg only.");
            }

            var previewQ = q;
            Frame? previewBase = null;
            if (hasPath && trajGoo?.BasePath is { Count: > 0 } bp
                && trajGoo.Value is { Points.Count: > 0 } traj)
            {
                var mid = traj.Points.Count / 2;
                previewQ = traj.Points[mid].JointState.Positions.ToArray();
                previewBase = bp[Math.Min(mid, bp.Count - 1)];
            }

            var preview = WalkingHexPreview.Build(layout, previewQ, previewBase);
            _previewMeshes = preview.Meshes.ToList();
            _previewWires = preview.Wires.ToList();
            _previewColors = preview.Colors.ToList();
            _previewContactCircles = preview.ContactCircles.ToList();
            ExpirePreview(true);

            if (!hasPath && tree.DriverCount != tip.Chain.Joints.Length)
            {
                AddRuntimeMessage(
                    GH_RuntimeMessageLevel.Remark,
                    $"Walking hex: Plan tip-path '{tipLink}' = {tip.Chain.Joints.Length} axes; TreeFK has {tree.DriverCount} drivers. Family=legged (not Stewart).");
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
            _previewContactCircles = [];
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
        var contactColor = Color.FromArgb(220, 80, 200, 120);
        foreach (var c in _previewContactCircles)
            args.Display.DrawCircle(c, contactColor, 2);
        if (_previewWires.Count > 0)
        {
            foreach (var line in _previewWires)
                args.Display.DrawLine(line, Color.FromArgb(200, 255, 140, 40), 2);
            return;
        }
        base.DrawViewportWires(args);
    }

    public override Guid ComponentGuid => Id;

    private static double[] BuildStanceQ(
        LeggedLayout layout, double hip, double femur, double tibia, List<double> overrideQ)
    {
        if (overrideQ.Count >= layout.DriverCount)
        {
            var q = new double[layout.DriverCount];
            for (var i = 0; i < q.Length; i++) q[i] = overrideQ[i];
            return q;
        }

        return LeggedGaitRhino.BuildStanceQ(layout, hip, femur, tibia);
    }

    private static RobotDescription BuildDescription(LeggedLayout layout)
    {
        var links = new List<UrdfLink>
        {
            new("body",
                [UrdfGeometry.Box(layout.BodyR * 1.6, layout.BodyR * 1.1, 0.04, new Frame(0, 0, layout.BodyZ))],
                r: 1, g: 0.4, b: 0.7, a: 0.85),
        };
        var joints = new List<UrdfJoint>();

        for (var leg = 0; leg < layout.LegCount; leg++)
        {
            var name = layout.LegNames[leg];
            var yaw = layout.HipYawsRad[leg];
            var hx = layout.BodyR * Math.Cos(yaw);
            var hy = layout.BodyR * Math.Sin(yaw);
            var coxa = layout.Coxa;
            var femur = layout.Femur;
            var tibia = layout.Tibia;
            var bodyZ = layout.BodyZ;
            var coxaLink = $"{name}_coxa";
            var femurLink = $"{name}_femur";
            var tibiaLink = $"{name}_tibia";

            links.Add(new UrdfLink(coxaLink, [UrdfGeometry.Cylinder(0.012, coxa, new Frame(coxa * 0.5, 0, 0))], r: 1, g: 0.55, b: 0.15, a: 1));
            links.Add(new UrdfLink(femurLink, [UrdfGeometry.Cylinder(0.012, femur, new Frame(femur * 0.5, 0, 0))], r: 1, g: 0.55, b: 0.15, a: 1));
            links.Add(new UrdfLink(tibiaLink, [UrdfGeometry.Cylinder(0.010, tibia, new Frame(tibia * 0.5, 0, 0))], r: 1, g: 0.55, b: 0.15, a: 1));

            joints.Add(new UrdfJoint($"{name}_hip", "revolute", "body", coxaLink,
                hx, hy, bodyZ, 0, 0, 1, -Math.PI, Math.PI));
            joints.Add(new UrdfJoint($"{name}_femur", "revolute", coxaLink, femurLink,
                coxa, 0, 0, 0, 1, 0, -Math.PI, Math.PI));
            joints.Add(new UrdfJoint($"{name}_tibia", "revolute", femurLink, tibiaLink,
                femur, 0, 0, 0, 1, 0, -Math.PI, Math.PI));
        }

        if (!RobotDescription.TryAssemble("walking_hexapod", links, joints, tipLink: layout.TipLinkName,
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

/// <summary>Walking-hex viewport meshes: rectangular body plate, orange legs, ground-contact rings.</summary>
internal static class WalkingHexPreview
{
    public readonly record struct Result(
        IReadOnlyList<Mesh> Meshes,
        IReadOnlyList<Color> Colors,
        IReadOnlyList<Line> Wires,
        IReadOnlyList<Circle> ContactCircles,
        Curve? SupportPolygon);

    public static Result Build(LeggedLayout layout, double[] q, Frame? baseFrame = null)
    {
        var meshes = new List<Mesh>();
        var colors = new List<Color>();
        var wires = new List<Line>();
        var contacts = new List<Circle>();
        var feet = new List<Point3d>();

        var bodyColor = Color.FromArgb(200, 255, 105, 180);
        var legColor = Color.FromArgb(220, 255, 140, 40);
        var jointColor = Color.FromArgb(230, 255, 160, 70);
        var contactColor = Color.FromArgb(180, 80, 200, 120);

        var bodyR = layout.BodyR;
        var bodyZ = layout.BodyZ;
        var coxa = layout.Coxa;
        var femur = layout.Femur;
        var tibia = layout.Tibia;
        var n = layout.LegCount;

        // Rectangular body slab (replaces hex plate + COG sphere).
        var hx = bodyR * 0.8;
        var hy = bodyR * 0.55;
        var hz = 0.02;
        var bodyBox = new Box(
            new Plane(new Point3d(0, 0, bodyZ), Vector3d.ZAxis),
            new Interval(-hx, hx),
            new Interval(-hy, hy),
            new Interval(-hz, hz));
        if (Mesh.CreateFromBox(bodyBox, 1, 1, 1) is { } bodyMesh)
        {
            meshes.Add(bodyMesh);
            colors.Add(bodyColor);
        }

        const double segR = 0.012;
        const double jointR = 0.018;
        for (var leg = 0; leg < n; leg++)
        {
            var yaw0 = layout.HipYawsRad[leg];
            var hip = new Point3d(bodyR * Math.Cos(yaw0), bodyR * Math.Sin(yaw0), bodyZ);
            var coxaA = q[leg * 3 + 0];
            var femurA = q[leg * 3 + 1];
            var tibiaA = q[leg * 3 + 2];

            var coxaDir = new Vector3d(Math.Cos(coxaA), Math.Sin(coxaA), 0);
            var knee = hip + coxaDir * coxa;

            var up = Vector3d.ZAxis;
            var femurDir = coxaDir * Math.Cos(femurA) - up * Math.Sin(femurA);
            femurDir.Unitize();
            var ankle = knee + femurDir * femur;

            var tibiaDir = coxaDir * Math.Cos(femurA + tibiaA) - up * Math.Sin(femurA + tibiaA);
            tibiaDir.Unitize();
            var foot = ankle + tibiaDir * tibia;
            feet.Add(foot);

            AddSeg(meshes, colors, wires, hip, knee, segR, jointR, legColor, jointColor);
            AddSeg(meshes, colors, wires, knee, ankle, segR, jointR, legColor, jointColor);
            AddSeg(meshes, colors, wires, ankle, foot, segR * 0.85, jointR * 0.85, legColor, jointColor);

            if (Math.Abs(foot.Z) <= LeggedContactPreview.GroundTolMeters)
            {
                var ground = new Point3d(foot.X, foot.Y, 0);
                contacts.Add(new Circle(new Plane(ground, Vector3d.ZAxis), LeggedContactPreview.RingRadiusMeters));
                if (Mesh.CreateFromCylinder(
                        new Cylinder(new Circle(new Plane(ground, Vector3d.ZAxis), LeggedContactPreview.RingRadiusMeters), 0.004),
                        16, 1) is { } pad)
                {
                    meshes.Add(pad);
                    colors.Add(contactColor);
                }
            }
        }

        Curve? support = null;
        if (feet.Count >= 3)
        {
            var poly = new Polyline(feet);
            poly.Add(feet[0]);
            support = poly.ToNurbsCurve();
        }

        if (baseFrame is { } bf)
        {
            var xform = BodyWorldXform(bf);
            for (var i = 0; i < meshes.Count; i++)
                meshes[i].Transform(xform);
            for (var i = 0; i < wires.Count; i++)
            {
                var a = wires[i].From;
                var b = wires[i].To;
                a.Transform(xform);
                b.Transform(xform);
                wires[i] = new Line(a, b);
            }
            for (var i = 0; i < contacts.Count; i++)
            {
                var c = contacts[i].Center;
                c.Transform(xform);
                c = new Point3d(c.X, c.Y, 0);
                contacts[i] = new Circle(new Plane(c, Vector3d.ZAxis), contacts[i].Radius);
            }
            if (support is not null)
                support.Transform(xform);
        }

        return new Result(meshes, colors, wires, contacts, support);
    }

    private static Transform BodyWorldXform(Frame baseFrame)
    {
        var yaw = 2.0 * Math.Atan2(baseFrame.Qz, baseFrame.Qw);
        var plane = new Plane(
            new Point3d(baseFrame.X, baseFrame.Y, 0),
            new Vector3d(Math.Cos(yaw), Math.Sin(yaw), 0),
            new Vector3d(-Math.Sin(yaw), Math.Cos(yaw), 0));
        return Transform.PlaneToPlane(Plane.WorldXY, plane);
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
}
