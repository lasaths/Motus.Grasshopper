using Motus.Core;
using Motus.Geometry;
using Rhino.Geometry;

namespace Motus.Rhino;

public static class KinematicsPreview
{
    public static DhForwardKinematics? TryFk(RobotModel robot) =>
        KinematicsProfiles.TryGet(robot.Preset, out _) ? new DhForwardKinematics(robot.Preset) : null;

    public static BaseFrame ResolveBase(RobotModel robot, BaseFrame? ghBase) => ghBase ?? robot.Preset.BaseFrame;

    public static ToolFrame ResolveTool(RobotModel robot, ToolFrame? ghTool) => ghTool ?? robot.Preset.ToolFrame;

    public static Point3d ToPoint(Frame frame) => new(frame.X, frame.Y, frame.Z);

    public static IEnumerable<Line> LinkLines(RobotModel robot, JointState state, BaseFrame? baseOverride = null, ToolFrame? toolOverride = null)
    {
        var fk = TryFk(robot);
        if (fk is null) return LegacyStickLinks(state);

        var baseF = ResolveBase(robot, baseOverride);
        var tool = ResolveTool(robot, toolOverride);
        var origins = fk.ComputeLinkOrigins(state.Positions, baseF.Frame);
        var lines = new List<Line>();
        var prev = ToPoint(baseF.Frame);
        foreach (var origin in origins)
        {
            var pt = ToPoint(origin);
            lines.Add(new Line(prev, pt));
            prev = pt;
        }
        lines.Add(new Line(prev, ToPoint(fk.ComputeTcp(state, baseF, tool).Tcp)));
        return lines;
    }

    public static IEnumerable<Mesh> LinkMeshes(RobotModel robot, JointState state, BaseFrame? baseOverride = null, ToolFrame? toolOverride = null)
    {
        var fk = TryFk(robot);
        if (fk is null) yield break;

        var baseF = ResolveBase(robot, baseOverride);
        var tool = ResolveTool(robot, toolOverride);
        var origins = fk.ComputeLinkOrigins(state.Positions, baseF.Frame);
        var radii = fk.LinkRadiiMeters;

        // Thin structural links follow the real chain: base -> joint centres -> TCP.
        var prev = ToPoint(baseF.Frame);
        for (var i = 0; i < origins.Count; i++)
        {
            var pt = ToPoint(origins[i]);
            var mesh = CylinderMesh(prev, pt, radii[i] * 0.45);
            if (mesh is not null) yield return mesh;
            prev = pt;
        }
        var tcp = ToPoint(fk.ComputeTcp(state, baseF, tool).Tcp);
        var toolMesh = CylinderMesh(prev, tcp, radii[^1] * 0.35);
        if (toolMesh is not null) yield return toolMesh;

        // Fat barrels mark each joint, aligned to its rotation axis (the DH frame's Z),
        // so the skeleton reads as an articulated arm rather than a bare polyline.
        for (var i = 0; i < origins.Count; i++)
        {
            var plane = FrameConversion.ToPlane(origins[i]);
            var jointMesh = BarrelMesh(plane.Origin, plane.ZAxis, radii[i], radii[i] * 1.6);
            if (jointMesh is not null) yield return jointMesh;
        }
    }

    public static Plane TcpPlane(RobotModel robot, JointState state, BaseFrame? baseOverride = null, ToolFrame? toolOverride = null)
    {
        var fk = TryFk(robot);
        var baseF = ResolveBase(robot, baseOverride);
        var tool = ResolveTool(robot, toolOverride);
        if (fk is null) return FrameConversion.ToPlane(baseF.Frame);
        return FrameConversion.ToPlane(fk.ComputeTcp(state, baseF, tool).Tcp);
    }

    public static Polyline TcpPath(RobotModel robot, IEnumerable<JointState> states, BaseFrame? baseOverride = null, ToolFrame? toolOverride = null)
    {
        var fk = TryFk(robot);
        var baseF = ResolveBase(robot, baseOverride);
        var tool = ResolveTool(robot, toolOverride);
        var pts = new List<Point3d>();
        foreach (var s in states)
        {
            if (fk is null)
            {
                pts.Add(s.Positions.Length > 0 ? new Point3d(Math.Cos(s.Positions[0]) * 0.15, Math.Sin(s.Positions[0]) * 0.15, 0) : Point3d.Origin);
                continue;
            }
            pts.Add(ToPoint(fk.ComputeTcp(s, baseF, tool).Tcp));
        }
        return pts.Count < 2 ? new Polyline() : new Polyline(pts);
    }

    public static void TrajectorySegments(
        RobotModel robot,
        Trajectory trajectory,
        TrajectoryValidationOptions? validation,
        out List<Line> valid,
        out List<Line> invalid,
        BaseFrame? baseOverride = null,
        ToolFrame? toolOverride = null)
    {
        valid = new List<Line>();
        invalid = new List<Line>();
        if (trajectory.Points.Count < 2) return;

        var fk = TryFk(robot);
        var baseF = ResolveBase(robot, baseOverride);
        var tool = ResolveTool(robot, toolOverride);
        var validator = new TrajectoryValidator();
        var opts = validation ?? new TrajectoryValidationOptions();

        Point3d TcpAt(JointState s)
        {
            if (fk is null) return LegacyStickLinks(s).LastOrDefault().To;
            return ToPoint(fk.ComputeTcp(s, baseF, tool).Tcp);
        }

        for (var i = 1; i < trajectory.Points.Count; i++)
        {
            var a = trajectory.Points[i - 1].JointState;
            var b = trajectory.Points[i].JointState;
            var seg = new Line(TcpAt(a), TcpAt(b));
            var mini = new Trajectory(robot, new[] { new TrajectoryPoint(0, a), new TrajectoryPoint(1, b) });
            if (validator.Validate(mini, opts).IsValid) valid.Add(seg);
            else invalid.Add(seg);
        }
    }

    private static Mesh? CylinderMesh(Point3d from, Point3d to, double radius)
    {
        var length = from.DistanceTo(to);
        if (length < 1e-6 || radius <= 0) return null;
        var dir = to - from;
        dir.Unitize();
        var plane = new Plane(from, dir);
        return Mesh.CreateFromCylinder(new Cylinder(new Circle(plane, radius), length), 12, 1);
    }

    // Barrel centred on a joint, extending +/- halfLength along its rotation axis.
    private static Mesh? BarrelMesh(Point3d center, Vector3d axis, double radius, double halfLength)
    {
        if (radius <= 0 || halfLength <= 0 || !axis.Unitize()) return null;
        var basePlane = new Plane(center - axis * halfLength, axis);
        return Mesh.CreateFromCylinder(new Cylinder(new Circle(basePlane, radius), halfLength * 2), 16, 1);
    }

  // ponytail: 2.5D fallback when preset has no kinematics profile
    private static IEnumerable<Line> LegacyStickLinks(JointState state)
    {
        const double linkLength = 0.12;
        var angle = 0.0;
        var prev = Point3d.Origin;
        for (var i = 0; i < state.Positions.Length; i++)
        {
            angle += state.Positions[i];
            var next = new Point3d(prev.X + Math.Cos(angle) * linkLength, prev.Y + Math.Sin(angle) * linkLength, prev.Z + (i % 2 == 0 ? linkLength * 0.2 : 0));
            yield return new Line(prev, next);
            prev = next;
        }
    }
}
