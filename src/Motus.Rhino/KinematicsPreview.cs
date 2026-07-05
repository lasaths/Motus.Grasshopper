using Motus.Core;
using Motus.Geometry;
using Rhino.Geometry;

namespace Motus.Rhino;

public static class KinematicsPreview
{
    public static IFkSolver? TryFk(RobotModel robot, SerialJointChain? chain = null)
    {
        try { return KinematicsResolver.CreateFkSolver(robot.Preset, chain); }
        catch (InvalidOperationException) { return null; }
    }

    public static Point3d ToPoint(Frame frame) => new(frame.X, frame.Y, frame.Z);

    public static IEnumerable<Line> LinkLines(
        RobotModel robot, JointState state, SerialJointChain? chain = null,
        BaseFrame? baseFrame = null, ToolFrame? toolFrame = null)
    {
        if (TryFk(robot, chain) is not { } fk) return [];

        var baseF = baseFrame ?? robot.Preset.BaseFrame;
        var tool = toolFrame ?? robot.Preset.ToolFrame;
        var origins = fk.ComputeLinkOrigins(state.Positions, baseF.Frame);
        var lines = new List<Line>();
        var prev = ToPoint(baseF.Frame);
        foreach (var origin in origins)
        {
            var pt = ToPoint(origin);
            lines.Add(new Line(prev, pt));
            prev = pt;
        }

        var tcp = ToPoint(fk.ComputeTcp(state, baseF, tool).Tcp);
        if (prev.DistanceTo(tcp) > 1e-6)
            lines.Add(new Line(prev, tcp));
        return lines;
    }

    public static IEnumerable<Mesh> LinkMeshes(
        RobotModel robot, JointState state, SerialJointChain? chain = null,
        BaseFrame? baseFrame = null, ToolFrame? toolFrame = null)
    {
        if (TryFk(robot, chain) is not { } fk) yield break;

        var baseF = baseFrame ?? robot.Preset.BaseFrame;
        var tool = toolFrame ?? robot.Preset.ToolFrame;
        var origins = fk.ComputeLinkOrigins(state.Positions, baseF.Frame);
        var radii = fk.LinkRadiiMeters;

        var prev = ToPoint(baseF.Frame);
        for (var i = 0; i < origins.Count; i++)
        {
            var pt = ToPoint(origins[i]);
            var mesh = CylinderMesh(prev, pt, radii[i] * 0.45);
            if (mesh is not null) yield return mesh;
            prev = pt;
        }

        var tcp = ToPoint(fk.ComputeTcp(state, baseF, tool).Tcp);
        if (prev.DistanceTo(tcp) > 1e-6)
        {
            var toolMesh = CylinderMesh(prev, tcp, radii[^1] * 0.35);
            if (toolMesh is not null) yield return toolMesh;
        }

        for (var i = 0; i < origins.Count; i++)
        {
            var plane = FrameConversion.ToPlane(origins[i]);
            var jointMesh = BarrelMesh(plane.Origin, plane.ZAxis, radii[i], radii[i] * 1.6);
            if (jointMesh is not null) yield return jointMesh;
        }
    }

    public static Plane TcpPlane(
        RobotModel robot, JointState state, SerialJointChain? chain = null,
        BaseFrame? baseFrame = null, ToolFrame? toolFrame = null)
    {
        var baseF = baseFrame ?? robot.Preset.BaseFrame;
        if (TryFk(robot, chain) is not { } fk)
            return FrameConversion.ToPlane(baseF.Frame);
        var tool = toolFrame ?? robot.Preset.ToolFrame;
        return FrameConversion.ToPlane(fk.ComputeTcp(state, baseF, tool).Tcp);
    }

    public static Polyline TcpPath(
        RobotModel robot, IEnumerable<JointState> states, SerialJointChain? chain = null,
        BaseFrame? baseFrame = null, ToolFrame? toolFrame = null)
    {
        if (TryFk(robot, chain) is not { } fk) return new Polyline();

        var baseF = baseFrame ?? robot.Preset.BaseFrame;
        var tool = toolFrame ?? robot.Preset.ToolFrame;
        var pts = states.Select(s => ToPoint(fk.ComputeTcp(s, baseF, tool).Tcp)).ToList();
        return pts.Count < 2 ? new Polyline() : new Polyline(pts);
    }

    public static void TrajectorySegments(
        RobotModel robot,
        Trajectory trajectory,
        TrajectoryValidationOptions? validation,
        out List<Line> valid,
        out List<Line> invalid,
        SerialJointChain? chain = null,
        BaseFrame? baseFrame = null,
        ToolFrame? toolFrame = null)
    {
        valid = new List<Line>();
        invalid = new List<Line>();
        if (trajectory.Points.Count < 2 || TryFk(robot, chain) is not { } fk) return;

        var baseF = baseFrame ?? robot.Preset.BaseFrame;
        var tool = toolFrame ?? robot.Preset.ToolFrame;
        var validator = new TrajectoryValidator();
        var opts = validation ?? new TrajectoryValidationOptions();

        Point3d TcpAt(JointState s) => ToPoint(fk.ComputeTcp(s, baseF, tool).Tcp);

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

    private static Mesh? BarrelMesh(Point3d center, Vector3d axis, double radius, double halfLength)
    {
        if (radius <= 0 || halfLength <= 0 || !axis.Unitize()) return null;
        var basePlane = new Plane(center - axis * halfLength, axis);
        return Mesh.CreateFromCylinder(new Cylinder(new Circle(basePlane, radius), halfLength * 2), 16, 1);
    }
}
