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
        RobotModel robot, JointState state, RobotCollisionModel? geometryOverride,
        SerialJointChain? chain = null, BaseFrame? baseFrame = null, ToolFrame? toolFrame = null)
    {
        if (geometryOverride is null) yield break;
        foreach (var mesh in LinkGeometryMeshes(robot, state, geometryOverride, chain, baseFrame, toolFrame))
            yield return mesh;
    }

    public static IEnumerable<Mesh> LinkMeshes(
        RobotModel robot, JointState state, SerialJointChain? chain = null,
        BaseFrame? baseFrame = null, ToolFrame? toolFrame = null)
    {
        foreach (var mesh in LinkMeshes(robot, state, robot.CollisionModel, chain, baseFrame, toolFrame))
            yield return mesh;
    }

    private static IEnumerable<Mesh> LinkGeometryMeshes(
        RobotModel robot,
        JointState state,
        RobotCollisionModel geometry,
        SerialJointChain? chain,
        BaseFrame? baseFrame,
        ToolFrame? toolFrame)
    {
        if (TryFk(robot, chain) is not { } fk) yield break;

        var baseF = baseFrame ?? robot.Preset.BaseFrame;
        var tool = toolFrame ?? robot.Preset.ToolFrame;
        var linkTransforms = fk.ComputeLinkTransforms(state.Positions);
        var baseM = Transforms.FromFrame(baseF.Frame);
        foreach (var link in geometry.Links)
        {
            if (link.LinkIndex < 0 || link.LinkIndex >= linkTransforms.Count) continue;
            if (link.LocalGeometry.Shape != CollisionShape.Mesh) continue;
            var world = TransformCollision(link.LocalGeometry, Transforms.Multiply(baseM, linkTransforms[link.LinkIndex]));
            if (ToRhinoMesh(world) is { } mesh) yield return mesh;
        }

        if (geometry.ToolGeometry is null) yield break;
        var tcp = fk.ComputeTcp(state, baseF, tool).Tcp;
        var toolWorld = TransformCollision(geometry.ToolGeometry, Transforms.FromFrame(tcp));
        if (ToRhinoMesh(toolWorld) is { } toolMesh)
            yield return toolMesh;
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

    private static CollisionObject TransformCollision(CollisionObject local, double[] linkWorldMatrix)
    {
        var worldMatrix = Transforms.Multiply(linkWorldMatrix, Transforms.FromFrame(local.Pose));
        var worldFrame = Transforms.ToFrame(worldMatrix);
        return local.Shape switch
        {
            CollisionShape.Sphere => CollisionObject.Sphere(local.Name, worldFrame, local.ExtentX),
            CollisionShape.Box => CollisionObject.Box(local.Name, worldFrame, local.ExtentX, local.ExtentY, local.ExtentZ),
            CollisionShape.Capsule => CollisionObject.Capsule(local.Name, worldFrame, local.ExtentX, local.ExtentY),
            CollisionShape.Mesh when local.MeshVertices is not null && local.MeshIndices is not null =>
                CollisionObject.Mesh(local.Name, Frame.Identity, TransformVertices(local.MeshVertices, worldMatrix), local.MeshIndices),
            _ => local
        };
    }

    private static List<double[]> TransformVertices(List<double[]> vertices, double[] worldMatrix)
    {
        var result = new List<double[]>(vertices.Count);
        foreach (var v in vertices)
        {
            var p = Transforms.TransformPoint(worldMatrix, v[0], v[1], v[2]);
            result.Add(new[] { p[0], p[1], p[2] });
        }
        return result;
    }

    public static Mesh? CollisionObjectMesh(CollisionObject obj) => ToRhinoMesh(obj);

    public static IEnumerable<Mesh> CollisionSceneMeshes(CollisionScene scene)
    {
        foreach (var obj in scene.Objects)
        {
            if (CollisionObjectMesh(obj) is { } mesh)
                yield return mesh;
        }
    }

    private static Mesh? ToRhinoMesh(CollisionObject obj)
    {
        switch (obj.Shape)
        {
            case CollisionShape.Sphere:
                return Mesh.CreateFromSphere(new Sphere(ToPoint(obj.Pose), obj.ExtentX), 16, 12);
            case CollisionShape.Box:
                var box = new Box(
                    FrameConversion.ToPlane(obj.Pose),
                    new Interval(-obj.ExtentX, obj.ExtentX),
                    new Interval(-obj.ExtentY, obj.ExtentY),
                    new Interval(-obj.ExtentZ, obj.ExtentZ));
                return Mesh.CreateFromBox(box, 1, 1, 1);
            case CollisionShape.Capsule:
                return CapsuleMesh(obj);
            case CollisionShape.Mesh:
                return RawMesh(obj.MeshVertices, obj.MeshIndices);
            default:
                return null;
        }
    }

    private static Mesh? CapsuleMesh(CollisionObject capsule)
    {
        var radius = capsule.ExtentX;
        var halfLength = capsule.ExtentY;
        if (radius <= 0 || halfLength <= 0) return null;
        var plane = FrameConversion.ToPlane(capsule.Pose);
        var axis = plane.ZAxis;
        if (!axis.Unitize()) return null;

        var mesh = new Mesh();
        var lineFrom = plane.Origin - axis * halfLength;
        var lineTo = plane.Origin + axis * halfLength;
        var body = CylinderMesh(lineFrom, lineTo, radius);
        if (body is not null) mesh.Append(body);
        var capA = Mesh.CreateFromSphere(new Sphere(lineFrom, radius), 12, 8);
        var capB = Mesh.CreateFromSphere(new Sphere(lineTo, radius), 12, 8);
        if (capA is not null) mesh.Append(capA);
        if (capB is not null) mesh.Append(capB);
        mesh.Normals.ComputeNormals();
        mesh.Compact();
        return mesh;
    }

    private static Mesh? RawMesh(IReadOnlyList<double[]>? vertices, IReadOnlyList<int>? indices)
    {
        if (vertices is null || indices is null || vertices.Count == 0 || indices.Count < 3) return null;
        var mesh = new Mesh();
        foreach (var v in vertices)
            mesh.Vertices.Add(v[0], v[1], v[2]);
        for (var i = 0; i + 2 < indices.Count; i += 3)
            mesh.Faces.AddFace(indices[i], indices[i + 1], indices[i + 2]);
        mesh.Normals.ComputeNormals();
        mesh.Compact();
        return mesh;
    }
}
