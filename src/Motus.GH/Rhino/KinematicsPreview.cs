using Motus.Core;
using Motus.Geometry;
using Rhino.Geometry;
using System.Drawing;

namespace Motus.GH.Rhino;

public static class KinematicsPreview
{
    /// <summary>LinkIndex sentinel: place mesh via TreeFK + LinkName (see UrdfVisualPreviewLoader.TreeLinkIndex).</summary>
    public const int TreeLinkIndex = -2;

    public static IFkSolver? TryFk(RobotModel robot, SerialJointChain? chain = null)
    {
        try
        {
            if (chain is null &&
                string.Equals(robot.Preset.Family, "urdf", StringComparison.OrdinalIgnoreCase))
                return null;
            return KinematicsResolver.CreateFkSolver(robot.Preset, chain);
        }
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
            if (link.LocalGeometry.Shape != CollisionShape.Mesh) continue;
            CollisionObject world;
            if (link.LinkIndex < 0)
                world = TransformCollision(link.LocalGeometry, baseM);
            else
            {
                if (link.LinkIndex >= linkTransforms.Count) continue;
                world = TransformCollision(link.LocalGeometry, Transforms.Multiply(baseM, linkTransforms[link.LinkIndex]));
            }
            if (ToRhinoMesh(world) is { } mesh) yield return mesh;
        }

        if (geometry.ToolGeometry is null) yield break;
        var toolM = ToolCollisionPreview.WorldMatrix(fk, state.Positions, baseF, tool, geometry);
        var toolWorld = TransformCollision(geometry.ToolGeometry, toolM);
        if (ToRhinoMesh(toolWorld) is { } toolMesh)
            yield return toolMesh;
    }

    /// <summary>Cache link-local meshes; per-frame cost is transform only (TreeFK Into when tree present).</summary>
    public sealed class PreviewMeshCache
    {
        private readonly IFkSolver _fk;
        private readonly TreeForwardKinematics? _treeFk;
        private readonly KinematicTree? _tree;
        private readonly double[]? _driverQ;
        private readonly double[][]? _treeMats;
        private readonly int[]? _treeLinkOfMesh; // per _links entry, or -1
        private readonly IReadOnlyList<string>? _armJointNames;
        private readonly BaseFrame _baseF;
        private readonly ToolFrame _toolF;
        private readonly double[] _baseMatrix;
        private readonly List<(int LinkIndex, string LinkName, Mesh Mesh)> _links;
        private readonly Mesh? _toolMesh;
        private readonly CollisionObject? _toolGeometry;
        private readonly bool _toolInFlangeFrame;
        private readonly Frame? _toolAttachOffset;
        private readonly double _toolOpenWidth;
        private readonly IReadOnlyList<Color?> _meshColors;
        private List<Mesh>? _frameMeshes;

        public IReadOnlyList<Color?> MeshColors => _meshColors;

        private PreviewMeshCache(
            IFkSolver fk,
            TreeForwardKinematics? treeFk,
            KinematicTree? tree,
            double[]? driverQ,
            double[][]? treeMats,
            int[]? treeLinkOfMesh,
            IReadOnlyList<string>? armJointNames,
            BaseFrame baseF,
            ToolFrame toolF,
            List<(int, string, Mesh)> links,
            Mesh? toolMesh,
            CollisionObject? toolGeometry,
            bool toolInFlangeFrame,
            Frame? toolAttachOffset,
            double toolOpenWidth,
            IReadOnlyList<Color?> meshColors)
        {
            _fk = fk;
            _treeFk = treeFk;
            _tree = tree;
            _driverQ = driverQ;
            _treeMats = treeMats;
            _treeLinkOfMesh = treeLinkOfMesh;
            _armJointNames = armJointNames;
            _baseF = baseF;
            _toolF = toolF;
            _baseMatrix = Transforms.FromFrame(baseF.Frame);
            _links = links;
            _toolMesh = toolMesh;
            _toolGeometry = toolGeometry;
            _toolInFlangeFrame = toolInFlangeFrame;
            _toolAttachOffset = toolAttachOffset;
            _toolOpenWidth = toolOpenWidth > 1e-9 ? toolOpenWidth : Robotiq2F85Kinematics.OpenWidthMeters;
            _meshColors = meshColors;
        }

        public static PreviewMeshCache? TryCreate(
            RobotModel robot,
            RobotCollisionModel geometry,
            SerialJointChain? chain = null,
            BaseFrame? baseFrame = null,
            ToolFrame? toolFrame = null,
            ToolCapabilities? toolCapabilities = null,
            Color?[]? urdfColors = null,
            KinematicTree? tree = null,
            IReadOnlyList<string>? armJointNames = null)
        {
            if (TryFk(robot, chain) is not { } fk) return null;

            var baseF = baseFrame ?? robot.Preset.BaseFrame;
            var toolF = toolFrame ?? robot.Preset.ToolFrame;
            var links = new List<(int, string, Mesh)>();
            var meshColors = new List<Color?>();

            for (var gi = 0; gi < geometry.Links.Count; gi++)
            {
                var link = geometry.Links[gi];
                if (link.LocalGeometry.Shape != CollisionShape.Mesh) continue;
                var baked = TransformCollision(link.LocalGeometry, Transforms.Identity());
                if (ToRhinoMesh(baked) is { } mesh)
                {
                    links.Add((link.LinkIndex, link.LinkName, mesh));
                    meshColors.Add(urdfColors is not null && gi < urdfColors.Length ? urdfColors[gi] : null);
                }
            }

            Mesh? toolMesh = null;
            if (geometry.ToolGeometry is { Shape: CollisionShape.Mesh })
            {
                var baked = TransformCollision(geometry.ToolGeometry, Transforms.Identity());
                toolMesh = ToRhinoMesh(baked);
            }

            if (toolMesh is not null)
                meshColors.Add(null);

            TreeForwardKinematics? treeFk = null;
            double[]? driverQ = null;
            double[][]? treeMats = null;
            int[]? treeLinkOfMesh = null;
            if (tree is not null)
            {
                treeFk = new TreeForwardKinematics(tree);
                driverQ = new double[tree.DriverCount];
                treeMats = new double[tree.Links.Count][];
                for (var i = 0; i < treeMats.Length; i++)
                    treeMats[i] = new double[16];
                treeLinkOfMesh = new int[links.Count];
                for (var i = 0; i < links.Count; i++)
                {
                    try { treeLinkOfMesh[i] = tree.IndexOfLink(links[i].Item2); }
                    catch { treeLinkOfMesh[i] = -1; }
                }
            }

            return links.Count == 0 && toolMesh is null
                ? null
                : new PreviewMeshCache(
                    fk,
                    treeFk,
                    tree,
                    driverQ,
                    treeMats,
                    treeLinkOfMesh,
                    armJointNames ?? robot.JointNames,
                    baseF,
                    toolF,
                    links,
                    toolMesh,
                    geometry.ToolGeometry,
                    geometry.ToolGeometryInFlangeFrame,
                    geometry.ToolGeometryAttachOffset,
                    toolCapabilities?.Parameters.FirstOrDefault(p =>
                        string.Equals(p.Name, "width", StringComparison.Ordinal))?.Max
                        ?? Robotiq2F85Kinematics.OpenWidthMeters,
                    meshColors);
        }

        public List<Mesh> MeshesFor(JointState state, EndEffectorState? toolState = null) =>
            UpdateMeshes(state, _frameMeshes ??= CreateFrameMeshList(), duplicate: true, toolState);

        public void UpdateMeshes(JointState state, List<Mesh> target, EndEffectorState? toolState = null)
        {
            UpdateMeshes(state, target, duplicate: false, toolState);
        }

        private List<Mesh> CreateFrameMeshList()
        {
            var list = new List<Mesh>(_links.Count + (_toolMesh is null ? 0 : 1));
            foreach (var (_, _, localMesh) in _links)
                list.Add(localMesh.DuplicateMesh());
            if (_toolMesh is not null)
                list.Add(_toolMesh.DuplicateMesh());
            return list;
        }

        private List<Mesh> UpdateMeshes(
            JointState state,
            List<Mesh> target,
            bool duplicate,
            EndEffectorState? toolState = null)
        {
            var linkMats = _fk.ComputeLinkTransforms(state.Positions);
            var meshCount = _links.Count + (_toolMesh is null ? 0 : 1);
            if (!duplicate)
            {
                while (target.Count < meshCount)
                    target.Add(new Mesh());
            }

            var results = duplicate
                ? new List<Mesh>(meshCount)
                : target;

            var jawWidth = _toolOpenWidth;
            if (toolState?.Values.TryGetValue("width", out var width) == true)
                jawWidth = width;

            if (_treeFk is not null && _tree is not null && _driverQ is not null && _treeMats is not null)
                FillTreeDriverQ(state.Positions, jawWidth);

            for (var i = 0; i < _links.Count; i++)
            {
                var (linkIndex, _, localMesh) = _links[i];
                double[] worldM;
                if (_treeFk is not null && _treeMats is not null && _treeLinkOfMesh is not null
                    && _treeLinkOfMesh[i] >= 0)
                {
                    worldM = Transforms.Multiply(_baseMatrix, _treeMats[_treeLinkOfMesh[i]]);
                }
                else if (linkIndex == TreeLinkIndex)
                {
                    // Tree missing: leave at base (should not happen for bundled URDF)
                    worldM = _baseMatrix;
                }
                else
                {
                    worldM = linkIndex < 0
                        ? _baseMatrix
                        : linkIndex < linkMats.Count
                            ? Transforms.Multiply(_baseMatrix, linkMats[linkIndex])
                            : _baseMatrix;
                }

                if (duplicate)
                {
                    var mesh = localMesh.DuplicateMesh();
                    mesh.Transform(ToRhinoTransform(worldM));
                    results.Add(mesh);
                }
                else
                {
                    target[i].CopyFrom(localMesh);
                    target[i].Transform(ToRhinoTransform(worldM));
                }
            }

            if (_toolMesh is not null)
            {
                // Fallback only when no URDF gripper links exist (planning hull viewport).
                var toolM = ToolCollisionPlacement.WorldMatrix(
                    _fk, state.Positions, _baseF, _toolF, _toolGeometry, _toolInFlangeFrame, _toolAttachOffset);
                var toolXform = ToRhinoTransform(toolM);

                if (duplicate)
                {
                    var mesh = _toolMesh.DuplicateMesh();
                    mesh.Transform(toolXform);
                    results.Add(mesh);
                }
                else
                {
                    var toolIndex = _links.Count;
                    target[toolIndex].CopyFrom(_toolMesh);
                    target[toolIndex].Transform(toolXform);
                }
            }
            else if (!duplicate && target.Count > _links.Count)
                target.RemoveRange(_links.Count, target.Count - _links.Count);

            return results;
        }

        private void FillTreeDriverQ(IReadOnlyList<double> armQ, double jawWidthMeters)
        {
            var tree = _tree!;
            var q = _driverQ!;
            var jaw = Robotiq2F85Kinematics.DriverAngleRadians(jawWidthMeters, _toolOpenWidth);
            for (var di = 0; di < tree.DriverCount; di++)
            {
                var j = tree.Joints[tree.DriverJointIndices[di]];
                if (j.Name.Contains("robotiq", StringComparison.OrdinalIgnoreCase)
                    && j.Name.Contains("left_knuckle", StringComparison.OrdinalIgnoreCase)
                    && !j.Name.Contains("finger", StringComparison.OrdinalIgnoreCase)
                    && !j.Name.Contains("inner", StringComparison.OrdinalIgnoreCase))
                {
                    q[di] = jaw;
                    continue;
                }

                var ai = -1;
                if (_armJointNames is not null)
                {
                    for (var k = 0; k < _armJointNames.Count; k++)
                    {
                        if (string.Equals(_armJointNames[k], j.Name, StringComparison.OrdinalIgnoreCase))
                        {
                            ai = k;
                            break;
                        }
                    }
                }

                if (ai < 0)
                    ai = di; // serial tree / same order
                q[di] = ai >= 0 && ai < armQ.Count ? armQ[ai] : 0;
            }

            _treeFk!.ComputeLinkTransformsInto(q, _treeMats!);
        }

        private static Transform ToRhinoTransform(double[] m) => new()
        {
            M00 = m[0], M01 = m[1], M02 = m[2], M03 = m[3],
            M10 = m[4], M11 = m[5], M12 = m[6], M13 = m[7],
            M20 = m[8], M21 = m[9], M22 = m[10], M23 = m[11],
            M30 = m[12], M31 = m[13], M32 = m[14], M33 = m[15],
        };
    }

    public static List<Mesh> LinkMeshesCached(
        PreviewMeshCache cache, JointState state) => cache.MeshesFor(state);

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
            case CollisionShape.Plane:
            {
                // Motus local +X = free normal; ToPlane recovers Rhino Z = Motus X.
                var pl = FrameConversion.ToPlane(obj.Pose);
                var slab = new Box(
                    pl,
                    new Interval(-1, 1),
                    new Interval(-1, 1),
                    new Interval(-0.002, 0));
                return Mesh.CreateFromBox(slab, 1, 1, 1);
            }
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
