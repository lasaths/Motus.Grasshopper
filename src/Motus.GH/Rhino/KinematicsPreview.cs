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
            if (Units.IsStewart(robot.Preset))
                return null;
            if (chain is null &&
                string.Equals(robot.Preset.Family, "urdf", StringComparison.OrdinalIgnoreCase))
                return null;
            return KinematicsResolver.CreateFkSolver(robot.Preset, chain);
        }
        catch (InvalidOperationException) { return null; }
    }

    /// <summary>Stewart leg + platform outline wires at the FK pose for <paramref name="lengths"/>.</summary>
    public static IEnumerable<Line> StewartLegLines(
        StewartPlatform platform,
        JointState lengths,
        CartesianPose? seedPose = null)
    {
        if (!TryStewartPose(platform, lengths, seedPose, out var m, out _))
            yield break;

        for (var i = 0; i < StewartPlatform.LegCount; i++)
        {
            var b = platform.BaseAnchors[i];
            var p = PlatformWorld(m, platform.PlatformAnchors[i]);
            yield return new Line(new Point3d(b.X, b.Y, b.Z), p);
        }

        for (var i = 0; i < StewartPlatform.LegCount; i++)
        {
            var a = PlatformWorld(m, platform.PlatformAnchors[i]);
            var b = PlatformWorld(m, platform.PlatformAnchors[(i + 1) % StewartPlatform.LegCount]);
            yield return new Line(a, b);
        }

        for (var i = 0; i < StewartPlatform.LegCount; i++)
        {
            var a = platform.BaseAnchors[i];
            var b = platform.BaseAnchors[(i + 1) % StewartPlatform.LegCount];
            yield return new Line(new Point3d(a.X, a.Y, a.Z), new Point3d(b.X, b.Y, b.Z));
        }
    }

    /// <summary>
    /// Filled Stewart preview inspired by hex-viz UIs: base plate, platform body, orange legs,
    /// joint balls, COG. Still prismatic Stewart — not a walking coxa/femur/tibia hexapod.
    /// </summary>
    public static StewartPreviewGeometry StewartPreview(
        StewartPlatform platform,
        JointState lengths,
        CartesianPose? seedPose = null)
    {
        var meshes = new List<Mesh>();
        var colors = new List<Color>();
        var wires = StewartLegLines(platform, lengths, seedPose).ToList();
        if (!TryStewartPose(platform, lengths, seedPose, out var m, out var tcp))
            return new StewartPreviewGeometry(meshes, colors, wires);

        var basePts = platform.BaseAnchors.Select(a => new Point3d(a.X, a.Y, a.Z)).ToArray();
        var platPts = platform.PlatformAnchors.Select(a => PlatformWorld(m, a)).ToArray();

        // Support / base plate (ground footprint).
        if (PolygonSlab(basePts, thickness: 0.012, upward: true) is { } baseMesh)
        {
            meshes.Add(baseMesh);
            colors.Add(Color.FromArgb(160, 90, 100, 110));
        }

        // Platform body (pink hex plate — matches common hex-viz body cue).
        if (PolygonSlab(platPts, thickness: 0.018, upward: true) is { } platMesh)
        {
            meshes.Add(platMesh);
            colors.Add(Color.FromArgb(200, 255, 105, 180));
        }

        // Prismatic legs + joint spheres.
        const double legR = 0.018;
        const double jointR = 0.028;
        var legColor = Color.FromArgb(220, 255, 140, 40);
        var jointColor = Color.FromArgb(230, 255, 160, 70);
        for (var i = 0; i < StewartPlatform.LegCount; i++)
        {
            var b = new Point3d(platform.BaseAnchors[i].X, platform.BaseAnchors[i].Y, platform.BaseAnchors[i].Z);
            var p = platPts[i];
            if (CylinderMesh(b, p, legR) is { } leg)
            {
                meshes.Add(leg);
                colors.Add(legColor);
            }
            if (Mesh.CreateFromSphere(new Sphere(b, jointR), 10, 8) is { } jb)
            {
                meshes.Add(jb);
                colors.Add(jointColor);
            }
            if (Mesh.CreateFromSphere(new Sphere(p, jointR), 10, 8) is { } jp)
            {
                meshes.Add(jp);
                colors.Add(jointColor);
            }
        }

        // COG / TCP marker at platform origin.
        var cog = new Point3d(tcp.X, tcp.Y, tcp.Z);
        if (Mesh.CreateFromSphere(new Sphere(cog, 0.035), 12, 10) is { } cogMesh)
        {
            meshes.Add(cogMesh);
            colors.Add(Color.FromArgb(230, 0, 200, 100));
        }

        return new StewartPreviewGeometry(meshes, colors, wires);
    }

    public readonly record struct StewartPreviewGeometry(
        IReadOnlyList<Mesh> Meshes,
        IReadOnlyList<Color> Colors,
        IReadOnlyList<Line> Wires);

    private static bool TryStewartPose(
        StewartPlatform platform,
        JointState lengths,
        CartesianPose? seedPose,
        out double[] matrix,
        out Frame tcp)
    {
        matrix = [];
        tcp = default;
        var fk = new StewartForwardKinematics(platform).TrySolve(lengths, seedPose);
        if (!fk.Success || fk.Pose is null)
            return false;
        tcp = fk.Pose.Tcp;
        matrix = Transforms.FromFrame(tcp);
        return true;
    }

    private static Point3d PlatformWorld(double[] m, Vec3 local)
    {
        Transforms.TransformPointInto(m, local.X, local.Y, local.Z, out var wx, out var wy, out var wz);
        return new Point3d(wx, wy, wz);
    }

    private static Mesh? PolygonSlab(Point3d[] ring, double thickness, bool upward)
    {
        if (ring.Length < 3) return null;
        var poly = new Polyline(ring.Concat([ring[0]]));
        if (!poly.IsValid) return null;
        var mesh = Mesh.CreateFromClosedPolyline(poly);
        if (mesh is null || !mesh.IsValid)
        {
            // Convex fan about centroid if CreateFromClosedPolyline fails on non-planar-ish rings.
            var c = Point3d.Origin;
            foreach (var p in ring) c += p;
            c /= ring.Length;
            mesh = new Mesh();
            mesh.Vertices.Add(c);
            foreach (var p in ring) mesh.Vertices.Add(p);
            for (var i = 0; i < ring.Length; i++)
            {
                var a = i + 1;
                var b = i + 2 <= ring.Length ? i + 2 : 1;
                mesh.Faces.AddFace(0, a, b);
            }
            mesh.Normals.ComputeNormals();
        }

        var n = Vector3d.ZAxis;
        if (mesh.Faces.Count > 0)
        {
            var f = mesh.Faces[0];
            var a = mesh.Vertices[f.A];
            var b = mesh.Vertices[f.B];
            var c = mesh.Vertices[f.C];
            n = Vector3d.CrossProduct(b - a, c - a);
            if (!n.Unitize()) n = Vector3d.ZAxis;
            if (upward && n.Z < 0) n = -n;
        }

        var top = mesh.DuplicateMesh();
        top.Transform(Transform.Translation(n * (thickness * 0.5)));
        var bottom = mesh.DuplicateMesh();
        bottom.Transform(Transform.Translation(-n * (thickness * 0.5)));
        bottom.Flip(true, true, true);

        var slab = new Mesh();
        slab.Append(top);
        slab.Append(bottom);
        var vCount = top.Vertices.Count;
        // Side walls only for the outer ring (skip centroid fan vertex 0 when present).
        var ringStart = top.Vertices.Count == ring.Length + 1 ? 1 : 0;
        var ringCount = top.Vertices.Count - ringStart;
        for (var i = 0; i < ringCount; i++)
        {
            var i0 = ringStart + i;
            var i1 = ringStart + (i + 1) % ringCount;
            slab.Faces.AddFace(i0, i1, vCount + i1, vCount + i0);
        }
        slab.Normals.ComputeNormals();
        slab.Compact();
        return slab.IsValid ? slab : null;
    }

    public static Plane TcpPlane(
        RobotModel robot, JointState state, SerialJointChain? chain = null,
        BaseFrame? baseFrame = null, ToolFrame? toolFrame = null,
        StewartPlatform? stewart = null)
    {
        var baseF = baseFrame ?? robot.Preset.BaseFrame;
        if (stewart is not null || Units.IsStewart(robot.Preset))
        {
            if (stewart is null)
                return FrameConversion.ToPlane(baseF.Frame);
            var fk = new StewartForwardKinematics(stewart).TrySolve(state);
            if (!fk.Success || fk.Pose is null)
                return FrameConversion.ToPlanePlate(baseF.Frame);
            return FrameConversion.ToPlanePlate(fk.Pose.Tcp);
        }
        if (TryFk(robot, chain) is not { } serialFk)
            return FrameConversion.ToPlane(baseF.Frame);
        var tool = toolFrame ?? robot.Preset.ToolFrame;
        return FrameConversion.ToPlane(serialFk.ComputeTcp(state, baseF, tool).Tcp);
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
        private readonly ToolCapabilities? _toolCapabilities;
        private readonly IReadOnlyList<ToolDriverBinding>? _toolBindings;
        private readonly string[]? _driverNames;
        private readonly IReadOnlyList<Color?> _meshColors;
        private readonly double[]? _treeDriverHome;
        private List<Mesh>? _frameMeshes;
        private Dictionary<string, double>? _toolStateScratch;

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
            ToolCapabilities? toolCapabilities,
            IReadOnlyList<ToolDriverBinding>? toolBindings,
            string[]? driverNames,
            IReadOnlyList<Color?> meshColors,
            double[]? treeDriverHome)
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
            _toolCapabilities = toolCapabilities;
            _toolBindings = toolBindings;
            _driverNames = driverNames;
            _meshColors = meshColors;
            _treeDriverHome = treeDriverHome;
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
            IReadOnlyList<string>? armJointNames = null,
            IReadOnlyList<ToolDriverBinding>? toolBindings = null,
            JointState? treeDriverHome = null)
        {
            if (TryFk(robot, chain) is not { } fk) return null;

            var baseF = baseFrame ?? robot.Preset.BaseFrame;
            var toolF = toolFrame ?? robot.Preset.ToolFrame;
            var links = new List<(int, string, Mesh)>();
            var meshColors = new List<Color?>();

            for (var gi = 0; gi < geometry.Links.Count; gi++)
            {
                var link = geometry.Links[gi];
                // Tessellate Mesh and primitives (box/sphere/capsule) — mechanism tools often use authored boxes.
                var baked = TransformCollision(link.LocalGeometry, Transforms.Identity());
                if (ToRhinoMesh(baked) is { } mesh)
                {
                    links.Add((link.LinkIndex, link.LinkName, mesh));
                    meshColors.Add(urdfColors is not null && gi < urdfColors.Length ? urdfColors[gi] : null);
                }
            }

            Mesh? toolMesh = null;
            if (geometry.ToolGeometry is not null)
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
            string[]? driverNames = null;
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

                driverNames = new string[tree.DriverCount];
                for (var di = 0; di < tree.DriverCount; di++)
                    driverNames[di] = tree.Joints[tree.DriverJointIndices[di]].Name;
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
                    toolCapabilities,
                    toolBindings,
                    driverNames,
                    meshColors,
                    treeDriverHome?.Positions.ToArray());
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
            var fillErr = TryFillTreeDriverQ(tree, armQ, _armJointNames, _treeDriverHome, q.AsSpan());
            if (fillErr is not null)
                throw new InvalidOperationException(fillErr);

            // Wave 2/3: Motus.NET ToolParameterBinding owns width→driver (mimic owns fingers).
            if ((_toolCapabilities is not null || _toolBindings is { Count: > 0 }) && _driverNames is not null)
            {
                var scratch = _toolStateScratch ??= new Dictionary<string, double>(1);
                scratch["width"] = jawWidthMeters;
                ToolParameterBinding.ApplyInto(
                    _toolCapabilities,
                    new EndEffectorState(scratch),
                    _driverNames,
                    q.AsSpan(),
                    _toolBindings,
                    _toolOpenWidth);
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

    /// <summary>
    /// Fill driver-index-ordered q for TreeFK. Unmatched drivers use <paramref name="treeDriverHome"/>
    /// only when its length equals <see cref="KinematicTree.DriverCount"/> (driver-index order).
    /// </summary>
    public static string? TryFillTreeDriverQ(
        KinematicTree tree,
        IReadOnlyList<double> armQ,
        IReadOnlyList<string>? armJointNames,
        IReadOnlyList<double>? treeDriverHome,
        Span<double> driverQ)
    {
        if (driverQ.Length != tree.DriverCount)
            return $"TreeFK driver buffer length ({driverQ.Length}) != tree driver count ({tree.DriverCount}).";

        if (treeDriverHome is not null && treeDriverHome.Count != tree.DriverCount)
        {
            return $"TreeDriverHome length ({treeDriverHome.Count}) must equal tree driver count ({tree.DriverCount}; driver-index order).";
        }

        var homeComplete = treeDriverHome is not null && treeDriverHome.Count == tree.DriverCount;
        for (var di = 0; di < tree.DriverCount; di++)
        {
            var j = tree.Joints[tree.DriverJointIndices[di]];
            var ai = -1;
            if (armJointNames is not null)
            {
                for (var k = 0; k < armJointNames.Count; k++)
                {
                    if (string.Equals(armJointNames[k], j.Name, StringComparison.OrdinalIgnoreCase))
                    {
                        ai = k;
                        break;
                    }
                }
            }

            if (ai < 0)
                ai = di < armQ.Count ? di : -1;

            if (ai >= 0 && ai < armQ.Count)
                driverQ[di] = armQ[ai];
            else if (homeComplete)
                driverQ[di] = treeDriverHome![di];
            else
            {
                return tree.DriverCount > armQ.Count
                    ? $"TreeFK preview: trajectory has {armQ.Count} joint(s) but tree has {tree.DriverCount} drivers; set TreeDriverHome ({tree.DriverCount} values, driver-index order) on the robot source."
                    : $"TreeFK preview: no value for driver '{j.Name}' (index {di}); provide joint in trajectory or complete TreeDriverHome.";
            }
        }

        return null;
    }

    public static List<Mesh> LinkMeshesCached(
        PreviewMeshCache cache, JointState state) => cache.MeshesFor(state);

    public static Polyline TcpPath(
        RobotModel robot, IEnumerable<JointState> states, SerialJointChain? chain = null,
        BaseFrame? baseFrame = null, ToolFrame? toolFrame = null,
        StewartPlatform? stewart = null)
    {
        if (stewart is not null || Units.IsStewart(robot.Preset))
        {
            if (stewart is null) return new Polyline();
            var fk = new StewartForwardKinematics(stewart);
            var pts = new List<Point3d>();
            foreach (var s in states)
            {
                var r = fk.TrySolve(s);
                if (r.Success && r.Pose is not null)
                    pts.Add(ToPoint(r.Pose.Tcp));
            }
            return pts.Count < 2 ? new Polyline() : new Polyline(pts);
        }

        if (TryFk(robot, chain) is not { } serialFk) return new Polyline();

        var baseF = baseFrame ?? robot.Preset.BaseFrame;
        var tool = toolFrame ?? robot.Preset.ToolFrame;
        var serialPts = states.Select(s => ToPoint(serialFk.ComputeTcp(s, baseF, tool).Tcp)).ToList();
        return serialPts.Count < 2 ? new Polyline() : new Polyline(serialPts);
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
