using Grasshopper.Kernel;
using Motus.Core;
using Motus.GH.Data;
using Motus.Geometry;
using Motus.Presets;
using System.Globalization;
using System.Xml.Linq;

namespace Motus.GH.Components;

public sealed class MotusLoadUrdfComponent : MotusComponentBase
{
    public MotusLoadUrdfComponent() : base("Motus Load URDF", "URDF", "Load a serial-chain URDF into a robot model", "Model", "file") { }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddTextParameter("Path", "P", "Path to .urdf file", GH_ParamAccess.item);
        p.AddTextParameter("BaseLink", "B", "Base link name", GH_ParamAccess.item, "base_link");
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("TipLink", "T", "Tip link name", GH_ParamAccess.item, "tool0");
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddGenericParameter("Robot", "Rb", "Robot model with URDF kinematics chain", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var path = "";
        var baseLink = "base_link";
        var tipLink = "tool0";
        if (!da.GetData(0, ref path) || string.IsNullOrWhiteSpace(path)) return;
        da.GetData(1, ref baseLink);
        da.GetData(2, ref tipLink);

        try
        {
            path = UrdfPathResolver.ResolveUrdfPath(path);
            var urdf = UrdfRobotLoader.Load(path, new UrdfLoadOptions
            {
                BaseLink = baseLink,
                TipLink = tipLink,
                ModelName = Path.GetFileNameWithoutExtension(path)
            });
            var previewGeometry = UrdfVisualPreviewLoader.TryLoad(path, baseLink, tipLink);
            var goo = RobotModelGoo.FromUrdf(urdf, previewGeometry);
            if (BundledToolLoader.TryDefaultForUrdfPath(path) is { } bundledTool)
            {
                goo.Tool = bundledTool;
                if (previewGeometry?.Links.Count > 0)
                {
                    var armLinks = previewGeometry.Links
                        .Where(l => !l.LinkName.Contains("robotiq", StringComparison.OrdinalIgnoreCase))
                        .ToList();
                    goo.PreviewGeometry = new RobotCollisionModel(armLinks);
                }
            }
            da.SetData(0, goo);
        }
        catch (Exception ex)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
        }
    }

    public override Guid ComponentGuid => new Guid("c8e4a1b2-3f5d-4e6a-9b7c-1d2e3f4a5b6c");
}

internal static class UrdfPathResolver
{
    public static string ResolveUrdfPath(string path)
    {
        if (string.IsNullOrWhiteSpace(path)) return path;
        if (File.Exists(path)) return Path.GetFullPath(path);
        var normalized = path.Replace('/', Path.DirectorySeparatorChar);
        var dir = Directory.GetCurrentDirectory();
        for (var i = 0; i < 10 && dir is not null; i++)
        {
            var candidate = Path.Combine(dir, normalized);
            if (File.Exists(candidate)) return candidate;
            dir = Directory.GetParent(dir)?.FullName;
        }
        return path;
    }
}

internal static class UrdfVisualPreviewLoader
{
    public static RobotCollisionModel? TryLoad(string urdfPath, string baseLink, string tipLink)
    {
        var doc = XDocument.Load(urdfPath);
        return TryLoad(doc.Root, Path.GetDirectoryName(Path.GetFullPath(urdfPath)) ?? ".", baseLink, tipLink);
    }

    private static RobotCollisionModel? TryLoad(XElement? robotRoot, string urdfDirectory, string baseLink, string tipLink)
    {
        if (robotRoot is null) return null;
        var chainLinkNames = BuildActuatedChainLinkNames(robotRoot, baseLink, tipLink);
        if (chainLinkNames.Count == 0) return null;

        var linksByName = robotRoot.Elements("link")
            .ToDictionary(l => l.Attribute("name")?.Value ?? "", l => l, StringComparer.OrdinalIgnoreCase);

        var geometries = new List<LinkCollisionGeometry>();
        for (var i = 0; i < chainLinkNames.Count; i++)
            AppendLinkVisuals(linksByName, chainLinkNames[i], i, urdfDirectory, geometries);

        AppendFixedDescendantVisuals(
            robotRoot, linksByName, tipLink, chainLinkNames.Count - 1,
            ComposeFixedForwardChain(robotRoot, chainLinkNames[^1], tipLink),
            urdfDirectory, geometries);

        return geometries.Count > 0 ? new RobotCollisionModel(geometries) : null;
    }

    private static CollisionObject? ParseVisualGeometry(string name, Frame pose, XElement geom, string urdfDirectory)
    {
        if (geom.Element("box") is { } box)
        {
            var size = ParseTriple(box.Attribute("size")?.Value, 0.1, 0.1, 0.1);
            return CollisionObject.Box(name, pose, size.x / 2, size.y / 2, size.z / 2);
        }
        if (geom.Element("cylinder") is { } cyl)
        {
            var radius = ParseDouble(cyl.Attribute("radius")?.Value, 0.05);
            var length = ParseDouble(cyl.Attribute("length")?.Value, 0.1);
            return CollisionObject.Capsule(name, pose, radius, length / 2);
        }
        if (geom.Element("sphere") is { } sph)
        {
            var radius = ParseDouble(sph.Attribute("radius")?.Value, 0.05);
            return CollisionObject.Sphere(name, pose, radius);
        }
        if (geom.Element("mesh") is { } mesh)
        {
            var filename = mesh.Attribute("filename")?.Value;
            if (string.IsNullOrWhiteSpace(filename)) return null;
            var scale = ParseTriple(mesh.Attribute("scale")?.Value, 1, 1, 1);
            if (Math.Abs(scale.x - scale.y) > 1e-9 || Math.Abs(scale.y - scale.z) > 1e-9)
                return null; // ponytail: skip non-uniform mesh scales for now.
            var path = ResolveMeshPath(filename, urdfDirectory);
            if (!File.Exists(path))
                return null;
            var (vertices, indices) =
                path.EndsWith(".stl", StringComparison.OrdinalIgnoreCase)
                    ? MeshFileLoader.ReadStlBytes(path, scale.x)
                    : path.EndsWith(".dae", StringComparison.OrdinalIgnoreCase)
                        ? ReadDae(path, scale.x)
                        : (new List<double[]>(), new List<int>());
            if (vertices.Count == 0 || indices.Count < 3)
                return null;
            return CollisionObject.Mesh(name, pose, vertices, indices);
        }
        return null;
    }

    private static (List<double[]> vertices, List<int> indices) ReadDae(string path, double uniformScale)
    {
        var doc = XDocument.Load(path);
        var root = doc.Root;
        if (root is null) return (new List<double[]>(), new List<int>());
        var ns = root.Name.Namespace;

        var vertices = new List<double[]>();
        var indices = new List<int>();

        foreach (var geom in root.Descendants(ns + "geometry"))
        {
            var mesh = geom.Element(ns + "mesh");
            if (mesh is null) continue;

            var localVertices = ParseDaePositions(mesh, ns, uniformScale);
            if (localVertices.Count == 0) continue;

            var vertMap = BuildVerticesMap(mesh, ns);
            foreach (var tri in mesh.Elements(ns + "triangles"))
                AppendDaeTriangles(tri, ns, localVertices, vertMap, vertices, indices);
            foreach (var poly in mesh.Elements(ns + "polylist"))
                AppendDaePolylist(poly, ns, localVertices, vertMap, vertices, indices);
        }

        return (vertices, indices);
    }

    private static Dictionary<string, List<double[]>> ParseDaePositions(XElement mesh, XNamespace ns, double scale)
    {
        var result = new Dictionary<string, List<double[]>>(StringComparer.OrdinalIgnoreCase);
        foreach (var source in mesh.Elements(ns + "source"))
        {
            var sourceId = source.Attribute("id")?.Value;
            if (string.IsNullOrWhiteSpace(sourceId)) continue;
            var floatArray = source.Element(ns + "float_array");
            if (floatArray is null) continue;

            var accessor = source.Element(ns + "technique_common")?.Element(ns + "accessor");
            var stride = ParseInt(accessor?.Attribute("stride")?.Value, 3);
            if (stride < 3) stride = 3;

            var vals = ParseDoubles(floatArray.Value);
            var points = new List<double[]>(vals.Count / stride);
            for (var i = 0; i + 2 < vals.Count; i += stride)
                points.Add(new[] { vals[i] * scale, vals[i + 1] * scale, vals[i + 2] * scale });

            result[sourceId] = points;
        }
        return result;
    }

    private static Dictionary<string, string> BuildVerticesMap(XElement mesh, XNamespace ns)
    {
        var map = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        foreach (var vertices in mesh.Elements(ns + "vertices"))
        {
            var vertId = vertices.Attribute("id")?.Value;
            if (string.IsNullOrWhiteSpace(vertId)) continue;
            var input = vertices.Elements(ns + "input")
                .FirstOrDefault(i => string.Equals(i.Attribute("semantic")?.Value, "POSITION", StringComparison.OrdinalIgnoreCase));
            var src = NormalizeRef(input?.Attribute("source")?.Value);
            if (!string.IsNullOrWhiteSpace(src))
                map[vertId] = src;
        }
        return map;
    }

    private static void AppendDaeTriangles(
        XElement tri,
        XNamespace ns,
        Dictionary<string, List<double[]>> localVertices,
        Dictionary<string, string> vertMap,
        List<double[]> outVertices,
        List<int> outIndices)
    {
        var inputs = tri.Elements(ns + "input").ToList();
        if (inputs.Count == 0) return;
        var stride = inputs.Max(i => ParseInt(i.Attribute("offset")?.Value, 0)) + 1;
        var vertexInput = inputs.FirstOrDefault(i => string.Equals(i.Attribute("semantic")?.Value, "VERTEX", StringComparison.OrdinalIgnoreCase));
        if (vertexInput is null) return;
        var vertexOffset = ParseInt(vertexInput.Attribute("offset")?.Value, 0);
        var sourceKey = NormalizeRef(vertexInput.Attribute("source")?.Value);
        if (sourceKey is null) return;
        if (vertMap.TryGetValue(sourceKey, out var mapped)) sourceKey = mapped;
        if (!localVertices.TryGetValue(sourceKey, out var points)) return;

        var p = tri.Element(ns + "p");
        if (p is null) return;
        var tokens = ParseInts(p.Value);
        for (var i = 0; i + (stride * 3 - 1) < tokens.Count; i += stride * 3)
        {
            for (var v = 0; v < 3; v++)
            {
                var idx = tokens[i + v * stride + vertexOffset];
                if (idx < 0 || idx >= points.Count) return;
                outVertices.Add(points[idx]);
                outIndices.Add(outVertices.Count - 1);
            }
        }
    }

    private static void AppendDaePolylist(
        XElement poly,
        XNamespace ns,
        Dictionary<string, List<double[]>> localVertices,
        Dictionary<string, string> vertMap,
        List<double[]> outVertices,
        List<int> outIndices)
    {
        var inputs = poly.Elements(ns + "input").ToList();
        if (inputs.Count == 0) return;
        var stride = inputs.Max(i => ParseInt(i.Attribute("offset")?.Value, 0)) + 1;
        var vertexInput = inputs.FirstOrDefault(i => string.Equals(i.Attribute("semantic")?.Value, "VERTEX", StringComparison.OrdinalIgnoreCase));
        if (vertexInput is null) return;
        var vertexOffset = ParseInt(vertexInput.Attribute("offset")?.Value, 0);
        var sourceKey = NormalizeRef(vertexInput.Attribute("source")?.Value);
        if (sourceKey is null) return;
        if (vertMap.TryGetValue(sourceKey, out var mapped)) sourceKey = mapped;
        if (!localVertices.TryGetValue(sourceKey, out var points)) return;

        var vcount = ParseInts(poly.Element(ns + "vcount")?.Value ?? "");
        var p = ParseInts(poly.Element(ns + "p")?.Value ?? "");
        var cursor = 0;
        foreach (var n in vcount)
        {
            if (n < 3) { cursor += n * stride; continue; }
            if (cursor + n * stride > p.Count) return;
            var firstIdx = p[cursor + vertexOffset];
            for (var t = 1; t < n - 1; t++)
            {
                var i1 = p[cursor + t * stride + vertexOffset];
                var i2 = p[cursor + (t + 1) * stride + vertexOffset];
                if (firstIdx < 0 || i1 < 0 || i2 < 0 ||
                    firstIdx >= points.Count || i1 >= points.Count || i2 >= points.Count) return;
                outVertices.Add(points[firstIdx]); outIndices.Add(outVertices.Count - 1);
                outVertices.Add(points[i1]); outIndices.Add(outVertices.Count - 1);
                outVertices.Add(points[i2]); outIndices.Add(outVertices.Count - 1);
            }
            cursor += n * stride;
        }
    }

    private static string ResolveMeshPath(string filename, string urdfDirectory)
    {
        var cleaned = filename.Replace("package://", "").Replace("file://", "");
        var path = Path.IsPathRooted(cleaned)
            ? Path.GetFullPath(cleaned)
            : Path.GetFullPath(Path.Combine(urdfDirectory, cleaned));
        if (!File.Exists(path))
            path = Path.GetFullPath(Path.Combine(urdfDirectory, Path.GetFileName(cleaned)));
        return path;
    }

    private static string? NormalizeRef(string? source)
    {
        if (string.IsNullOrWhiteSpace(source)) return null;
        return source.StartsWith("#", StringComparison.Ordinal) ? source[1..] : source;
    }

    private static void AppendLinkVisuals(
        Dictionary<string, XElement> linksByName,
        string linkName,
        int linkIndex,
        string urdfDirectory,
        List<LinkCollisionGeometry> geometries)
    {
        if (!linksByName.TryGetValue(linkName, out var linkEl)) return;
        var visualIdx = 0;
        foreach (var visual in linkEl.Elements("visual"))
        {
            var origin = visual.Element("origin");
            var xyz = ParseTriple(origin?.Attribute("xyz")?.Value);
            var rpy = ParseTriple(origin?.Attribute("rpy")?.Value);
            var pose = FrameFromRpy(xyz.x, xyz.y, xyz.z, rpy.x, rpy.y, rpy.z);
            var geom = visual.Element("geometry");
            if (geom is null) continue;
            var name = $"{linkName}_vis{visualIdx++}";
            var obj = ParseVisualGeometry(name, pose, geom, urdfDirectory);
            if (obj is not null)
                geometries.Add(new LinkCollisionGeometry(linkIndex, linkName, obj));
        }
    }

    private static void AppendFixedDescendantVisuals(
        XElement robotRoot,
        Dictionary<string, XElement> linksByName,
        string parentLink,
        int attachLinkIndex,
        Frame parentToWorld,
        string urdfDirectory,
        List<LinkCollisionGeometry> geometries)
    {
        foreach (var joint in robotRoot.Elements("joint"))
        {
            if (!string.Equals(joint.Attribute("type")?.Value, "fixed", StringComparison.OrdinalIgnoreCase))
                continue;
            if (!string.Equals(joint.Element("parent")?.Attribute("link")?.Value, parentLink, StringComparison.OrdinalIgnoreCase))
                continue;

            var child = joint.Element("child")?.Attribute("link")?.Value ?? "";
            if (string.IsNullOrWhiteSpace(child)) continue;

            var origin = joint.Element("origin");
            var xyz = ParseTriple(origin?.Attribute("xyz")?.Value);
            var rpy = ParseTriple(origin?.Attribute("rpy")?.Value);
            var jointPose = FrameFromRpy(xyz.x, xyz.y, xyz.z, rpy.x, rpy.y, rpy.z);
            var childToWorld = ComposeFrames(parentToWorld, jointPose);

            if (linksByName.TryGetValue(child, out var linkEl))
            {
                var visualIdx = 0;
                foreach (var visual in linkEl.Elements("visual"))
                {
                    var visOrigin = visual.Element("origin");
                    var visXyz = ParseTriple(visOrigin?.Attribute("xyz")?.Value);
                    var visRpy = ParseTriple(visOrigin?.Attribute("rpy")?.Value);
                    var visPose = FrameFromRpy(visXyz.x, visXyz.y, visXyz.z, visRpy.x, visRpy.y, visRpy.z);
                    var geom = visual.Element("geometry");
                    if (geom is null) continue;
                    var name = $"{child}_vis{visualIdx++}";
                    var obj = ParseVisualGeometry(name, ComposeFrames(childToWorld, visPose), geom, urdfDirectory);
                    if (obj is not null)
                        geometries.Add(new LinkCollisionGeometry(attachLinkIndex, child, obj));
                }
            }

            AppendFixedDescendantVisuals(robotRoot, linksByName, child, attachLinkIndex, childToWorld, urdfDirectory, geometries);
        }
    }

    private static Frame ComposeFrames(Frame parent, Frame local) =>
        Transforms.ToFrame(Transforms.Multiply(Transforms.FromFrame(parent), Transforms.FromFrame(local)));

    private static Frame ComposeFixedForwardChain(XElement robotRoot, string fromLink, string toLink)
    {
        if (string.Equals(fromLink, toLink, StringComparison.OrdinalIgnoreCase))
            return new Frame(0, 0, 0, 1, 0, 0, 0);

        var joints = robotRoot.Elements("joint")
            .Where(j => string.Equals(j.Attribute("type")?.Value, "fixed", StringComparison.OrdinalIgnoreCase))
            .Select(j => new
            {
                Parent = j.Element("parent")?.Attribute("link")?.Value ?? "",
                Child = j.Element("child")?.Attribute("link")?.Value ?? "",
                Origin = j.Element("origin")
            })
            .Where(j => !string.IsNullOrWhiteSpace(j.Parent) && !string.IsNullOrWhiteSpace(j.Child))
            .GroupBy(j => j.Parent, StringComparer.OrdinalIgnoreCase)
            .ToDictionary(g => g.Key, g => g.ToList(), StringComparer.OrdinalIgnoreCase);

        var queue = new Queue<(string Link, Frame Pose)>();
        queue.Enqueue((fromLink, new Frame(0, 0, 0, 1, 0, 0, 0)));
        var visited = new HashSet<string>(StringComparer.OrdinalIgnoreCase) { fromLink };

        while (queue.Count > 0)
        {
            var (link, pose) = queue.Dequeue();
            if (string.Equals(link, toLink, StringComparison.OrdinalIgnoreCase))
                return pose;
            if (!joints.TryGetValue(link, out var children)) continue;
            foreach (var joint in children)
            {
                if (!visited.Add(joint.Child)) continue;
                var xyz = ParseTriple(joint.Origin?.Attribute("xyz")?.Value);
                var rpy = ParseTriple(joint.Origin?.Attribute("rpy")?.Value);
                var step = FrameFromRpy(xyz.x, xyz.y, xyz.z, rpy.x, rpy.y, rpy.z);
                queue.Enqueue((joint.Child, ComposeFrames(pose, step)));
            }
        }

        return new Frame(0, 0, 0, 1, 0, 0, 0);
    }

    private static List<string> BuildActuatedChainLinkNames(XElement robotRoot, string baseLink, string tipLink)
    {
        var joints = robotRoot.Elements("joint")
            .Select(j => new
            {
                Type = (j.Attribute("type")?.Value ?? "fixed").Trim(),
                Parent = (j.Element("parent")?.Attribute("link")?.Value ?? "").Trim(),
                Child = (j.Element("child")?.Attribute("link")?.Value ?? "").Trim()
            })
            .Where(j => !string.IsNullOrWhiteSpace(j.Parent) && !string.IsNullOrWhiteSpace(j.Child))
            .ToList();

        var byChild = joints.ToDictionary(j => j.Child, j => j, StringComparer.OrdinalIgnoreCase);
        var path = new List<string>();
        var link = tipLink;
        var guard = 0;
        while (!string.Equals(link, baseLink, StringComparison.OrdinalIgnoreCase))
        {
            if (++guard > 128) return new List<string>();
            if (!byChild.TryGetValue(link, out var joint)) return new List<string>();
            if (!string.Equals(joint.Type, "fixed", StringComparison.OrdinalIgnoreCase))
                path.Add(joint.Child);
            link = joint.Parent;
        }
        path.Reverse();
        return path;
    }

    private static Frame FrameFromRpy(double x, double y, double z, double roll, double pitch, double yaw) =>
        Transforms.ToFrame(Transforms.FromRpy(x, y, z, roll, pitch, yaw));

    private static (double x, double y, double z) ParseTriple(string? s, double dx = 0, double dy = 0, double dz = 0)
    {
        if (string.IsNullOrWhiteSpace(s)) return (dx, dy, dz);
        var p = s.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
        if (p.Length < 3) return (dx, dy, dz);
        return (ParseDouble(p[0], dx), ParseDouble(p[1], dy), ParseDouble(p[2], dz));
    }

    private static double ParseDouble(string? s, double fallback) =>
        double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var v) ? v : fallback;

    private static int ParseInt(string? s, int fallback) =>
        int.TryParse(s, NumberStyles.Integer, CultureInfo.InvariantCulture, out var v) ? v : fallback;

    private static List<double> ParseDoubles(string raw)
    {
        var parts = raw.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
        var vals = new List<double>(parts.Length);
        foreach (var p in parts)
            if (double.TryParse(p, NumberStyles.Float, CultureInfo.InvariantCulture, out var v))
                vals.Add(v);
        return vals;
    }

    private static List<int> ParseInts(string raw)
    {
        var parts = raw.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
        var vals = new List<int>(parts.Length);
        foreach (var p in parts)
            if (int.TryParse(p, NumberStyles.Integer, CultureInfo.InvariantCulture, out var v))
                vals.Add(v);
        return vals;
    }
}
