using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.GH.Loaders;

/// <summary>Default end-effectors and bundled robot assets beside the plugin.</summary>
internal static class BundledToolLoader
{
    public const string Ur10eRobotiqUrdf = "resources/robots/ur10e_robotiq/ur10e_robotiq.urdf";
    private const string RobotiqStl = "resources/tools/robotiq_2f85_tcp_local.stl";
    private static readonly Frame RobotiqTcp = new(0, 0, 0.1633, 0.7071067811865476, 0, 0.7071067811865476, 0);
    private static readonly double[] TcpLocalToTool0Local = ComputeTcpLocalToTool0Local();

    private static double[] ComputeTcpLocalToTool0Local()
    {
        var tcpInFlange = Transforms.FromFrame(RobotiqTcp);
        var tool0InFlange = Transforms.FromRpy(0, 0, 0, Math.PI / 2, 0, Math.PI / 2);
        return Transforms.Multiply(Transforms.Inverse(tool0InFlange), tcpInFlange);
    }

    private static List<double[]> ConvertVerticesToTool0Local(IReadOnlyList<double[]> tcpLocal)
    {
        var result = new List<double[]>(tcpLocal.Count);
        foreach (var v in tcpLocal)
        {
            var p = Transforms.TransformPoint(TcpLocalToTool0Local, v[0], v[1], v[2]);
            result.Add(new[] { p[0], p[1], p[2] });
        }

        return result;
    }

    public static ToolDefinition? TryDefaultForUrdfPath(string urdfPath)
    {
        var file = Path.GetFileNameWithoutExtension(urdfPath);
        if (file.Contains("robotiq", StringComparison.OrdinalIgnoreCase) ||
            file.Equals("ur10e_robotiq", StringComparison.OrdinalIgnoreCase))
            return TryLoadRobotiq(urdfPath);
        return null;
    }

    private static ToolDefinition? TryLoadRobotiq(string? urdfPath = null)
    {
        var resolvedUrdf = ResolveRobotiqUrdfPath(urdfPath);
        foreach (var path in RobotiqMeshCandidates(urdfPath))
        {
            if (TryLoadRobotiqFromPath(path, resolvedUrdf) is { } tool)
                return tool;
        }

        return null;
    }

    private static string? ResolveRobotiqUrdfPath(string? urdfPath)
    {
        if (!string.IsNullOrWhiteSpace(urdfPath) && File.Exists(urdfPath))
            return Path.GetFullPath(urdfPath);

        var bundled = ResolveBundledPath(Ur10eRobotiqUrdf);
        return File.Exists(bundled) ? bundled : urdfPath;
    }

    private static IEnumerable<string> RobotiqMeshCandidates(string? urdfPath)
    {
        yield return ResolveBundledPath(RobotiqStl);

        if (string.IsNullOrWhiteSpace(urdfPath) || !File.Exists(urdfPath))
            yield break;

        var urdfDir = Path.GetDirectoryName(Path.GetFullPath(urdfPath));
        if (urdfDir is null) yield break;

        yield return Path.GetFullPath(Path.Combine(urdfDir, "meshes", "robotiq_2f85", "robotiq_2f85_tcp_local.stl"));
        yield return Path.GetFullPath(Path.Combine(urdfDir, "..", "..", "tools", "robotiq_2f85_tcp_local.stl"));
    }

    private static ToolDefinition? TryLoadRobotiqFromPath(string path, string? urdfPath = null)
    {
        if (!File.Exists(path)) return null;

        var (vertices, indices) = StlReader.Read(path);
        if (vertices.Count < 3 || indices.Count < 3 || !MeshVerticesFinite(vertices))
            return null;

        var tool0Vertices = ConvertVerticesToTool0Local(vertices);
        var geometry = CollisionObject.Mesh("robotiq_2f85", Frame.Identity, tool0Vertices, indices);
        var attachOffset = string.IsNullOrWhiteSpace(urdfPath)
            ? null
            : UrdfFixedChain.TryTipAttachOffset(urdfPath, "base_link", "tool0");
        return new ToolDefinition("robotiq_2f85", RobotiqTcp, geometry, ToolCapabilities.Robotiq2F85)
        {
            GeometryInFlangeFrame = true,
            GeometryAttachOffset = attachOffset
        };
    }

    private static bool MeshVerticesFinite(IReadOnlyList<double[]> vertices)
    {
        foreach (var v in vertices)
        {
            if (v.Length < 3) return false;
            if (!double.IsFinite(v[0]) || !double.IsFinite(v[1]) || !double.IsFinite(v[2]))
                return false;
        }
        return true;
    }

    internal static string? TryLoadRobotiqFailureReason(string? urdfPath = null)
    {
        foreach (var path in RobotiqMeshCandidates(urdfPath))
        {
            if (!File.Exists(path))
                continue;
            var (vertices, indices) = StlReader.Read(path);
            if (vertices.Count < 3 || indices.Count < 3)
                return $"Robotiq STL has no triangles: {path}";
            if (!MeshVerticesFinite(vertices))
                return $"Robotiq STL has invalid (non-finite) vertices. Re-run: node scripts/fetch-ur10e-assets.mjs ({path})";
            return null;
        }
        return "Robotiq gripper mesh not found beside the plugin. Reinstall Motus or rebuild with resources/tools deployed.";
    }

    internal static string ResolveBundledPath(string relativePath)
    {
        if (Path.IsPathRooted(relativePath) && File.Exists(relativePath))
            return Path.GetFullPath(relativePath);

        var normalized = relativePath.Replace('/', Path.DirectorySeparatorChar);
        foreach (var candidate in EnumerateCandidates(normalized))
        {
            if (File.Exists(candidate))
                return candidate;
        }

        var fromCwd = ResolveFromWorkingDirectory(normalized);
        if (fromCwd is not null)
            return fromCwd;

        return EnumerateCandidates(normalized).First();
    }

    private static string? ResolveFromWorkingDirectory(string normalized)
    {
        var dir = Directory.GetCurrentDirectory();
        for (var i = 0; i < 10 && dir is not null; i++)
        {
            var candidate = Path.Combine(dir, normalized);
            if (File.Exists(candidate))
                return Path.GetFullPath(candidate);
            dir = Directory.GetParent(dir)?.FullName;
        }

        return null;
    }

    private static IEnumerable<string> EnumerateCandidates(string normalized)
    {
        foreach (var root in PluginRoots())
        {
            foreach (var rel in RelativePathVariants(normalized))
                yield return Path.Combine(root, rel);

            var dir = root;
            for (var i = 0; i < 8 && dir is not null; i++)
            {
                foreach (var rel in RelativePathVariants(normalized))
                    yield return Path.Combine(dir, rel);
                dir = Directory.GetParent(dir)?.FullName;
            }
        }
    }

    private static IEnumerable<string> RelativePathVariants(string normalized)
    {
        yield return normalized;
        const string prefix = "resources" + "\\";
        if (normalized.StartsWith(prefix, StringComparison.OrdinalIgnoreCase))
            yield return normalized[prefix.Length..];
    }

    // Grasshopper loads .gha from %AppData%/Grasshopper/Libraries/Motus — not Rhino's netcore folder.
    private static IEnumerable<string> PluginRoots()
    {
        var seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        var asmPath = typeof(BundledToolLoader).Assembly.Location;
        if (!string.IsNullOrWhiteSpace(asmPath))
        {
            var asmDir = Path.GetDirectoryName(asmPath);
            if (!string.IsNullOrWhiteSpace(asmDir) && seen.Add(asmDir))
                yield return asmDir;
        }

        var baseDir = AppContext.BaseDirectory;
        if (!string.IsNullOrWhiteSpace(baseDir) && seen.Add(baseDir))
            yield return baseDir;

        var ghMotus = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
            "Grasshopper", "Libraries", "Motus");
        if (Directory.Exists(ghMotus) && seen.Add(ghMotus))
            yield return ghMotus;
    }
}
