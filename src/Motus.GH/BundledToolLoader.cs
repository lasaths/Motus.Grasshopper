using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.GH;

/// <summary>Default end-effectors shipped beside the plugin (resources/tools/).</summary>
internal static class BundledToolLoader
{
    private const string RobotiqStl = "resources/tools/robotiq_2f85_tcp_local.stl";
    private static readonly Frame RobotiqTcp = new(0, 0, 0.1633, 0.7071067811865476, 0, 0.7071067811865476, 0);

    public static ToolDefinition? TryDefaultForModel(string modelName)
    {
        try
        {
            var model = PresetLoader.LoadRobotModelByName(modelName);
            return ToolDefinition.FromPreset(model);
        }
        catch (FileNotFoundException)
        {
            return null;
        }
    }

    public static ToolDefinition? TryDefaultForUrdfPath(string urdfPath)
    {
        var file = Path.GetFileNameWithoutExtension(urdfPath);
        if (file.Contains("robotiq", StringComparison.OrdinalIgnoreCase) ||
            file.Equals("ur10e_robotiq", StringComparison.OrdinalIgnoreCase))
            return TryLoadRobotiq();
        return null;
    }

    private static ToolDefinition? TryLoadRobotiq()
    {
        var path = ResolveBundledPath(RobotiqStl);
        if (!File.Exists(path)) return null;

        var (vertices, indices) = StlReader.Read(path);
        if (vertices.Count < 3 || indices.Count < 3) return null;

        var geometry = CollisionObject.Mesh("robotiq_2f85", Frame.Identity, vertices, indices);
        return new ToolDefinition("robotiq_2f85", RobotiqTcp, geometry);
    }

    internal static string ResolveBundledPath(string relativePath)
    {
        if (Path.IsPathRooted(relativePath) && File.Exists(relativePath))
            return Path.GetFullPath(relativePath);

        var normalized = relativePath.Replace('/', Path.DirectorySeparatorChar);
        var dir = AppContext.BaseDirectory;
        for (var i = 0; i < 6 && dir is not null; i++)
        {
            var candidate = Path.Combine(dir, normalized);
            if (File.Exists(candidate)) return candidate;
            dir = Directory.GetParent(dir)?.FullName;
        }

        return Path.Combine(AppContext.BaseDirectory, normalized);
    }
}
