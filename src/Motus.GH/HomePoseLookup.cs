using System.Text.Json;
using Motus.Core;

namespace Motus.GH;

internal static class HomePoseLookup
{
    private static Dictionary<string, Dictionary<string, double>>? _cache;

    private static Dictionary<string, Dictionary<string, double>> Load()
    {
        if (_cache is not null) return _cache;
        var path = Path.Combine(AppContext.BaseDirectory, "resources", "viewer_presets.json");
        if (!File.Exists(path))
            return _cache = new Dictionary<string, Dictionary<string, double>>(StringComparer.OrdinalIgnoreCase);

        var json = File.ReadAllText(path);
        var root = JsonSerializer.Deserialize<Dictionary<string, JsonElement>>(json)
            ?? new Dictionary<string, JsonElement>();
        var map = new Dictionary<string, Dictionary<string, double>>(StringComparer.OrdinalIgnoreCase);
        foreach (var (key, el) in root)
        {
            if (!el.TryGetProperty("defaultPose", out var poseEl)) continue;
            var pose = JsonSerializer.Deserialize<Dictionary<string, double>>(poseEl.GetRawText());
            if (pose is not null) map[key] = pose;
        }
        return _cache = map;
    }

    private static string? PresetKey(RobotModel robot) =>
        robot.Preset.ModelName.ToLowerInvariant() switch
        {
            "ur5e" => "ur5e_collision",
            "ur10e" => "ur10e",
            _ => robot.Preset.ModelName.Replace(' ', '_').ToLowerInvariant()
        };

    public static JointState HomeOrZeros(RobotModel robot)
    {
        var presets = Load();
        var key = PresetKey(robot);
        if (key is null || !presets.TryGetValue(key, out var named))
            return new JointState(new double[robot.Preset.AxisCount]);

        var names = robot.JointNames;
        if (names is not null && names.Count == robot.Preset.AxisCount)
        {
            var q = new double[names.Count];
            for (var i = 0; i < names.Count; i++)
                q[i] = named.GetValueOrDefault(names[i], 0);
            return new JointState(q);
        }

        return new JointState(new double[robot.Preset.AxisCount]);
    }
}
