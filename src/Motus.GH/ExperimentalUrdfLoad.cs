namespace Motus.GH;

/// <summary>
/// Experimental Motus Robot URDF loads (free-flyer / H2 / Go2 fixtures) — Preview/TreeFK only.
/// Rhino-free helper for Motus Robot + qa-smoke.
/// </summary>
public static class ExperimentalUrdfLoad
{
    public const string ExperimentalRemark =
        "Experimental URDF load (Family=urdf) — TreeFK / Preview scrub only; not Walk, not biped balance, not HolonomicSE3 flight GA.";

    public const string ZeroAxisAddOn =
        "AxisCount=0 meshless hull — not UR MoveJ; HolonomicSE3 / Family=aerial planning is Motus.NET tip API, not a GH Aerial component.";

    /// <summary>
    /// True when path or model name looks like an experimental fixture (free-flyer / Unitree H2 / Go2).
    /// </summary>
    public static bool IsExperimentalPathOrModel(string? pathOrModel)
    {
        if (string.IsNullOrWhiteSpace(pathOrModel))
            return false;
        var s = pathOrModel.Replace('\\', '/');
        return ContainsIgnoreCase(s, "free_flyer")
               || ContainsIgnoreCase(s, "unitree_h2")
               || ContainsIgnoreCase(s, "h2_minimal")
               || ContainsIgnoreCase(s, "unitree_go2")
               || ContainsIgnoreCase(s, "go2_minimal");
    }

    /// <summary>
    /// Collect Remark strings for Motus Robot after a successful URDF load.
    /// </summary>
    public static IReadOnlyList<string> RemarksFor(
        string? path,
        string? family,
        string? modelName,
        int axisCount)
    {
        var list = new List<string>();
        var experimental = IsExperimentalPathOrModel(path)
                           || IsExperimentalPathOrModel(modelName)
                           || (string.Equals(family, "urdf", StringComparison.OrdinalIgnoreCase) && axisCount == 0);
        if (!experimental)
            return list;

        list.Add(ExperimentalRemark);
        if (axisCount == 0)
            list.Add(ZeroAxisAddOn);
        return list;
    }

    private static bool ContainsIgnoreCase(string haystack, string needle) =>
        haystack.Contains(needle, StringComparison.OrdinalIgnoreCase);
}
