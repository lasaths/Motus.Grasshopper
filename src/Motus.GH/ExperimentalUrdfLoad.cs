using Motus.Core;

namespace Motus.GH;

/// <summary>
/// Experimental Motus Robot URDF loads (free-flyer / H2 / Go2 fixtures) — Preview/TreeFK only.
/// Rhino-free helper for Motus Robot + qa-smoke.
/// </summary>
public static class ExperimentalUrdfLoad
{
    public const string ExperimentalRemark =
        "Experimental URDF load — free-flyer / H2 / Go2 fixtures. Motus-honest offline planning/preview only; not PX4 / Walk / biped balance.";

    public const string ZeroAxisAddOn =
        "AxisCount=0 meshless hull — not UR MoveJ; plane goals on Motus Plan use HolonomicSE3 (experimental, Motus 2.1).";

    public const string AerialPromotedRemark =
        "Family promoted to aerial for free-flyer URDF — HolonomicSE3 Plan/Export bodyPose; not a flight controller.";

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

    public static bool IsFreeFlyerPathOrModel(string? pathOrModel)
    {
        if (string.IsNullOrWhiteSpace(pathOrModel))
            return false;
        return ContainsIgnoreCase(pathOrModel.Replace('\\', '/'), "free_flyer");
    }

    /// <summary>
    /// Free-flyer meshless URDFs load as Family=urdf; promote to <see cref="Units.AerialFamily"/>
    /// so Plan/Export use HolonomicSE3 honesty (Motus 2.1 ADR 0006).
    /// </summary>
    public static RobotModel PromoteFreeFlyerFamily(RobotModel model, string? path)
    {
        if (model.Preset.AxisCount != 0)
            return model;
        if (!IsFreeFlyerPathOrModel(path) && !IsFreeFlyerPathOrModel(model.Preset.ModelName))
            return model;
        if (Units.IsAerial(model.Preset))
            return model;

        var p = model.Preset;
        var aerial = new RobotPreset
        {
            Manufacturer = p.Manufacturer,
            ModelName = p.ModelName,
            Family = Units.AerialFamily,
            AxisCount = p.AxisCount,
            JointLimits = p.JointLimits,
            ReachMeters = p.ReachMeters,
            PayloadKg = p.PayloadKg,
            BaseFrame = p.BaseFrame,
            ToolFrame = p.ToolFrame,
            Notes = p.Notes,
            SourceNote = p.SourceNote,
            Disclaimer = p.Disclaimer
        };
        return new RobotModel(aerial, model.CollisionModel, model.JointNames);
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
        var freeFlyer = IsFreeFlyerPathOrModel(path) || IsFreeFlyerPathOrModel(modelName);
        var experimental = IsExperimentalPathOrModel(path)
                           || IsExperimentalPathOrModel(modelName)
                           || (string.Equals(family, "urdf", StringComparison.OrdinalIgnoreCase) && axisCount == 0)
                           || (string.Equals(family, Units.AerialFamily, StringComparison.OrdinalIgnoreCase) && freeFlyer);
        if (!experimental)
            return list;

        list.Add(ExperimentalRemark);
        if (freeFlyer || string.Equals(family, Units.AerialFamily, StringComparison.OrdinalIgnoreCase))
            list.Add(AerialPromotedRemark);
        if (axisCount == 0)
            list.Add(ZeroAxisAddOn);
        return list;
    }

    private static bool ContainsIgnoreCase(string haystack, string needle) =>
        haystack.Contains(needle, StringComparison.OrdinalIgnoreCase);
}
