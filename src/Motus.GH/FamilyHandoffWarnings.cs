using Motus.Core;

namespace Motus.GH;

/// <summary>
/// Family handoff honesty for Motus Waypoints / Export — shared with qa-smoke (Rhino-free).
/// Gate on <see cref="RobotPreset.Family"/>, not bare <c>AxisCount == 6</c>.
/// </summary>
public static class FamilyHandoffWarnings
{
    public const string StewartWaypoints =
        "Stewart Family=stewart: Q values are leg lengths in meters — do not wire to UR MoveJ (radians).";

    public const string AerialWaypoints =
        "Family=aerial: HolonomicSE3 body poses (m + RPY rad) — Q is not UR MoveJ radians. Prefer Motus Export bodyPose / Preview body scrub.";

    public const string StewartExport =
        "Family=stewart export: joint coordinates are leg lengths in meters — do not wire CSV/JSON Q values to UR MoveJ (radians).";

    public const string AerialExport =
        "Family=aerial export: bodyPose SE(3) (m + RPY rad) — not jointsRadians / UR MoveJ. Motus.NET JSON/CSV marks waypointsQ=not_ur_movej.";

    public const string UrdfZeroAxis =
        "Family=urdf AxisCount=0: Q is empty — not UR MoveJ. Experimental free-flyer / meshless load; prefer Motus.NET HolonomicSE3 export (Family=aerial) when authored.";

    public static string LeggedFullDriverWaypoints(int axisCount) =>
        $"Family=legged full-driver gait: Q has {axisCount} joint(s) in radians (PlanBodyPath/Walk) — not UR MoveJ.";

    public static string LeggedTipPathWaypoints(int axisCount, int treeDrivers) =>
        $"Family=legged tip-path: Q has {axisCount} joint(s) in radians (one leg) — not full mechanism ({treeDrivers} drivers). Do not wire to UR MoveJ.";

    public static string LeggedGenericWaypoints() =>
        "Family=legged: Q values are joint angles in radians (not Stewart meters) — do not wire full-driver gait to UR MoveJ.";

    public static string TipPathSerialWaypoints(int axisCount, int treeDrivers) =>
        $"Tip-path robot: Q has {axisCount} joint(s) per waypoint (one serial chain) — not full mechanism ({treeDrivers} tree drivers). Do not wire to UR MoveJ for the whole robot.";

    public static string NonSixAxisUrController(int axisCount) =>
        $"Robot has {axisCount} axes; many UR controllers expect 6 joint values per waypoint.";

    public static string LeggedTipPathExport(int axisCount, int treeDrivers) =>
        $"Family=legged tip-path export: Q has {axisCount} joint(s) in radians (one leg) — not full mechanism ({treeDrivers} drivers). Do not wire to UR MoveJ.";

    public static string LeggedGenericExport() =>
        "Family=legged export: Q values are joint angles in radians — not Stewart meters and not a UR MoveJ handoff for the whole mechanism.";

    /// <summary>
    /// Waypoints Status warnings (at most one primary family message).
    /// </summary>
    public static IReadOnlyList<string> ForWaypoints(
        RobotPreset preset,
        int treeDrivers,
        bool tipPathOnly,
        bool hasStewartContext,
        bool hasMechanism,
        int? mechanismDriverCount)
    {
        var axisCount = preset.AxisCount;
        var stewart = Units.IsStewart(preset) || hasStewartContext;
        var legged = Units.IsLegged(preset);
        var aerial = string.Equals(preset.Family, "aerial", StringComparison.OrdinalIgnoreCase);

        if (stewart)
            return [StewartWaypoints];

        if (legged)
        {
            if (hasMechanism && !tipPathOnly && axisCount == (mechanismDriverCount ?? axisCount))
                return [LeggedFullDriverWaypoints(axisCount)];
            if (tipPathOnly && treeDrivers > axisCount)
                return [LeggedTipPathWaypoints(axisCount, treeDrivers)];
            return [LeggedGenericWaypoints()];
        }

        if (aerial)
            return [AerialWaypoints];

        if (string.Equals(preset.Family, "urdf", StringComparison.OrdinalIgnoreCase) && axisCount == 0)
            return [UrdfZeroAxis];

        if (tipPathOnly && treeDrivers > axisCount)
            return [TipPathSerialWaypoints(axisCount, treeDrivers)];

        if (axisCount != 6)
            return [NonSixAxisUrController(axisCount)];

        return [];
    }

    /// <summary>
    /// Export Status warnings (at most one primary family message).
    /// </summary>
    public static IReadOnlyList<string> ForExport(
        RobotPreset preset,
        int treeDrivers,
        bool tipPathOnly,
        bool hasStewartContext)
    {
        var axisCount = preset.AxisCount;
        var stewart = Units.IsStewart(preset) || hasStewartContext;
        var legged = Units.IsLegged(preset);
        var aerial = string.Equals(preset.Family, "aerial", StringComparison.OrdinalIgnoreCase);

        if (stewart)
            return [StewartExport];

        if (legged)
        {
            if (tipPathOnly && treeDrivers > axisCount)
                return [LeggedTipPathExport(axisCount, treeDrivers)];
            return [LeggedGenericExport()];
        }

        if (aerial)
            return [AerialExport];

        if (string.Equals(preset.Family, "urdf", StringComparison.OrdinalIgnoreCase) && axisCount == 0)
            return [UrdfZeroAxis];

        return [];
    }
}
