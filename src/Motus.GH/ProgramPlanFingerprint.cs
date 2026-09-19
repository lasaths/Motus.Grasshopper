using Motus.Core;

namespace Motus.GH;

/// <summary>
/// Motus Program Auto Plan input fingerprint — includes Seg, Start, Collision, Group, Attach, Robot, Tool, Prior.
/// Rhino-free for qa-smoke.
/// </summary>
public static class ProgramPlanFingerprint
{
    public static string Compute(
        RobotModel model,
        ToolDefinition? tool,
        IReadOnlyList<MotionSegment> segments,
        JointState start,
        CollisionScene? scene,
        PlanningGroup? group,
        IReadOnlyList<AttachedBody> attached,
        string? priorEndFp)
    {
        var parts = new List<string>
        {
            "model=" + model.Preset.ModelName + ":axes=" + model.Preset.AxisCount,
            "tool=" + (tool?.Name ?? "null"),
            "prior=" + (priorEndFp ?? "null")
        };
        parts.AddRange(segments.Select(s => SegmentFingerprint(s) + ":contacts=" +
                string.Join(";", s.AllowedCollisionPairs.Select(pair => $"{pair.A},{pair.B}"))));
        parts.Add(string.Join(",", start.Positions.Select(q => q.ToString("R"))));
        parts.Add(scene is null
            ? "scene=null"
            : "scene=" + string.Join(";", scene.Objects.Select(o => $"{o.Name}:{o.ContentHash}:{FrameFp(o.Pose)}"))
                + "#pairs=" + string.Join(";", scene.AllowedPairs.Select(p => $"{p.A},{p.B}")));
        if (group is not null)
        {
            parts.Add("group=" + group.Name + ":" + group.BaseLink + ":" + group.TipLink + ":" +
                      string.Join(",", group.JointNames));
        }
        else
        {
            parts.Add("group=null");
        }

        var attachOrdered = attached.OrderBy(a => a.Name, StringComparer.Ordinal).ToList();
        parts.Add("attach=" + attachOrdered.Count + ":" + string.Join(";", attachOrdered.Select(a =>
            $"{a.Name}:{a.Geometry.ContentHash}:{FrameFp(a.TcpLocalPose)}")));
        return string.Join("|", parts);
    }

    public static string PriorEndFingerprint(Trajectory? prior)
    {
        if (prior is not { Points.Count: > 0 })
            return "null";
        var end = prior.Points[^1];
        var tool = end.ToolState is null
            ? "tool=null"
            : "tool=" + string.Join(",", end.ToolState.Values.Select(kv => $"{kv.Key}={kv.Value:R}"));
        return "q=" + string.Join(",", end.JointState.Positions.Select(q => q.ToString("R"))) + ";" + tool;
    }

    private static string FrameFp(Frame f) =>
        $"{f.X:R},{f.Y:R},{f.Z:R},{f.Qw:R},{f.Qx:R},{f.Qy:R},{f.Qz:R}";

    private static string SegmentFingerprint(MotionSegment s) => s switch
    {
        TransferSegment transfer => $"TRANSFER:{FrameFp(transfer.Goal.Tcp)}",
        LinSegment lin => $"LIN:{lin.StepMeters:R}:{lin.BlendRadiusMeters:R}:{FrameFp(lin.Goal.Tcp)}",
        CircSegment circ => $"CIRC:{circ.ArcSamples}:{circ.BlendRadiusMeters:R}:{FrameFp(circ.Via.Tcp)}:{FrameFp(circ.Goal.Tcp)}",
        PtpSegment ptp => $"PTP:{ptp.BlendRadiusMeters:R}:{string.Join(",", ptp.Goal.Positions.Select(q => q.ToString("R")))}",
        SetToolStateSegment set => $"SET:{set.DurationSeconds:R}:{string.Join(",", set.State.Values.Select(kv => $"{kv.Key}={kv.Value:R}"))}",
        WaitSegment w => $"WAIT:{w.DurationSeconds:R}",
        AttachSegment a => $"ATTACH:{a.Name}:{FrameFp(a.TcpLocal)}",
        DetachSegment d => $"DETACH:{d.Name}:{FrameFp(d.WorldPose)}",
        _ => s.GetType().Name
    };
}
