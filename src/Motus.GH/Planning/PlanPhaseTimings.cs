using System.Text;

namespace Motus.GH.Planning;

/// <summary>Lightweight phase timings for planner profiling (ms).</summary>
internal sealed class PlanPhaseTimings
{
    public long CheckerBuildMs { get; set; }
    public long PreflightMs { get; set; }
    public long PlannerMs { get; set; }
    public long CommitMs { get; set; }
    public int GoalCount { get; set; }

    public string FormatSummary()
    {
        var sb = new StringBuilder(96);
        sb.Append("Plan timing (ms): checker=").Append(CheckerBuildMs)
            .Append(" preflight=").Append(PreflightMs)
            .Append(" planner=").Append(PlannerMs);
        if (CommitMs > 0)
            sb.Append(" commit=").Append(CommitMs);
        if (GoalCount > 1)
            sb.Append(" goals=").Append(GoalCount);
        return sb.ToString();
    }
}
