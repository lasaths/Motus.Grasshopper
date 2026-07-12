using Grasshopper.Kernel;
using Motus.OMPL.NET;
using System;
using System.Linq;
using System.Threading;

namespace Motus.GH.Planning;

public readonly record struct RrtPlanSettings(
    int MaxIterations,
    double MaxPlanTimeSeconds,
    SamplingPlannerId PlannerId,
    double GoalBias,
    double StepRadians)
{
    public static RrtPlanSettings Defaults => new(4000, 0, SamplingPlannerId.RrtConnect, 0.08, 0.12);

    public static bool TryBuild(IGH_DataAccess da, out RrtPlanSettings settings, out string? error) =>
        TryRead(da, 0, 1, 2, 3, 4, out settings, out error);

    public static bool TryRead(
        IGH_DataAccess da,
        int maxIterIndex,
        int timeLimitIndex,
        int plannerIndex,
        int goalBiasIndex,
        int stepIndex,
        out RrtPlanSettings settings,
        out string? error)
    {
        settings = Defaults;
        error = null;

        var maxIterations = settings.MaxIterations;
        if (da.GetData(maxIterIndex, ref maxIterations))
        {
            if (maxIterations <= 0)
            {
                error = "RrtMaxIter must be positive.";
                return false;
            }

            settings = settings with { MaxIterations = maxIterations };
        }

        var timeLimit = settings.MaxPlanTimeSeconds;
        if (da.GetData(timeLimitIndex, ref timeLimit))
        {
            if (timeLimit < 0)
            {
                error = "RrtTimeLimit must be zero (no limit) or positive seconds.";
                return false;
            }

            settings = settings with { MaxPlanTimeSeconds = timeLimit };
        }

        var plannerText = "RrtConnect";
        if (da.GetData(plannerIndex, ref plannerText))
        {
            if (!SamplingPlannerRegistry.TryParse(plannerText, out var plannerId))
            {
                error = $"Unknown planner '{plannerText.Trim()}'. Available: {string.Join(", ", SamplingPlannerRegistry.ListAvailable().Select(d => d.ShortName))}.";
                return false;
            }

            var desc = SamplingPlannerRegistry.Resolve(plannerId);
            if (desc is null || (!desc.NativeSupported && !desc.ManagedSupported))
            {
                error = desc?.UnavailableReason ?? $"Planner '{plannerText.Trim()}' is not available in this build.";
                return false;
            }

            settings = settings with { PlannerId = plannerId };
        }

        var goalBias = settings.GoalBias;
        if (da.GetData(goalBiasIndex, ref goalBias))
        {
            if (goalBias is < 0 or > 1)
            {
                error = "RrtGoalBias must be between 0 and 1.";
                return false;
            }

            settings = settings with { GoalBias = goalBias };
        }

        var stepRadians = settings.StepRadians;
        if (da.GetData(stepIndex, ref stepRadians))
        {
            if (stepRadians <= 0)
            {
                error = "RrtStep must be positive (radians).";
                return false;
            }

            settings = settings with { StepRadians = stepRadians };
        }

        return true;
    }

    public SamplingPlannerOptions ToOptions(CancellationToken cancellationToken, Action<double>? goalProgress) => new()
    {
        MaxIterations = MaxIterations,
        MaxPlanTimeSeconds = MaxPlanTimeSeconds,
        PlannerId = PlannerId,
        GoalBias = GoalBias,
        StepRadians = StepRadians,
        RandomSeed = 42,
        ShouldCancel = () => cancellationToken.IsCancellationRequested,
        ReportIteration = goalProgress is null
            ? null
            : (iter, max) => goalProgress((double)iter / max)
    };

    public string PlannerLabel => SamplingPlannerRegistry.Resolve(PlannerId)?.Label ?? PlannerId.ToString();

    /// <summary>Maps unavailable planner IDs to managed RRT-Connect (stub-build compatibility).</summary>
    public RrtPlanSettings CoerceAvailable(out string? fallbackReason)
    {
        var desc = SamplingPlannerRegistry.Resolve(PlannerId);
        if (desc is not null && (desc.NativeSupported || desc.ManagedSupported))
        {
            fallbackReason = null;
            return this;
        }

        fallbackReason = desc?.UnavailableReason ?? $"Planner '{PlannerLabel}' is not available in this build.";
        return this with { PlannerId = SamplingPlannerId.RrtConnect };
    }
}
