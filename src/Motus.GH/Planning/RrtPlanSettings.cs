using Grasshopper.Kernel;
using Motus.OMPL.NET;
using System;
using System.Threading;

namespace Motus.GH.Planning;

public readonly record struct RrtPlanSettings(
    int MaxIterations,
    double MaxPlanTimeSeconds,
    OmplPlannerId PlannerId,
    double GoalBias,
    double StepRadians)
{
    public static RrtPlanSettings Defaults => new(4000, 0, OmplPlannerId.RrtConnect, 0.08, 0.12);

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
            settings = settings with { PlannerId = ParsePlannerId(plannerText) };

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

    public RrtConnectOptions ToOptions(CancellationToken cancellationToken, Action<double>? goalProgress) => new()
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

    public string PlannerLabel => PlannerId == OmplPlannerId.RrtStar ? "RRT*" : "RRT-Connect";

    private static OmplPlannerId ParsePlannerId(string text)
    {
        var normalized = text.Trim();
        if (normalized.Equals("RrtStar", StringComparison.OrdinalIgnoreCase) ||
            normalized.Equals("RRT*", StringComparison.OrdinalIgnoreCase) ||
            normalized.Equals("Star", StringComparison.OrdinalIgnoreCase))
            return OmplPlannerId.RrtStar;

        return OmplPlannerId.RrtConnect;
    }
}
