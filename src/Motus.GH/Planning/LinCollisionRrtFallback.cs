using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.NET;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Motus.GH.Planning;

/// <summary>
/// Motus Plan plane-goal path: when TCP-LIN dies on collision, IK the goal and RRT in joint space.
/// Kept free of Rhino/GH deps so qa-smoke can exercise the same contract.
/// </summary>
internal static class LinCollisionRrtFallback
{
    public const string Warning =
        "TCP-LIN blocked by collision; used RRT joint path (not a straight TCP line).";

    public static PlanningResult Plan(
        RobotModel session,
        SerialJointChain? chain,
        JointState start,
        CartesianPose goal,
        PlanningContext planningContext,
        ICollisionChecker checker,
        SamplingPlannerOptions options,
        IEnumerable<string>? linErrors = null)
    {
        var reach = new CartesianGoalSolver().TryReach(
            session,
            goal,
            CartesianGoalSolver.EnumerateDefaultSeeds(start, session),
            chain);
        if (!reach.Success)
        {
            var ikErrors = (linErrors ?? Array.Empty<string>())
                .Concat(reach.Errors)
                .Concat(new[]
                {
                    "TCP-LIN blocked by collision; goal IK failed — cannot RRT to Cartesian goal."
                });
            return PlanningResult.Failed(ikErrors.ToArray());
        }

        var req = new PlanningRequest(
            session,
            start,
            reach.Solution!,
            planningContext.ToPlanningOptions(new PlanningOptions
            {
                CollisionScene = planningContext.Scene,
                CollisionChecker = checker
            }));

        var rrt = SamplingPlanner.Create(checker, options).Plan(req);
        if (!rrt.Success)
        {
            var failErrors = (linErrors ?? Array.Empty<string>())
                .Concat(rrt.Errors)
                .DefaultIfEmpty("TCP-LIN blocked by collision; RRT fallback failed.")
                .ToArray();
            return PlanningResult.Failed(failErrors);
        }

        var warnings = rrt.Warnings.ToList();
        warnings.Add(Warning);
        return PlanningResult.Succeeded(rrt.Trajectory!, warnings);
    }
}
