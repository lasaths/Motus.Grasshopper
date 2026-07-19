using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.NET;
using Motus.GH.Rhino;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;

namespace Motus.GH.Planning;

internal sealed record PlanRequest(
    RobotContext Context,
    IReadOnlyList<(JointState? joints, Plane? plane)> Goals,
    JointState Start,
    PlanningContext PlanningContext,
    double LinStepMeters,
    bool CollisionInputWired,
    RrtPlanSettings RrtSettings);

internal sealed class PlanExecutionResult
{
    public List<PlanningResult> Results { get; init; } = [];
    public Trajectory? ChainedTrajectory { get; init; }
    public List<Trajectory> SegmentTrajectories { get; init; } = [];
    public bool Cancelled { get; init; }
}

internal static class PlanExecutor
{
    private const double MaxJointStep = 0.05;

    public static PlanExecutionResult Execute(
        PlanRequest request,
        CancellationToken cancellationToken,
        Action<double>? reportProgress = null,
        PlanPhaseTimings? timings = null)
    {
        if (cancellationToken.IsCancellationRequested)
            return new PlanExecutionResult { Cancelled = true };

        var results = new List<PlanningResult>(request.Goals.Count);
        var segmentTrajectories = new List<Trajectory>();
        var session = request.Context.EffectiveModel;
        var currentStart = request.Start;
        Trajectory? chained = null;
        var goalCount = Math.Max(1, request.Goals.Count);
        if (timings is not null)
            timings.GoalCount = request.Goals.Count;

        var needsCollision = PlanningCollision.SceneHasObstacles(request.PlanningContext.Scene)
            || request.PlanningContext.Attached.Count > 0;
        ICollisionChecker? sharedChecker = null;
        if (needsCollision)
        {
            var checkerSw = Stopwatch.StartNew();
            sharedChecker = GhExtract.TryCollisionChecker(
                session,
                request.Context.Chain,
                request.PlanningContext.Scene,
                request.PlanningContext.Attached);
            if (timings is not null)
                timings.CheckerBuildMs = checkerSw.ElapsedMilliseconds;
        }

        for (var goalIndex = 0; goalIndex < request.Goals.Count; goalIndex++)
        {
            if (cancellationToken.IsCancellationRequested)
                return new PlanExecutionResult { Cancelled = true, Results = results };

            var spanStart = (double)goalIndex / goalCount;
            var spanSize = 1.0 / goalCount;
            reportProgress?.Invoke(spanStart);

            Action<double>? goalProgress = reportProgress is null
                ? null
                : sub => reportProgress(spanStart + sub * spanSize);

            var goal = request.Goals[goalIndex];

            var preflightSw = Stopwatch.StartNew();
            var preflight = GhExtract.TryPreflightCollision(
                request.Context,
                request.PlanningContext,
                currentStart,
                goal,
                sharedChecker);
            if (timings is not null)
                timings.PreflightMs += preflightSw.ElapsedMilliseconds;

            PlanningResult result;
            var plannerSw = Stopwatch.StartNew();
            if (preflight is not null)
            {
                result = preflight;
            }
            else
            {
                result = goal.plane is { } plane
                    ? PlanCartesianLin(request, currentStart, plane, cancellationToken, goalProgress, sharedChecker)
                    : needsCollision
                        ? PlanRrt(request, currentStart, goal.joints!, cancellationToken, goalProgress, sharedChecker)
                        : new JointLinearPlanner().Plan(new PlanningRequest(
                            session,
                            currentStart,
                            goal.joints!,
                            request.PlanningContext.ToPlanningOptions(new PlanningOptions { MaxJointStepRadians = MaxJointStep })));
            }
            if (timings is not null)
                timings.PlannerMs += plannerSw.ElapsedMilliseconds;

            results.Add(result);
            reportProgress?.Invoke(spanStart + spanSize);

            if (result.Success && result.Trajectory is not null)
            {
                chained = AppendTrajectory(chained, result.Trajectory, session);
                currentStart = result.Trajectory.Points[^1].JointState;
            }
        }

        if (cancellationToken.IsCancellationRequested)
            return new PlanExecutionResult { Cancelled = true, Results = results };

        if (chained is null && results.Any(r => r.Success))
        {
            foreach (var result in results)
            {
                if (result.Success && result.Trajectory is not null)
                    segmentTrajectories.Add(result.Trajectory);
            }
        }

        reportProgress?.Invoke(1.0);

        return new PlanExecutionResult
        {
            Results = results,
            ChainedTrajectory = chained,
            SegmentTrajectories = segmentTrajectories
        };
    }

    private static Trajectory? AppendTrajectory(Trajectory? acc, Trajectory segment, RobotModel robot)
    {
        if (segment.Points.Count == 0) return acc;
        if (acc is null)
            return new Trajectory(robot, segment.Points);

        // Mutate via new list sized for growth — avoid O(N²) ToList() on every goal append.
        var points = new List<TrajectoryPoint>(acc.Points.Count + segment.Points.Count);
        points.AddRange(acc.Points);
        var timeOffset = points[^1].TimeSeconds;
        for (var i = 1; i < segment.Points.Count; i++)
        {
            var pt = segment.Points[i];
            points.Add(new TrajectoryPoint(timeOffset + pt.TimeSeconds, pt.JointState));
        }

        return new Trajectory(robot, points);
    }

    private static PlanningResult PlanCartesianLin(
        PlanRequest request,
        JointState start,
        Plane plane,
        CancellationToken cancellationToken,
        Action<double>? goalProgress,
        ICollisionChecker? sharedChecker)
    {
        goalProgress?.Invoke(0.1);

        var ctx = request.Context;
        var planningContext = request.PlanningContext;
        var session = ctx.EffectiveModel;
        var goal = new CartesianPose(FrameConversion.FromPlane(plane));
        if (!KinematicsResolver.SupportsModel(session.Preset, ctx.Chain))
        {
            return PlanningResult.Failed(new[]
            {
                $"No kinematics profile for '{session.Preset.ModelName}'."
            });
        }

        if (cancellationToken.IsCancellationRequested)
            return PlanningResult.Failed(new[] { "Planning cancelled." });

        goalProgress?.Invoke(0.25);

        // Wave 2: N-DOF / rail plane goals use numerical IK (not UR 6R analytic).
        var ndofNote = session.Preset.AxisCount != 6
            ? $"Plane goal on {session.Preset.AxisCount}-axis robot uses numerical IK (not UR analytic)."
            : null;

        var needsCollision = PlanningCollision.SceneHasObstacles(planningContext.Scene) || planningContext.Attached.Count > 0;
        ICollisionChecker? checker = needsCollision ? sharedChecker : null;
        var opts = planningContext.ToPlanningOptions(new PlanningOptions
        {
            MaxJointStepRadians = MaxJointStep,
            CollisionChecker = checker,
            CollisionScene = planningContext.Scene
        });

        var linOptions = new CartesianLinOptions(StepMeters: request.LinStepMeters, ContinueOnIkFailure: false);
        var linRequest = new CartesianPlanningRequest(session, start, goal, opts, planningContext.Scene);
        var linResult = new CartesianLinearPathPlanner(session.Preset, ctx.Chain).PlanToResult(linRequest, linOptions);
        if (linResult.Success)
        {
            goalProgress?.Invoke(1.0);
            var linWarnings = linResult.Warnings.ToList();
            if (ndofNote is not null) linWarnings.Add(ndofNote);
            if (!request.CollisionInputWired)
                linWarnings.Add("Collision input unwired — plane goal planned in free space (LIN only).");
            else if (needsCollision)
                linWarnings.Add("Collision validated on link envelopes; TCP line may still intersect obstacles that do not hit link geometry.");
            return PlanningResult.Succeeded(linResult.Trajectory!, linWarnings);
        }

        if (linResult.Errors.Any(e => e.Contains("Collision", StringComparison.OrdinalIgnoreCase)))
        {
            if (!needsCollision || sharedChecker is null)
                return linResult;

            if (cancellationToken.IsCancellationRequested)
                return PlanningResult.Failed(new[] { "Planning cancelled." });

            goalProgress?.Invoke(0.5);
            var rrtOpts = request.RrtSettings.ToOptions(cancellationToken, goalProgress);
            var rrtFallback = LinCollisionRrtFallback.Plan(
                session,
                ctx.Chain,
                start,
                goal,
                planningContext,
                sharedChecker,
                rrtOpts,
                linResult.Errors);
            if (rrtFallback.Success)
            {
                goalProgress?.Invoke(1.0);
                if (ndofNote is not null && !rrtFallback.Warnings.Contains(ndofNote))
                {
                    var w = rrtFallback.Warnings.ToList();
                    w.Add(ndofNote);
                    return PlanningResult.Succeeded(rrtFallback.Trajectory!, w);
                }
            }
            return rrtFallback;
        }

        if (cancellationToken.IsCancellationRequested)
            return PlanningResult.Failed(new[] { "Planning cancelled." });

        goalProgress?.Invoke(0.5);

        var reach = new CartesianGoalSolver().TryReach(
            session,
            goal,
            CartesianGoalSolver.EnumerateDefaultSeeds(start, session),
            ctx.Chain);
        if (!reach.Success)
        {
            return PlanningResult.Failed(reach.Errors.Concat(new[]
            {
                "TCP-LIN failed at intermediate poses. For large moves use a Joint State goal or wire Start near the target."
            }).ToArray());
        }

        var goalJoints = reach.Solution!;
        var jointResult = new JointLinearPlanner().Plan(new PlanningRequest(session, start, goalJoints, opts));
        if (!jointResult.Success)
        {
            return PlanningResult.Failed(jointResult.Errors
                .DefaultIfEmpty("Cartesian planning failed.")
                .ToArray());
        }

        goalProgress?.Invoke(1.0);
        var warnings = jointResult.Warnings.ToList();
        if (ndofNote is not null) warnings.Add(ndofNote);
        warnings.Add("TCP-LIN failed; used joint-space path to the Cartesian goal instead (not a straight TCP line).");
        return PlanningResult.Succeeded(jointResult.Trajectory!, warnings);
    }

    private static PlanningResult PlanRrt(
        PlanRequest request,
        JointState start,
        JointState goal,
        CancellationToken cancellationToken,
        Action<double>? goalProgress,
        ICollisionChecker? sharedChecker)
    {
        var ctx = request.Context;
        var planningContext = request.PlanningContext;
        var session = ctx.EffectiveModel;
        var checker = sharedChecker ?? GhExtract.TryCollisionChecker(
            session,
            ctx.Chain,
            planningContext.Scene,
            planningContext.Attached);
        if (checker is null)
            return PlanningResult.Failed(new[] { "No collision checker available for this robot model." });

        var opts = request.RrtSettings.ToOptions(cancellationToken, goalProgress);

        var req = new PlanningRequest(
            session,
            start,
            goal,
            planningContext.ToPlanningOptions(new PlanningOptions
            {
                CollisionScene = planningContext.Scene,
                CollisionChecker = checker
            }));

        var result = SamplingPlanner.Create(checker, opts).Plan(req);
        if (result.Success)
            goalProgress?.Invoke(1.0);
        return result;
    }
}
