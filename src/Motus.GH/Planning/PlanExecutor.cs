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
    /// <summary>True when Family=legged PlanBodyPath synthesized a full-driver gait.</summary>
    public bool LeggedGaitSynthesized { get; init; }
    public IReadOnlyList<Frame>? LeggedBasePath { get; init; }
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
        // Mobility SE2 plans against the arm Model, not EffectiveModel (group/session).
        var planningRobot = request.Context.MobilityGoal is not null
            ? request.Context.Model
            : session;
        ICollisionChecker? sharedChecker = null;
        if (needsCollision)
        {
            var checkerSw = Stopwatch.StartNew();
            sharedChecker = request.Context.Stewart is not null
                ? new StewartCollisionChecker(request.Context.Stewart)
                : GhExtract.TryCollisionChecker(
                    planningRobot,
                    request.Context.Chain,
                    request.PlanningContext.Scene,
                    request.PlanningContext.Attached,
                    request.Context);
            if (timings is not null)
                timings.CheckerBuildMs = checkerSw.ElapsedMilliseconds;
        }

        // Family=legged body-path gait: one-shot over all plane goals (not per-plane TCP LIN).
        if (TryPlanLeggedBodyPath(request, sharedChecker, cancellationToken, timings, out var leggedExec))
            return leggedExec!;

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
            // GroupMap locks non-group joints at segment start (JointLinear used to ignore GroupMap).
            if (goal.joints is { } goalJs &&
                request.PlanningContext.ActiveGroup is not null &&
                session.JointNames is { Count: > 0 })
            {
                var map = JointIndexMap.Resolve(session, request.PlanningContext.ActiveGroup);
                goal = (new JointState(map.EmbedGroupState(currentStart, map.ExtractGroupPositions(goalJs)).Positions.ToArray()), null);
            }

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
                var useSampling = needsCollision || request.Context.MobilityGoal is not null;
                result = goal.plane is { } plane
                    ? PlanCartesianLin(request, currentStart, plane, cancellationToken, goalProgress, sharedChecker)
                    : useSampling
                        ? PlanRrt(request, currentStart, goal.joints!, cancellationToken, goalProgress, sharedChecker)
                        : new JointLinearPlanner().Plan(new PlanningRequest(
                            session,
                            currentStart,
                            goal.joints!,
                            BuildPlanningOptions(request, MaxJointStep, checker: null)));
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
        if (ctx.IsStewart || Units.IsStewart(session.Preset))
        {
            var stewartGoal = new CartesianPose(FrameConversion.FromPlanePlate(plane));
            return PlanStewartLin(request, start, stewartGoal, cancellationToken, goalProgress, sharedChecker);
        }

        var goal = new CartesianPose(FrameConversion.FromPlane(plane));
        var tipN = ctx.Chain?.Joints.Length ?? 0;
        var allDrivers = tipN > 0 && session.Preset.AxisCount > tipN;
        var linRobot = allDrivers ? GhExtract.TipPathModel(session, tipN) : session;
        var linStart = allDrivers ? KinematicsPreview.TipJointState(start, tipN) : start;
        var checker = allDrivers && sharedChecker is not null
            ? PadBranchesChecker.Wrap(sharedChecker, start, tipN)
            : sharedChecker;

        if (!KinematicsResolver.SupportsModel(linRobot.Preset, ctx.Chain))
        {
            return PlanningResult.Failed(new[]
            {
                $"No kinematics profile for '{session.Preset.ModelName}'."
            });
        }

        if (cancellationToken.IsCancellationRequested)
            return PlanningResult.Failed(new[] { "Planning cancelled." });

        goalProgress?.Invoke(0.25);

        var ndofNote = allDrivers
            ? $"AllDrivers plane/LIN: tip-chain IK only; side branches held at start ({session.Preset.AxisCount - tipN} axes). Use joint goals to move them."
            : session.Preset.AxisCount != 6
                ? $"Plane goal on {session.Preset.AxisCount}-axis robot uses numerical IK (not UR analytic)."
                : null;

        var needsCollision = PlanningCollision.SceneHasObstacles(planningContext.Scene) || planningContext.Attached.Count > 0;
        ICollisionChecker? planChecker = needsCollision ? checker : null;
        var opts = planningContext.ToPlanningOptions(new PlanningOptions
        {
            MaxJointStepRadians = MaxJointStep,
            CollisionChecker = planChecker,
            CollisionScene = planningContext.Scene
        });

        var linOptions = new CartesianLinOptions(StepMeters: request.LinStepMeters, ContinueOnIkFailure: false);
        var linRequest = new CartesianPlanningRequest(linRobot, linStart, goal, opts, planningContext.Scene);
        var linResult = new CartesianLinearPathPlanner(linRobot.Preset, ctx.Chain).PlanToResult(linRequest, linOptions);
        if (linResult.Success)
        {
            goalProgress?.Invoke(1.0);
            var traj = EmbedSideBranches(linResult.Trajectory!, session, start, tipN);
            var linWarnings = linResult.Warnings.ToList();
            if (ndofNote is not null) linWarnings.Add(ndofNote);
            if (!request.CollisionInputWired)
                linWarnings.Add("Collision input unwired — plane goal planned in free space (LIN only).");
            else if (needsCollision)
                linWarnings.Add("Collision validated on link envelopes; TCP line may still intersect obstacles that do not hit link geometry.");
            return PlanningResult.Succeeded(traj, linWarnings);
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
                linRobot,
                ctx.Chain,
                linStart,
                goal,
                planningContext,
                checker!,
                rrtOpts,
                linResult.Errors);
            if (rrtFallback.Success)
            {
                goalProgress?.Invoke(1.0);
                var traj = EmbedSideBranches(rrtFallback.Trajectory!, session, start, tipN);
                var w = rrtFallback.Warnings.ToList();
                if (ndofNote is not null && !w.Contains(ndofNote))
                    w.Add(ndofNote);
                return PlanningResult.Succeeded(traj, w);
            }
            return rrtFallback;
        }

        if (cancellationToken.IsCancellationRequested)
            return PlanningResult.Failed(new[] { "Planning cancelled." });

        goalProgress?.Invoke(0.5);

        var reach = new CartesianGoalSolver().TryReach(
            linRobot,
            goal,
            CartesianGoalSolver.EnumerateDefaultSeeds(linStart, linRobot),
            ctx.Chain);
        if (!reach.Success)
        {
            return PlanningResult.Failed(reach.Errors.Concat(new[]
            {
                "TCP-LIN failed at intermediate poses. For large moves use a Joint State goal or wire Start near the target."
            }).ToArray());
        }

        var goalJoints = reach.Solution!;
        var jointResult = new JointLinearPlanner().Plan(new PlanningRequest(linRobot, linStart, goalJoints, opts));
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
        return PlanningResult.Succeeded(EmbedSideBranches(jointResult.Trajectory!, session, start, tipN), warnings);
    }

    private static Trajectory EmbedSideBranches(Trajectory tipTraj, RobotModel full, JointState fullStart, int tipN)
    {
        if (tipN <= 0 || fullStart.AxisCount <= tipN)
            return tipTraj.Robot.Preset.AxisCount == full.Preset.AxisCount
                ? tipTraj
                : new Trajectory(full, tipTraj.Points);

        var points = new List<TrajectoryPoint>(tipTraj.Points.Count);
        foreach (var pt in tipTraj.Points)
        {
            var q = new double[fullStart.AxisCount];
            var tipQ = pt.JointState.Positions;
            for (var i = 0; i < tipN && i < tipQ.Length; i++)
                q[i] = tipQ[i];
            for (var i = tipN; i < fullStart.AxisCount; i++)
                q[i] = fullStart.Positions[i];
            points.Add(new TrajectoryPoint(pt.TimeSeconds, new JointState(q)));
        }
        return new Trajectory(full, points);
    }

    /// <summary>Pads tip-only JointState with side-branch values from <paramref name="fullStart"/> for TreeFK collision.</summary>
    private sealed class PadBranchesChecker : ICollisionChecker
    {
        private readonly ICollisionChecker _inner;
        private readonly double[] _full;
        private readonly int _tipN;
        private JointState? _scratch;

        private PadBranchesChecker(ICollisionChecker inner, JointState fullStart, int tipN)
        {
            _inner = inner;
            _tipN = tipN;
            _full = fullStart.Positions.ToArray();
        }

        public static ICollisionChecker Wrap(ICollisionChecker inner, JointState fullStart, int tipN) =>
            tipN <= 0 || fullStart.AxisCount <= tipN
                ? inner
                : new PadBranchesChecker(inner, fullStart, tipN);

        public bool IsCollisionFree(JointState state, CollisionScene scene) =>
            _inner.IsCollisionFree(Pad(state), scene);

        public bool SegmentCollisionFree(JointState from, JointState to, CollisionScene scene, double stepRadians)
        {
            // Don't share scratch across from/to — Pad mutates one buffer.
            var fromPad = PadCopy(from);
            var toPad = PadCopy(to);
            return _inner.SegmentCollisionFree(fromPad, toPad, scene, stepRadians);
        }

        private JointState Pad(JointState state)
        {
            if (state.AxisCount >= _full.Length)
                return state;
            var q = _scratch?.Positions;
            if (q is null || q.Length != _full.Length)
            {
                q = new double[_full.Length];
                _scratch = JointState.Wrap(q);
            }
            for (var i = 0; i < _tipN && i < state.AxisCount; i++)
                q[i] = state.Positions[i];
            for (var i = _tipN; i < _full.Length; i++)
                q[i] = _full[i];
            return _scratch!;
        }

        private JointState PadCopy(JointState state)
        {
            if (state.AxisCount >= _full.Length)
                return state;
            var q = new double[_full.Length];
            for (var i = 0; i < _tipN && i < state.AxisCount; i++)
                q[i] = state.Positions[i];
            for (var i = _tipN; i < _full.Length; i++)
                q[i] = _full[i];
            return new JointState(q);
        }
    }

    private static PlanningResult PlanStewartLin(
        PlanRequest request,
        JointState start,
        CartesianPose goal,
        CancellationToken cancellationToken,
        Action<double>? goalProgress,
        ICollisionChecker? sharedChecker)
    {
        var ctx = request.Context;
        if (ctx.Stewart is null)
        {
            return PlanningResult.Failed([
                "Stewart robot is missing StewartPlatform handle. Use Motus Stewart (not Motus Robot URDF)."]);
        }

        if (cancellationToken.IsCancellationRequested)
            return PlanningResult.Failed(["Planning cancelled."]);

        goalProgress?.Invoke(0.2);
        var mid = 0.5 * (ctx.Stewart.StrokeLimits[0].Min + ctx.Stewart.StrokeLimits[0].Max);
        var midSeed = new CartesianPose(new Frame(0, 0, mid));
        var fk = new StewartForwardKinematics(ctx.Stewart);
        // Mid-stroke seed: HomeLengths residual≈0 (example 08). Unseeded avg-L guess + Motus.NET
        // ≤0.13 FD condition gate false-singular'd at iteration 0.
        var startFk = fk.TrySolve(start, midSeed);
        if (!startFk.Success || startFk.Pose is null)
            return PlanningResult.Failed([$"Stewart start FK failed: {startFk}"]);
        var startPose = startFk.Pose;

        goalProgress?.Invoke(0.4);
        var hasCollision = PlanningCollision.SceneHasObstacles(request.PlanningContext.Scene)
            || request.PlanningContext.Attached.Count > 0;
        var checker = sharedChecker ?? (hasCollision ? new StewartCollisionChecker(ctx.Stewart) : null);
        var opts = BuildPlanningOptions(
            request,
            request.RrtSettings.StepRadians > 0 ? request.RrtSettings.StepRadians : MaxJointStep,
            checker);
        var planner = new StewartCartesianPathPlanner(ctx.Stewart);
        var result = planner.PlanToResult(
            startPose,
            goal,
            start,
            request.LinStepMeters,
            planningOptions: opts);
        if (!result.Success)
        {
            if (hasCollision && IsCollisionFailure(result))
            {
                if (cancellationToken.IsCancellationRequested)
                    return PlanningResult.Failed(["Planning cancelled."]);

                var goalIk = new StewartInverseKinematics(ctx.Stewart).TrySolveDetailed(goal);
                if (goalIk.Success && goalIk.JointState is not null)
                {
                    goalProgress?.Invoke(0.55);
                    var rrt = PlanRrt(request, start, goalIk.JointState, cancellationToken, goalProgress, checker);
                    if (rrt.Success && rrt.Trajectory is not null)
                    {
                        var rrtWarnings = rrt.Warnings.ToList();
                        rrtWarnings.Add("Stewart TCP-LIN collided; used RRT in leg-length space instead (not a straight TCP platform path).");
                        AddStewartMoveJWarning(rrtWarnings);
                        return PlanningResult.Succeeded(rrt.Trajectory, rrtWarnings);
                    }

                    return rrt;
                }
            }

            return result;
        }

        goalProgress?.Invoke(1.0);
        var warnings = result.Warnings.ToList();
        AddStewartMoveJWarning(warnings);
        return PlanningResult.Succeeded(result.Trajectory!, warnings);
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
        var hasCollision = PlanningCollision.SceneHasObstacles(planningContext.Scene)
            || planningContext.Attached.Count > 0;
        var planningRobot = ctx.MobilityGoal is not null ? ctx.Model : session;
        var checker = sharedChecker
            ?? (ctx.Stewart is not null && hasCollision
                ? new StewartCollisionChecker(ctx.Stewart)
                : GhExtract.TryCollisionChecker(
                    planningRobot,
                    ctx.Chain,
                    planningContext.Scene,
                    planningContext.Attached,
                    ctx));
        if (hasCollision && checker is null)
            return PlanningResult.Failed(new[] { "No collision checker available for this robot model." });

        var opts = request.RrtSettings.ToOptions(cancellationToken, goalProgress);

        var req = new PlanningRequest(
            planningRobot,
            start,
            goal,
            BuildPlanningOptions(request, request.RrtSettings.StepRadians, checker));

        var planner = checker is not null
            ? SamplingPlanner.Create(checker, opts)
            : new SamplingPlanner(planningRobot.Preset, ctx.Chain, opts);
        var result = planner.Plan(req);
        if (result.Success)
            goalProgress?.Invoke(1.0);
        return result;
    }

    private static PlanningOptions BuildPlanningOptions(
        PlanRequest request,
        double maxStep,
        ICollisionChecker? checker) =>
        request.PlanningContext.ToPlanningOptions(new PlanningOptions
        {
            MaxJointStepRadians = maxStep,
            CollisionScene = request.PlanningContext.Scene,
            CollisionChecker = checker,
            Mobility = request.Context.MobilityGoal
        });

    private static bool IsCollisionFailure(PlanningResult result) =>
        result.Errors.Any(e => e.Contains("collision", StringComparison.OrdinalIgnoreCase)) ||
        result.Messages.Any(m => m.Code.Contains("collision", StringComparison.OrdinalIgnoreCase));

    private static void AddStewartMoveJWarning(List<string> warnings)
    {
        const string warning = "Stewart TCP-LIN: JointState = leg lengths (meters). Not UR MoveJ radians.";
        if (!warnings.Any(w => string.Equals(w, warning, StringComparison.Ordinal)))
            warnings.Add(warning);
    }

    /// <summary>
    /// Family=legged one-shot body-path gait. Returns true when this request was handled
    /// (success or named fail). Returns false to fall through to tip LIN / joint / RRT.
    /// </summary>
    private static bool TryPlanLeggedBodyPath(
        PlanRequest request,
        ICollisionChecker? sharedChecker,
        CancellationToken cancellationToken,
        PlanPhaseTimings? timings,
        out PlanExecutionResult? exec)
    {
        exec = null;
        var ctx = request.Context;
        var isLegged = Units.IsLegged(ctx.EffectiveModel.Preset)
            || Units.IsLegged(ctx.Model.Preset)
            || ctx.Mechanism is not null;
        if (!isLegged)
            return false;

        var goals = request.Goals;
        if (goals.Count == 0)
            return false;

        var anyPlane = false;
        var anyJoint = false;
        for (var i = 0; i < goals.Count; i++)
        {
            if (goals[i].plane is not null) anyPlane = true;
            if (goals[i].joints is not null) anyJoint = true;
        }

        // Joint-only tip path — leave to existing planners.
        if (!anyPlane)
            return false;

        if (ctx.Mechanism is null)
        {
            exec = FailedLegged(
                "Legged plane goals need Mechanism handle (Motus Mechanism → Walk → Rb). " +
                "Do not interpret planes as tip TCP LIN without Mechanism.");
            return true;
        }

        if (anyJoint && anyPlane)
        {
            exec = FailedLegged(
                "Legged Plan rejects mixed plane+joint goals. " +
                "Use all planes (≥2) for body-path gait, or joints / one plane for tip-path.");
            return true;
        }

        // Single plane → tip foot TCP LIN (unchanged).
        if (goals.Count < 2)
            return false;

        if (cancellationToken.IsCancellationRequested)
        {
            exec = new PlanExecutionResult { Cancelled = true };
            return true;
        }

        var pathXy = new List<Vec3>(goals.Count);
        for (var i = 0; i < goals.Count; i++)
        {
            var plane = goals[i].plane!.Value;
            var o = plane.Origin;
            if (!double.IsFinite(o.X) || !double.IsFinite(o.Y) || !double.IsFinite(o.Z))
            {
                exec = FailedLegged($"Goal[{i}]: plane origin NaN/Inf (m).");
                return true;
            }

            pathXy.Add(new Vec3(o.X, o.Y, 0));
        }

        var mechanism = ctx.Mechanism;
        var bodyPose = new PathFollowBodyPose(clearanceMeters: mechanism.NominalBodyClearance);
        var options = BuildPlanningOptions(request, MaxJointStep, sharedChecker);

        var plannerSw = Stopwatch.StartNew();
        var plan = LeggedGait.PlanBodyPath(
            mechanism,
            pathXy,
            out var gait,
            model: null,
            speed: LeggedGait.DefaultSpeedMetersPerSecond,
            stepLength: LeggedGait.DefaultStepLengthMeters,
            stepHeight: LeggedGait.DefaultStepHeightMeters,
            hipStance: ctx.HipStanceRadians,
            femurStance: ctx.FemurStanceRadians,
            tibiaStance: ctx.TibiaStanceRadians,
            bodyPose: bodyPose,
            terrain: null,
            options: options);
        if (timings is not null)
        {
            timings.PlannerMs += plannerSw.ElapsedMilliseconds;
            timings.GoalCount = goals.Count;
        }

        if (!plan.Success || plan.Trajectory is null)
        {
            exec = new PlanExecutionResult { Results = [plan] };
            return true;
        }

        var warnings = plan.Warnings.ToList();
        warnings.Insert(0,
            $"Legged body-path ({goals.Count} planes): origins only (m); orientation ignored; yaw from path. " +
            "Start/Step unused. Flat Z=0 (no Terrain on Plan). SSM hard-fail.");
        var succeeded = PlanningResult.Succeeded(plan.Trajectory, warnings);

        exec = new PlanExecutionResult
        {
            Results = [succeeded],
            ChainedTrajectory = plan.Trajectory,
            LeggedGaitSynthesized = true,
            LeggedBasePath = gait?.BasePath
        };
        return true;
    }

    private static PlanExecutionResult FailedLegged(string message) =>
        new()
        {
            Results =
            [
                PlanningResult.Failed(new[]
                {
                    new PlanningMessage(
                        PlanningMessageCodes.InvalidInput,
                        message,
                        PlanningMessageSeverity.Error)
                })
            ]
        };
}
