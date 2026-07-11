using Grasshopper.Kernel;
using GH_IO.Serialization;
using Motus.Core;
using Motus.Geometry;
using Motus.GH;
using Motus.GH.Data;
using Motus.GH.UI;
using Motus.OMPL.NET;
using Motus.Rhino;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;

namespace Motus.GH.Components;

public sealed class MotusPlanComponent : MotusComponentBase
{
    private const double MaxJointStep = 0.05;
    private const double DefaultLinStepMeters = 0.005;
    private const int AutoPlanDebounceMs = 400;

    private List<PlanningResult>? _cached;
    private List<TrajectoryGoo>? _cachedGoos;
    private bool _run;
    private bool _autoPlan;
    private string? _lastPlannedFingerprint;
    private int _debounceGen;
    private int _planCancelGen;
    private int _planCancelGenAtStart;
    private bool _planningPending;

    public MotusPlanComponent() : base("Motus Plan", "Plan", "Plan motion to a plane (TCP LIN) or joint goal; click Plan or enable Auto Plan", "Plan", "flow-arrow") { }

    public override void CreateAttributes() =>
        m_attributes = new ButtonAttributes(this, () => _autoPlan ? "Replan" : "Plan", () => _autoPlan, RequestRun);

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Robot", "Rb", "Robot model", GH_ParamAccess.item);
        p.AddGenericParameter("Goal", "G", "Targets as Planes (TCP LIN) or Joint States", GH_ParamAccess.list);
        p.AddGenericParameter("Start", "S", "Start joint state (defaults to UR10e home or zeros)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Step", "St", "Plane goals only: TCP LIN step size (m)", GH_ParamAccess.item, DefaultLinStepMeters);
        p[p.ParamCount - 1].Optional = true;
        p.AddGenericParameter("Collision", "C", "Required for obstacle-aware planning. Without this, red obstacle previews are display-only. Joint goals → RRT-Connect; plane goals → LIN validate", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddGenericParameter("Group", "Gr", "Optional planning group (locks non-group joints)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddGenericParameter("Attach", "A", "Optional attached bodies list", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddGenericParameter("Trajectory", "T", "Planned trajectories", GH_ParamAccess.list);
        p.AddTextParameter("Status", "St", "Status message", GH_ParamAccess.item);
        p.AddTextParameter("Warnings", "W", "Warnings", GH_ParamAccess.list);
    }

    public override void AppendAdditionalMenuItems(ToolStripDropDown menu)
    {
        Menu_AppendItem(menu, "Auto Plan", AutoPlanMenuClick, true, _autoPlan);
        base.AppendAdditionalMenuItems(menu);
    }

    public override bool Write(GH_IWriter writer)
    {
        writer.SetBoolean("AutoPlan", _autoPlan);
        return base.Write(writer);
    }

    public override bool Read(GH_IReader reader)
    {
        if (reader.ItemExists("AutoPlan"))
            _autoPlan = reader.GetBoolean("AutoPlan");
        return base.Read(reader);
    }

    private void AutoPlanMenuClick(object? sender, EventArgs e)
    {
        RecordUndoEvent("Auto Plan");
        _autoPlan = !_autoPlan;
        _lastPlannedFingerprint = null;
        _debounceGen++;
        _planningPending = false;
        if (_autoPlan && HasCollisionInputsWired())
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                "Auto Plan enabled with collision — planning is debounced but may still be slow on large scenes.");
        }

        ExpireSolution(true);
    }

    private bool HasCollisionInputsWired() =>
        Params.Input[4].SourceCount > 0 || Params.Input[6].SourceCount > 0;

    private void RequestRun()
    {
        _run = true;
        _planningPending = false;
        ExpireSolution(true);
    }

    private static TrajectoryGoo TrajectoryFrom(RobotModelGoo robotGoo, Trajectory trajectory)
    {
        robotGoo.EnsureBundledTool();
        return new TrajectoryGoo(trajectory)
        {
            Chain = robotGoo.Chain,
            PreviewGeometry = robotGoo.EffectivePreviewGeometry(),
            PreviewMeshColors = robotGoo.PreviewMeshColors,
            BaseFrameOverride = robotGoo.BaseFrameOverride,
            ToolSnapshot = robotGoo.Tool
        };
    }

    private static Trajectory? AppendTrajectory(Trajectory? acc, Trajectory segment, RobotModel robot)
    {
        if (segment.Points.Count == 0) return acc;
        if (acc is null)
            return new Trajectory(robot, segment.Points.ToList());

        var points = acc.Points.ToList();
        var timeOffset = points[^1].TimeSeconds;
        for (var i = 1; i < segment.Points.Count; i++)
        {
            var pt = segment.Points[i];
            points.Add(new TrajectoryPoint(timeOffset + pt.TimeSeconds, pt.JointState));
        }
        return new Trajectory(robot, points);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!GhExtract.TryRobotGoo(da, 0, out var robotGoo)) return;
        var ctx = RobotContext.FromGoo(robotGoo);

        if (!GhExtract.TryGoals(da, 1, out var goals, out var goalErrors))
        {
            foreach (var error in goalErrors)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, error);
            if (goals.Count == 0)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Provide at least one valid Plane or Joint State goal.");
            EmitOutputs(da, null, GhExtract.PlanStatusKind.Manual);
            return;
        }

        var start = GhExtract.StartOrHome(da, 2, ctx.Model);
        var linStep = DefaultLinStepMeters;
        var stepInput = DefaultLinStepMeters;
        if (da.GetData(3, ref stepInput))
        {
            if (stepInput <= 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Step must be positive for plane goals.");
                EmitOutputs(da, null, GhExtract.PlanStatusKind.Manual);
                return;
            }

            linStep = stepInput;
        }

        var planningContext = GhExtract.BuildPlanningContext(ctx.EffectiveModel, da, 4, 5, 6);
        var fingerprint = PlanInputFingerprint.Compute(
            ctx.Model,
            robotGoo.BaseFrameOverride,
            robotGoo.Tool,
            goals,
            start,
            planningContext,
            linStep);

        var planNow = _run;
        if (planNow)
            _run = false;

        if (_autoPlan && !Locked && !planNow && fingerprint != _lastPlannedFingerprint)
        {
            _planCancelGen++;
            ScheduleDebouncedPlan(fingerprint);
            _planningPending = true;
        }

        if (!planNow)
        {
            EmitOutputs(da, fingerprint, ResolveIdleStatusKind(fingerprint));
            return;
        }

        _planCancelGenAtStart = _planCancelGen;
        _planningPending = false;
        RunPlanning(da, robotGoo, ctx, goals, start, planningContext, fingerprint, linStep);
    }

    private GhExtract.PlanStatusKind ResolveIdleStatusKind(string fingerprint)
    {
        if (_planningPending || (_autoPlan && fingerprint != _lastPlannedFingerprint))
            return GhExtract.PlanStatusKind.Planning;
        if (_autoPlan)
            return GhExtract.PlanStatusKind.AutoCached;
        return _cached is null ? GhExtract.PlanStatusKind.Manual : GhExtract.PlanStatusKind.ManualCached;
    }

    private void EmitOutputs(IGH_DataAccess da, string? fingerprint, GhExtract.PlanStatusKind statusKind)
    {
        if (_cachedGoos is { Count: > 0 })
            da.SetDataList(0, _cachedGoos);

        if (statusKind == GhExtract.PlanStatusKind.Planning && _cachedGoos is { Count: > 0 })
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                "Trajectory is from previous inputs; replanning…");
        }

        da.SetData(1, GhExtract.BuildStatusMessage(_cached, statusKind));
        if (_cached is not null)
            da.SetDataList(2, GhExtract.BuildWarnings(_cached));
    }

    private void ScheduleDebouncedPlan(string fingerprint)
    {
        var gen = ++_debounceGen;
        if (OnPingDocument() is not Grasshopper.Kernel.GH_Document doc) return;
        doc.ScheduleSolution(AutoPlanDebounceMs, _ =>
        {
            if (gen != _debounceGen || Locked || !_autoPlan) return;
            _run = true;
            ExpireSolution(true);
        });
    }

    private void RunPlanning(
        IGH_DataAccess da,
        RobotModelGoo robotGoo,
        RobotContext ctx,
        List<(JointState? joints, Plane? plane)> goals,
        JointState start,
        PlanningContext planningContext,
        string fingerprint,
        double linStep)
    {
        _cached = new List<PlanningResult>(goals.Count);
        _cachedGoos = new List<TrajectoryGoo>(goals.Count);
        var session = ctx.EffectiveModel;
        var currentStart = start;
        Trajectory? chained = null;

        var collisionInputWired = HasCollisionInputsWired();
        foreach (var goal in goals)
        {
            var result = goal.plane is { } plane
                ? PlanCartesianLin(ctx, planningContext, currentStart, plane, linStep, collisionInputWired)
                : (planningContext.Scene.Objects.Count > 0 || planningContext.Attached.Count > 0)
                    ? PlanRrt(ctx, planningContext, currentStart, goal.joints!)
                    : new JointLinearPlanner().Plan(new PlanningRequest(
                        session,
                        currentStart,
                        goal.joints!,
                        planningContext.ToPlanningOptions(new PlanningOptions { MaxJointStepRadians = MaxJointStep })));
            _cached.Add(result);
            if (result.Success && result.Trajectory is not null)
            {
                chained = AppendTrajectory(chained, result.Trajectory, session);
                currentStart = result.Trajectory.Points[^1].JointState;
            }
        }

        _lastPlannedFingerprint = fingerprint;
        if (chained is not null)
            _cachedGoos.Add(TrajectoryFrom(robotGoo, chained));
        else if (_cached.Any(r => r.Success))
        {
            foreach (var result in _cached)
            {
                if (result.Success && result.Trajectory is not null)
                    _cachedGoos.Add(TrajectoryFrom(robotGoo, result.Trajectory));
            }
        }

        if (_cachedGoos.Count > 0)
            da.SetDataList(0, _cachedGoos);

        if (!collisionInputWired && _cached.Any(r => r.Success))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                "Obstacle previews are visual only until ColScene is wired to Collision.");
        }

        var statusKind = _autoPlan ? GhExtract.PlanStatusKind.Auto : GhExtract.PlanStatusKind.Manual;
        da.SetData(1, GhExtract.BuildStatusMessage(_cached, statusKind));
        da.SetDataList(2, GhExtract.BuildWarnings(_cached));
    }

    private PlanningResult PlanCartesianLin(
        RobotContext ctx,
        PlanningContext planningContext,
        JointState start,
        Plane plane,
        double linStepMeters,
        bool collisionInputWired)
    {
        var session = ctx.EffectiveModel;
        var goal = new CartesianPose(FrameConversion.FromPlane(plane));
        if (!KinematicsResolver.SupportsModel(session.Preset, ctx.Chain))
        {
            return PlanningResult.Failed(new[]
            {
                $"No kinematics profile for '{session.Preset.ModelName}'."
            });
        }

        var needsCollision = PlanningCollision.SceneHasObstacles(planningContext.Scene) || planningContext.Attached.Count > 0;
        ICollisionChecker? checker = needsCollision
            ? GhExtract.TryCollisionChecker(session, ctx.Chain, planningContext.Scene, planningContext.Attached)
            : null;
        var opts = planningContext.ToPlanningOptions(new PlanningOptions
        {
            MaxJointStepRadians = MaxJointStep,
            CollisionChecker = checker,
            CollisionScene = planningContext.Scene
        });

        var linOptions = new CartesianLinOptions(StepMeters: linStepMeters, ContinueOnIkFailure: false);
        var linRequest = new CartesianPlanningRequest(session, start, goal, opts, planningContext.Scene);
        var linResult = new CartesianLinearPathPlanner(session.Preset, ctx.Chain).PlanToResult(linRequest, linOptions);
        if (linResult.Success)
        {
            var linWarnings = linResult.Warnings.ToList();
            if (!collisionInputWired)
                linWarnings.Add("Collision input unwired — plane goal planned in free space (LIN only).");
            else if (needsCollision)
                linWarnings.Add("Collision validated on link envelopes; TCP line may still intersect obstacles that do not hit link geometry.");
            return PlanningResult.Succeeded(linResult.Trajectory!, linWarnings);
        }

        if (linResult.Errors.Any(e => e.Contains("Collision", StringComparison.OrdinalIgnoreCase)))
            return linResult;

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

        AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
            "TCP-LIN failed; joint-space fallback used — TCP path is not straight.");
        var warnings = jointResult.Warnings.ToList();
        warnings.Add("TCP-LIN failed; used joint-space path to the Cartesian goal instead (not a straight TCP line).");
        return PlanningResult.Succeeded(jointResult.Trajectory!, warnings);
    }

    private PlanningResult PlanRrt(RobotContext ctx, PlanningContext planningContext, JointState start, JointState goal)
    {
        var session = ctx.EffectiveModel;
        var checker = GhExtract.TryCollisionChecker(session, ctx.Chain, planningContext.Scene, planningContext.Attached);
        if (checker is null)
            return PlanningResult.Failed(new[] { "No collision checker available for this robot model." });

        var opts = new RrtConnectOptions
        {
            MaxIterations = 4000,
            RandomSeed = 42,
            ShouldCancel = () => OnPingDocument() is null || _planCancelGen != _planCancelGenAtStart
        };

        var req = new PlanningRequest(
            session,
            start,
            goal,
            planningContext.ToPlanningOptions(new PlanningOptions
            {
                CollisionScene = planningContext.Scene,
                CollisionChecker = checker
            }));

        return new RrtConnectPlanner(checker, opts).Plan(req);
    }

    public override Guid ComponentGuid => new Guid("8bb0bae3-527f-4e80-a8a4-c8a88b7276de");
}
