using Grasshopper.Kernel;
using Motus.Core;
using Motus.Geometry;
using Motus.GH.Async;
using Motus.GH.Components;
using Motus.GH.Data;
using Motus.GH.Planning;
using Motus.GH.Rhino;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Threading;

namespace Motus.GH.Planning;

internal sealed class PlanWorker : WorkerInstance, IWorkerSkip
{
    private readonly MotusPlanComponent _owner;

    public bool SkipWork { get; private set; }

    public RobotContext Context { get; private set; }
    public List<(JointState? joints, Plane? plane)> Goals { get; } = [];
    public JointState Start { get; private set; } = null!;
    public PlanningContext PlanningContext { get; private set; } = null!;
    public double LinStepMeters { get; private set; }
    public RrtPlanSettings RrtSettings { get; private set; } = RrtPlanSettings.Defaults;
    public bool CollisionInputWired { get; private set; }
    public string Fingerprint { get; private set; } = string.Empty;
    public bool IsAutoPlan { get; private set; }

    public SerialJointChain? Chain { get; private set; }
    public RobotCollisionModel? PreviewGeometry { get; private set; }
    public Color?[]? PreviewMeshColors { get; private set; }
    public Frame? BaseFrameOverride { get; private set; }
    public ToolDefinition? ToolSnapshot { get; private set; }

    public PlanExecutionResult? Result { get; private set; }
    public List<string> RuntimeRemarks { get; } = [];

    public PlanWorker(MotusPlanComponent owner) => _owner = owner;

    public override WorkerInstance Duplicate() => new PlanWorker(_owner);

    public override void GetData(IGH_DataAccess da, GH_ComponentParamServer parameters)
    {
        SkipWork = false;
        Result = null;
        RuntimeRemarks.Clear();
        Goals.Clear();

        if (!GhExtract.TryRobotGoo(da, 0, out var robotGoo))
        {
            SkipWork = true;
            return;
        }

        robotGoo.EnsureBundledTool();
        Context = RobotContext.FromGoo(robotGoo);
        Chain = robotGoo.Chain;
        PreviewGeometry = robotGoo.EffectivePreviewGeometry();
        PreviewMeshColors = robotGoo.PreviewMeshColors;
        BaseFrameOverride = robotGoo.BaseFrameOverride;
        ToolSnapshot = robotGoo.Tool;

        if (!GhExtract.TryGoals(da, 1, out var goals, out _))
        {
            SkipWork = true;
            return;
        }

        Goals.AddRange(goals);
        Start = GhExtract.StartOrHome(da, 2, Context.Model);
        LinStepMeters = MotusPlanComponent.DefaultLinStepMeters;
        var stepInput = LinStepMeters;
        if (da.GetData(3, ref stepInput))
            LinStepMeters = stepInput;

        PlanningContext = GhExtract.BuildPlanningContext(
            Context.EffectiveModel,
            da,
            MotusPlanInputs.Collision,
            MotusPlanInputs.Group,
            MotusPlanInputs.Attach,
            GhExtract.ParseCollisionInput(da, MotusPlanInputs.Collision).Scene);
        if (!RrtPlanSettings.TryRead(
                da,
                MotusPlanInputs.RrtMaxIter,
                MotusPlanInputs.RrtTimeLimit,
                MotusPlanInputs.RrtPlanner,
                MotusPlanInputs.RrtGoalBias,
                MotusPlanInputs.RrtStep,
                out var rrtSettings,
                out _))
        {
            SkipWork = true;
            return;
        }

        RrtSettings = rrtSettings;
        CollisionInputWired = _owner.HasCollisionInputsWired();
        IsAutoPlan = _owner.AutoPlanEnabled;

        Fingerprint = PlanInputFingerprint.Compute(
            Context.Model,
            BaseFrameOverride,
            ToolSnapshot,
            Goals,
            Start,
            PlanningContext,
            LinStepMeters,
            RrtSettings);
    }

    public override void DoWork(Action<string, double> reportProgress, Action done)
    {
        var lastProgress = 0.0;
        var progressLock = new object();

        void Report(double fraction)
        {
            double publish;
            lock (progressLock)
            {
                if (fraction <= lastProgress)
                    publish = lastProgress;
                else
                {
                    lastProgress = fraction;
                    publish = fraction;
                }
            }

            reportProgress("plan", publish);
        }

        // Native OMPL has no iteration callback — nudge progress so the UI does not sit at 0%.
        using var heartbeat = new System.Threading.Timer(_ =>
        {
            if (CancellationToken.IsCancellationRequested)
                return;

            lock (progressLock)
            {
                if (lastProgress >= 0.95)
                    return;
                lastProgress = Math.Min(0.95, lastProgress + 0.03);
                reportProgress("plan", lastProgress);
            }
        }, null, 400, 400);

        try
        {
            Report(0);
            var request = new PlanRequest(Context, Goals, Start, PlanningContext, LinStepMeters, CollisionInputWired, RrtSettings);
            Result = PlanExecutor.Execute(request, CancellationToken, Report);
            Report(1);

            if (Result.Cancelled)
            {
                CompletionMessage = "Cancelled";
                return;
            }

            if (!CollisionInputWired && Result.Results.Any(r => r.Success))
                RuntimeRemarks.Add("Obstacle previews are visual only until ColScene is wired to Collision.");
            else if (CollisionInputWired &&
                     !PlanningCollision.SceneHasObstacles(PlanningContext.Scene) &&
                     PlanningContext.Attached.Count == 0 &&
                     Result.Results.Any(r => r.Success))
            {
                RuntimeRemarks.Add("Collision input ignored — wire ColMesh/ColSphere → ColScene → Plan, or pass a valid mesh/Brep/ColScene.");
            }

            foreach (var result in Result.Results)
            {
                if (result.Success && result.Warnings.Any(w => w.Contains("TCP-LIN failed; used joint-space", StringComparison.OrdinalIgnoreCase)))
                {
                    RuntimeRemarks.Add("TCP-LIN failed; joint-space fallback used — TCP path is not straight.");
                    break;
                }
            }

            CompletionMessage = Result.Results.All(r => r.Success) ? "Done" : "Failed";
        }
        finally
        {
            done();
        }
    }

    public override void SetData(IGH_DataAccess da)
    {
        if (Result is null || Result.Cancelled)
            return;

        var goos = BuildTrajectoryGoos();
        CommitCachedResults();
        WriteOutputs(da, Result.Results, goos, IsAutoPlan);
    }

    public override void CommitCachedResults()
    {
        if (Result is null || Result.Cancelled)
            return;

        var goos = BuildTrajectoryGoos();
        _owner.ApplyWorkerResult(Fingerprint, Result.Results, goos, IsAutoPlan, RuntimeRemarks);
    }

    private List<TrajectoryGoo> BuildTrajectoryGoos()
    {
        var goos = new List<TrajectoryGoo>();
        if (Result is null)
            return goos;

        if (Result.ChainedTrajectory is { } chained)
        {
            goos.Add(TrajectoryFrom(chained));
            return goos;
        }

        foreach (var trajectory in Result.SegmentTrajectories)
            goos.Add(TrajectoryFrom(trajectory));
        return goos;
    }

    private TrajectoryGoo TrajectoryFrom(Trajectory trajectory)
    {
        var robot = trajectory.Robot;
        return new TrajectoryGoo(trajectory)
        {
            Chain = Chain,
            PreviewGeometry = PreviewGeometry ?? robot.CollisionModel,
            PreviewMeshColors = PreviewMeshColors,
            BaseFrameOverride = BaseFrameOverride,
            ToolSnapshot = ToolSnapshot
        };
    }

    private void WriteOutputs(
        IGH_DataAccess da,
        IReadOnlyList<PlanningResult> results,
        IReadOnlyList<TrajectoryGoo> goos,
        bool isAutoPlan)
    {
        if (goos.Count > 0)
            da.SetDataList(0, goos);

        var statusKind = isAutoPlan ? GhExtract.PlanStatusKind.Auto : GhExtract.PlanStatusKind.Manual;
        da.SetData(1, GhExtract.BuildStatusMessage(results, statusKind));
        da.SetDataList(2, GhExtract.BuildWarnings(results));
    }
}
