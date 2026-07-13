using Grasshopper.Kernel;
using Motus.Core;
using Motus.Geometry;
using Motus.GH.Components;
using Motus.GH.Data;
using Motus.GH.Rhino;
using Rhino.Geometry;
using System.Collections.Generic;
using System.Drawing;

namespace Motus.GH.Planning;

/// <summary>Immutable plan inputs collected once per solve/launch to avoid duplicate parsing.</summary>
internal sealed class PlanInputSnapshot
{
    public RobotContext Context { get; init; } = default!;
    public IReadOnlyList<(JointState? joints, Plane? plane)> Goals { get; init; } = [];
    public JointState Start { get; init; } = null!;
    public PlanningContext PlanningContext { get; init; } = null!;
    public double LinStepMeters { get; init; }
    public RrtPlanSettings RrtSettings { get; init; } = RrtPlanSettings.Defaults;
    public bool CollisionInputWired { get; init; }
    public string Fingerprint { get; init; } = string.Empty;
    public bool IsAutoPlan { get; init; }

    public SerialJointChain? Chain { get; init; }
    public RobotCollisionModel? PreviewGeometry { get; init; }
    public Color?[]? PreviewMeshColors { get; init; }
    public Frame? BaseFrameOverride { get; init; }
    public ToolDefinition? ToolSnapshot { get; init; }

    public static bool TryCollect(
        IGH_DataAccess da,
        MotusPlanComponent owner,
        out PlanInputSnapshot? snapshot)
    {
        snapshot = null;

        if (!GhExtract.TryRobotGoo(da, MotusPlanInputs.Robot, out var robotGoo))
            return false;

        robotGoo.EnsureBundledTool();
        var context = RobotContext.FromGoo(robotGoo);

        if (!GhExtract.TryGoals(da, MotusPlanInputs.Goal, out var goals, out _))
            return false;

        var start = GhExtract.StartOrHome(da, MotusPlanInputs.Start, context.Model);
        var linStep = MotusPlanComponent.DefaultLinStepMeters;
        var stepInput = linStep;
        if (da.GetData(MotusPlanInputs.Step, ref stepInput))
            linStep = stepInput;

        var collisionParse = GhExtract.ParseCollisionInput(da, MotusPlanInputs.Collision);
        if (collisionParse.Error is not null)
            return false;

        var planningContext = GhExtract.BuildPlanningContext(
            context.EffectiveModel,
            da,
            MotusPlanInputs.Collision,
            MotusPlanInputs.Group,
            MotusPlanInputs.Attach,
            collisionParse.Scene);

        var rrtSettings = GhExtract.ResolveRrtSettings(da, MotusPlanInputs.RrtSettings, owner);
        var needsSampling = GhExtract.GoalsNeedSamplingPlanner(goals, planningContext);
        var fingerprintRrt = needsSampling ? rrtSettings : RrtPlanSettings.Defaults;
        var fingerprint = PlanInputFingerprint.Compute(
            context.Model,
            robotGoo.BaseFrameOverride,
            robotGoo.Tool,
            goals,
            start,
            planningContext,
            linStep,
            fingerprintRrt.PlannerId,
            fingerprintRrt.MaxIterations,
            fingerprintRrt.MaxPlanTimeSeconds,
            fingerprintRrt.GoalBias,
            fingerprintRrt.StepRadians);

        snapshot = new PlanInputSnapshot
        {
            Context = context,
            Goals = goals,
            Start = start,
            PlanningContext = planningContext,
            LinStepMeters = linStep,
            RrtSettings = rrtSettings,
            CollisionInputWired = collisionParse.Wired,
            Fingerprint = fingerprint,
            IsAutoPlan = owner.AutoPlanEnabled,
            Chain = robotGoo.Chain,
            PreviewGeometry = robotGoo.EffectivePreviewGeometry(),
            PreviewMeshColors = robotGoo.PreviewMeshColors,
            BaseFrameOverride = robotGoo.BaseFrameOverride,
            ToolSnapshot = robotGoo.Tool
        };
        return true;
    }
}
