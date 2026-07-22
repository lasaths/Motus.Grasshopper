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
    public bool UsedDefaultStart { get; init; }
    public PlanningContext PlanningContext { get; init; } = null!;
    public double LinStepMeters { get; init; }
    public RrtPlanSettings RrtSettings { get; init; } = RrtPlanSettings.Defaults;
    public bool CollisionInputWired { get; init; }
    public string Fingerprint { get; init; } = string.Empty;
    public bool IsAutoPlan { get; init; }

    public SerialJointChain? Chain { get; init; }
    public KinematicTree? Tree { get; init; }
    public RobotCollisionModel? PreviewGeometry { get; init; }
    public Color?[]? PreviewMeshColors { get; init; }
    public Frame? BaseFrameOverride { get; init; }
    public ToolDefinition? ToolSnapshot { get; init; }

    public static bool TryCollect(
        IGH_DataAccess da,
        MotusPlanComponent owner,
        out PlanInputSnapshot? snapshot,
        out string? error)
    {
        snapshot = null;
        error = null;

        var robotIdx = MotusPlanInputs.IndexOf(owner, MotusPlanInputs.Robot);
        var goalIdx = MotusPlanInputs.IndexOf(owner, MotusPlanInputs.Goal);
        var startIdx = MotusPlanInputs.IndexOf(owner, MotusPlanInputs.Start);
        var stepIdx = MotusPlanInputs.IndexOf(owner, MotusPlanInputs.Step);
        var collisionIdx = MotusPlanInputs.IndexOf(owner, MotusPlanInputs.Collision);
        var groupIdx = MotusPlanInputs.IndexOf(owner, MotusPlanInputs.Group);
        var attachIdx = MotusPlanInputs.IndexOf(owner, MotusPlanInputs.Attach);
        var rrtIdx = MotusPlanInputs.IndexOf(owner, MotusPlanInputs.RrtSettings);

        if (robotIdx < 0 || goalIdx < 0)
            return false;

        if (!GhExtract.TryRobotGoo(da, robotIdx, out var robotGoo))
            return false;

        robotGoo.EnsureBundledTool();
        var context = RobotContext.FromGoo(robotGoo);

        if (!GhExtract.TryGoals(da, goalIdx, out var goals, out _))
            return false;

        if (!GhExtract.TryStartOrHome(da, startIdx, context, out var start, out var usedDefaultStart, out error))
            return false;
        var linStep = MotusPlanComponent.DefaultLinStepMeters;
        var stepInput = linStep;
        if (stepIdx >= 0 && da.GetData(stepIdx, ref stepInput))
            linStep = stepInput;

        var collisionParse = GhExtract.ParseCollisionInput(da, collisionIdx);
        if (collisionParse.Error is not null)
            return false;

        var planningContext = GhExtract.BuildPlanningContext(
            context.EffectiveModel,
            da,
            collisionIdx,
            groupIdx,
            attachIdx,
            collisionParse.Scene);

        var rrtSettings = GhExtract.ResolveRrtSettings(da, rrtIdx, owner);
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
            UsedDefaultStart = usedDefaultStart,
            PlanningContext = planningContext,
            LinStepMeters = linStep,
            RrtSettings = rrtSettings,
            CollisionInputWired = collisionParse.Wired,
            Fingerprint = fingerprint,
            IsAutoPlan = owner.AutoPlanEnabled,
            Chain = robotGoo.Chain,
            Tree = robotGoo.Tree,
            PreviewGeometry = robotGoo.EffectivePreviewGeometry(),
            PreviewMeshColors = robotGoo.PreviewMeshColors,
            BaseFrameOverride = robotGoo.BaseFrameOverride,
            ToolSnapshot = robotGoo.Tool
        };
        return true;
    }
}
