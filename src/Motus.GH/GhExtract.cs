using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Motus.Core;
using Motus.Geometry;
using Motus.GH.Data;
using Motus.OMPL.NET;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Motus.GH;

internal static class GhExtract
{
    public static bool TryRobotGoo(IGH_DataAccess da, int index, out RobotModelGoo goo)
    {
        goo = null!;
        RobotModelGoo? g = null;
        if (!da.GetData(index, ref g) || g?.Value is null) return false;
        g.EnsureChainFromPath();
        goo = g;
        return true;
    }

    public static bool TryRobot(IGH_DataAccess da, int index, out RobotModel robot)
    {
        robot = null!;
        if (!TryRobotGoo(da, index, out var goo)) return false;
        robot = goo.Value!;
        return true;
    }

    public static bool TryRobotContext(IGH_DataAccess da, int index, out RobotContext ctx)
    {
        ctx = default;
        if (!TryRobotGoo(da, index, out var goo)) return false;
        ctx = RobotContext.FromGoo(goo);
        return true;
    }

    public static bool TryTrajectoryGoo(IGH_DataAccess da, int index, out TrajectoryGoo goo)
    {
        goo = null!;
        TrajectoryGoo? g = null;
        if (!da.GetData(index, ref g) || g?.Value is null) return false;
        goo = g;
        return true;
    }

    public static bool TryTrajectory(IGH_DataAccess da, int index, out Trajectory trajectory)
    {
        trajectory = null!;
        if (!TryTrajectoryGoo(da, index, out var goo)) return false;
        trajectory = goo.Value;
        return true;
    }

    public static bool TryGoal(IGH_DataAccess da, int index, out JointState? joints, out Plane? plane)
    {
        joints = null;
        plane = null;
        IGH_Goo? goo = null;
        if (!da.GetData(index, ref goo) || goo is null) return false;
        if (goo is JointStateGoo js && js.Value is not null) { joints = js.Value; return true; }
        if (goo.CastTo<Plane>(out var pl)) { plane = pl; return true; }
        return false;
    }

    public static bool TryGoals(
        IGH_DataAccess da,
        int index,
        out List<(JointState? joints, Plane? plane)> goals,
        out List<string> errors)
    {
        goals = new List<(JointState? joints, Plane? plane)>();
        errors = new List<string>();
        var rawGoals = new List<IGH_Goo>();
        if (!da.GetDataList(index, rawGoals) || rawGoals.Count == 0)
        {
            errors.Add("Goal list is empty.");
            return false;
        }

        for (var i = 0; i < rawGoals.Count; i++)
        {
            var goo = rawGoals[i];
            if (goo is null)
            {
                errors.Add($"Goal[{i}] is null. Provide a Plane or a Joint State.");
                continue;
            }

            if (goo is JointStateGoo js && js.Value is not null)
            {
                goals.Add((js.Value, null));
                continue;
            }

            if (goo.CastTo<Plane>(out var plane))
            {
                goals.Add((null, plane));
                continue;
            }

            errors.Add($"Goal[{i}] is not supported. Provide a Plane or a Joint State.");
        }

        return goals.Count > 0;
    }

    public static JointState StartOrHome(IGH_DataAccess da, int index, RobotModel robot)
    {
        JointStateGoo? goo = null;
        if (da.GetData(index, ref goo) && goo?.Value is not null) return goo.Value;
        return HomePoseLookup.HomeOrZeros(robot);
    }

    public static bool TryCollisionObject(IGH_Goo goo, out CollisionObject obj)
    {
        obj = null!;
        if (goo is CollisionObjectGoo cog && cog.Value is not null)
        {
            obj = cog.Value;
            return true;
        }
        return false;
    }

    public static CollisionScene? OptionalCollisionScene(IGH_DataAccess da, int index)
    {
        CollisionSceneGoo? goo = null;
        if (!da.GetData(index, ref goo) || goo?.Value is null) return null;
        return goo.Value;
    }

    public static PlanningContext BuildPlanningContext(
        RobotModel robot,
        IGH_DataAccess da,
        int collisionIndex,
        int groupIndex,
        int attachIndex)
    {
        var scene = OptionalCollisionScene(da, collisionIndex);
        var planningContext = PlanningContext.Create(robot, scene);
        PlanningGroupGoo? groupGoo = null;
        if (da.GetData(groupIndex, ref groupGoo) && groupGoo?.Value is not null)
            planningContext = planningContext.ForGroup(groupGoo.Value);

        var attachedBodies = new List<AttachedBodyGoo>();
        if (da.GetDataList(attachIndex, attachedBodies))
        {
            foreach (var body in attachedBodies)
            {
                if (body.Value is not null)
                    planningContext = planningContext.Attach(body.Value);
            }
        }

        return planningContext;
    }

    public static bool TryMotionSegments(
        IGH_DataAccess da,
        int index,
        out List<MotionSegment> segments,
        out List<string> errors)
    {
        segments = new List<MotionSegment>();
        errors = new List<string>();
        var raw = new List<MotionSegmentGoo>();
        if (!da.GetDataList(index, raw) || raw.Count == 0)
        {
            errors.Add("Segments list is empty.");
            return false;
        }

        for (var i = 0; i < raw.Count; i++)
        {
            var goo = raw[i];
            if (goo?.Value is null)
            {
                errors.Add($"Segment[{i}] is null.");
                continue;
            }

            segments.Add(goo.Value);
        }

        return segments.Count > 0;
    }

    public enum PlanStatusKind
    {
        Manual,
        ManualCached,
        Auto,
        AutoCached,
        Planning
    }

    public static string BuildStatusMessage(IReadOnlyList<PlanningResult>? results, PlanStatusKind kind)
    {
        if (kind == PlanStatusKind.Planning)
            return results is { Count: > 0 } ? "Planning…" : "Planning… (press Replan to start).";
        if (kind == PlanStatusKind.Manual && results is null)
            return "Press Plan to compute.";

        if (results is null || results.Count == 0)
        {
            return kind switch
            {
                PlanStatusKind.ManualCached => "No results (cached).",
                PlanStatusKind.AutoCached => "No results (auto, cached).",
                _ => "No results."
            };
        }

        var successCount = results.Count(r => r.Success);
        var suffix = kind switch
        {
            PlanStatusKind.ManualCached => " (cached).",
            PlanStatusKind.Auto => " (auto).",
            PlanStatusKind.AutoCached => " (auto, cached).",
            _ => "."
        };
        if (successCount == results.Count) return $"Success {successCount}/{results.Count}{suffix}";

        var failedDetails = results
            .Select((result, index) => (result, index))
            .Where(x => !x.result.Success)
            .Select(x => $"Goal[{x.index}]: {(x.result.Errors.Count > 0 ? string.Join("; ", x.result.Errors) : "Failed.")}");
        return $"Success {successCount}/{results.Count}{suffix} {string.Join(" | ", failedDetails)}";
    }

    public static string BuildStatusMessage(IReadOnlyList<PlanningResult> results, bool cached) =>
        BuildStatusMessage(results, cached ? PlanStatusKind.ManualCached : PlanStatusKind.Manual);

    public static List<string> BuildWarnings(IReadOnlyList<PlanningResult> results)
    {
        var warnings = new List<string>();
        foreach (var pair in results.Select((result, index) => (result, index)))
        {
            foreach (var warning in pair.result.Warnings)
                warnings.Add($"Goal[{pair.index}]: {warning}");
        }

        var capabilities = MotusCapabilities.Describe();
        if (!warnings.Any(w => string.Equals(w, capabilities, StringComparison.OrdinalIgnoreCase)))
            warnings.Add(capabilities);
        return warnings;
    }

    public static string BuildProgramStatusMessage(PlanningResult result, bool cached)
    {
        var suffix = cached ? " (cached)." : ".";
        if (result.Success) return $"Success{suffix}";
        var detail = result.Errors.Count > 0 ? string.Join("; ", result.Errors) : "Failed.";
        return $"Failed{suffix} {detail}";
    }

    public static List<string> BuildProgramWarnings(PlanningResult result)
    {
        var warnings = result.Warnings.ToList();
        var capabilities = MotusCapabilities.Describe();
        if (!warnings.Any(w => string.Equals(w, capabilities, StringComparison.OrdinalIgnoreCase)))
            warnings.Add(capabilities);
        return warnings;
    }

    public static PlanningOptions BuildOptions(
        RobotModel robot,
        SerialJointChain? chain,
        double maxStep,
        CollisionScene? scene,
        IReadOnlyList<AttachedBody>? attached = null) =>
        new()
        {
            MaxJointStepRadians = maxStep,
            CollisionScene = scene,
            CollisionChecker = scene is not null || (attached?.Count ?? 0) > 0
                ? TryCollisionChecker(robot, chain, scene, attached)
                : null,
            AttachedBodies = attached
        };

    public static ICollisionChecker? TryCollisionChecker(
        RobotModel robot,
        SerialJointChain? chain = null,
        CollisionScene? scene = null,
        IReadOnlyList<AttachedBody>? attached = null)
    {
        try
        {
            if (!KinematicsResolver.SupportsModel(robot.Preset, chain)) return null;
            return CollisionCheckerFactory.Create(robot, chain, attached);
        }
        catch (InvalidOperationException)
        {
            return null;
        }
    }
}
