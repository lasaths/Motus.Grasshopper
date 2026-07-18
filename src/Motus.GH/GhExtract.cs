using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Motus.Core;
using Motus.Geometry;
using Motus.GH.Data;
using Motus.GH.Planning;
using Motus.GH.Rhino;
using Motus.GH.Urdf;
using Motus.OMPL.NET;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;

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

    public static bool TryGoal(IGH_DataAccess da, int index, out JointState? joints, out Plane? plane)
    {
        joints = null;
        plane = null;
        if (index < 0) return false;
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

    public static JointState StartOrHome(IGH_DataAccess da, int index, RobotModel robot) =>
        StartOrHome(da, index, robot, out _);

    public static JointState StartOrHome(
        IGH_DataAccess da,
        int index,
        RobotModel robot,
        out bool usedDefaultHome)
    {
        usedDefaultHome = true;
        if (index < 0)
            return HomePoseLookup.HomeOrZeros(robot);

        JointStateGoo? goo = null;
        if (da.GetData(index, ref goo) && goo?.Value is not null)
        {
            usedDefaultHome = false;
            return goo.Value;
        }

        return HomePoseLookup.HomeOrZeros(robot);
    }

    public static void RemarkIfDefaultStart(GH_Component owner, bool usedDefaultHome)
    {
        if (usedDefaultHome)
        {
            owner.AddRuntimeMessage(
                GH_RuntimeMessageLevel.Remark,
                "Start not wired — using home/zeros.");
        }
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

    internal readonly struct CollisionInputParse
    {
        public CollisionScene? Scene { get; init; }
        public bool Wired { get; init; }
        public string? Error { get; init; }
        public string? Warning { get; init; }
    }

    public static CollisionInputParse ParseCollisionInput(IGH_DataAccess da, int index)
    {
        if (index < 0)
            return new CollisionInputParse { Wired = false };

        IGH_Goo? goo = null;
        if (!da.GetData(index, ref goo) || goo is null)
            return new CollisionInputParse { Wired = false };

        if (goo is CollisionSceneGoo sceneGoo && sceneGoo.Value is { } scene)
        {
            if (scene.Objects.Count == 0)
            {
                return new CollisionInputParse
                {
                    Wired = true,
                    Scene = scene,
                    Warning = "Collision scene is empty — ColScene received no valid collision objects."
                };
            }

            return new CollisionInputParse { Wired = true, Scene = scene };
        }

        if (TryCollisionObject(goo, out var obj))
        {
            return new CollisionInputParse
            {
                Wired = true,
                Scene = new CollisionScene(new[] { obj }),
                Warning = "Single collision object wired to Plan — use ColScene when merging multiple obstacles."
            };
        }

        if (TryCollisionObjectFromGeometry(goo, out obj))
        {
            return new CollisionInputParse
            {
                Wired = true,
                Scene = new CollisionScene(new[] { obj! }),
                Warning = "Raw mesh/Brep wired to Collision — prefer ColMesh → ColScene → Plan for explicit control."
            };
        }

        return new CollisionInputParse
        {
            Wired = true,
            Error = "Collision input not recognized. Wire ColScene (ColMesh/ColSphere → ColScene → Plan.Collision)."
        };
    }

    public static CollisionScene? OptionalCollisionScene(IGH_DataAccess da, int index) =>
        ParseCollisionInput(da, index).Scene;

    // Cache raw Mesh/Brep → CollisionObject so Plan idle solves don't remesh every tick.
    private static readonly ConditionalWeakTable<object, CollisionObject> GeometryCollisionCache = new();

    private static bool TryCollisionObjectFromGeometry(IGH_Goo goo, out CollisionObject? obj)
    {
        obj = null;
        if (goo is not IGH_GeometricGoo) return false;

        object? key = null;
        if (goo is GH_Mesh ghm && ghm.Value is { IsValid: true } mesh)
            key = mesh;
        else if (goo is GH_Brep ghb && ghb.Value is { IsValid: true } brep)
            key = brep;
        if (key is null) return false;

        if (GeometryCollisionCache.TryGetValue(key, out var cached))
        {
            obj = cached;
            return true;
        }

        if (key is Mesh m)
            obj = CollisionMeshBuilder.FromMesh(m, Plane.WorldXY, "obstacle");
        else if (key is Brep b)
            obj = CollisionMeshBuilder.FromBrep(b, Plane.WorldXY, "obstacle");

        if (obj is null) return false;
        GeometryCollisionCache.Add(key, obj);
        return true;
    }

    public static PlanningContext BuildPlanningContext(
        RobotModel robot,
        IGH_DataAccess da,
        int collisionIndex,
        int groupIndex,
        int attachIndex,
        CollisionScene? collisionScene = null)
    {
        var scene = collisionScene ?? OptionalCollisionScene(da, collisionIndex);
        var planningContext = PlanningContext.Create(robot, scene);
        PlanningGroupGoo? groupGoo = null;
        if (groupIndex >= 0 && da.GetData(groupIndex, ref groupGoo) && groupGoo?.Value is not null)
            planningContext = planningContext.ForGroup(groupGoo.Value);

        if (attachIndex >= 0)
        {
            var attachedBodies = new List<AttachedBodyGoo>();
            if (da.GetDataList(attachIndex, attachedBodies))
            {
                foreach (var body in attachedBodies)
                {
                    if (body.Value is not null)
                        planningContext = planningContext.Attach(body.Value);
                }
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

    public static string BuildStatusMessage(IReadOnlyList<PlanningResult>? results, PlanStatusKind kind, string? activity = null)
    {
        if (kind == PlanStatusKind.Planning)
            return activity ?? "Planning…";
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
            .Select(x => $"Goal[{x.index}]: {FormatFailure(x.result)}");
        return $"Success {successCount}/{results.Count}{suffix} {string.Join(" | ", failedDetails)}";
    }

    public static string BuildStatusMessage(IReadOnlyList<PlanningResult> results, bool cached) =>
        BuildStatusMessage(results, cached ? PlanStatusKind.ManualCached : PlanStatusKind.Manual);

    public static List<string> BuildWarnings(IReadOnlyList<PlanningResult> results)
    {
        var warnings = new List<string>();
        foreach (var pair in results.Select((result, index) => (result, index)))
        {
            foreach (var warning in ExtractWarnings(pair.result))
                warnings.Add($"Goal[{pair.index}]: {warning}");
        }

        var capabilities = MotusCapabilities.Describe();
        if (!warnings.Any(w => string.Equals(w, capabilities, StringComparison.OrdinalIgnoreCase)))
            warnings.Add(capabilities);
        return warnings;
    }

    /// <summary>Joint goals with obstacles or attached bodies route through the sampling planner.</summary>
    public static bool GoalsNeedSamplingPlanner(
        IReadOnlyList<(JointState? joints, Plane? plane)> goals,
        PlanningContext context) =>
        goals.Any(g => g.joints is not null) &&
        (PlanningCollision.SceneHasObstacles(context.Scene) || context.Attached.Count > 0);

    public static string DescribePlanningActivity(
        IReadOnlyList<(JointState? joints, Plane? plane)> goals,
        PlanningContext context,
        bool collisionWired,
        RrtPlanSettings? rrtSettings = null)
    {
        rrtSettings ??= RrtPlanSettings.Defaults;
        var rrt = rrtSettings.Value;
        if (goals.Count == 0) return "Planning…";

        var parts = new List<string>(goals.Count);
        for (var i = 0; i < goals.Count; i++)
        {
            var goal = goals[i];
            if (goal.plane is not null)
            {
                var collision = PlanningCollision.SceneHasObstacles(context.Scene) || context.Attached.Count > 0;
                parts.Add(collision
                    ? $"Goal[{i}]: TCP-LIN + link collision check"
                    : $"Goal[{i}]: TCP-LIN");
                continue;
            }

            if (PlanningCollision.SceneHasObstacles(context.Scene) || context.Attached.Count > 0)
            {
                var obstacleCount = context.Scene.Objects.Count;
                var attachCount = context.Attached.Count;
                var scene = obstacleCount > 0
                    ? $"{obstacleCount} obstacle{(obstacleCount == 1 ? "" : "s")}"
                    : attachCount > 0
                        ? $"{attachCount} attached body{(attachCount == 1 ? "" : "ies")}"
                        : "collision scene";
                parts.Add($"Goal[{i}]: {rrt.PlannerLabel} ({scene})");
                continue;
            }

            parts.Add($"Goal[{i}]: joint-linear");
        }

        return "Planning… " + string.Join("; ", parts);
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
        var warnings = ExtractWarnings(result).ToList();
        var capabilities = MotusCapabilities.Describe();
        if (!warnings.Any(w => string.Equals(w, capabilities, StringComparison.OrdinalIgnoreCase)))
            warnings.Add(capabilities);
        return warnings;
    }

    private static string FormatFailure(PlanningResult result)
    {
        var codedErrors = result.Messages
            .Where(m => m.Severity == PlanningMessageSeverity.Error)
            .Select(m => $"{m.Code}: {m.Message}")
            .ToList();
        if (codedErrors.Count > 0)
            return string.Join("; ", codedErrors);
        return result.Errors.Count > 0 ? string.Join("; ", result.Errors) : "Failed.";
    }

    private static IEnumerable<string> ExtractWarnings(PlanningResult result)
    {
        var codedWarnings = result.Messages
            .Where(m => m.Severity == PlanningMessageSeverity.Warning)
            .Select(m => $"{m.Code}: {m.Message}")
            .ToList();
        return codedWarnings.Count > 0 ? codedWarnings : result.Warnings;
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
            return CollisionCheckerFactory.GetOrCreate(robot, chain, attached, scene);
        }
        catch (InvalidOperationException)
        {
            return null;
        }
    }

    /// <summary>Fast-fail before expensive planners when start/goal already intersect obstacles.</summary>
    public static PlanningResult? TryPreflightCollision(
        RobotContext ctx,
        PlanningContext planningContext,
        JointState start,
        (JointState? joints, Plane? plane) goal,
        ICollisionChecker? checker = null)
    {
        if (!PlanningCollision.SceneHasObstacles(planningContext.Scene) && planningContext.Attached.Count == 0)
            return null;

        checker ??= TryCollisionChecker(ctx.EffectiveModel, ctx.Chain, planningContext.Scene, planningContext.Attached);
        if (checker is null)
            return null;

        JointState? goalState = goal.joints;
        if (goal.plane is { } plane)
        {
            var reach = CartesianGoalSolver.TryReachFromStart(
                ctx.EffectiveModel,
                new CartesianPose(FrameConversion.FromPlane(plane)),
                start,
                ctx.Chain);
            if (!reach.Success)
                return null;
            goalState = reach.Solution;
        }

        if (goalState is null)
            return null;

        return PlanningCollision.ValidateEndpoints(
            start,
            goalState,
            planningContext.Scene,
            checker,
            planningContext.Attached.Count > 0);
    }

    public static RrtPlanSettings ResolveRrtSettings(IGH_DataAccess da, int index, GH_Component? owner = null)
    {
        if (index < 0)
            return RrtPlanSettings.Defaults;

        var goo = default(RrtPlanSettingsGoo);
        if (!da.GetData(index, ref goo) || goo is null)
            return RrtPlanSettings.Defaults;

        if (!goo.IsValid)
        {
            owner?.AddRuntimeMessage(
                GH_RuntimeMessageLevel.Warning,
                "RRT Settings input is invalid — using Motus RRT Settings defaults.");
            return RrtPlanSettings.Defaults;
        }

        var settings = goo.Value.CoerceAvailable(out var fallbackReason);
        if (fallbackReason is not null)
        {
            owner?.AddRuntimeMessage(
                GH_RuntimeMessageLevel.Warning,
                $"{fallbackReason} Using RRT-Connect.");
        }

        return settings;
    }
}
