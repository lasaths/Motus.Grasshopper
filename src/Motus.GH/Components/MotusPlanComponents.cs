using Grasshopper.Kernel;
using Motus.Core;
using Motus.Geometry;
using Motus.GH;
using Motus.GH.Data;
using Motus.OMPL.NET;
using Motus.Rhino;
using Rhino.Geometry;

namespace Motus.GH.Components;

internal static class PlanCache
{
    public static bool TryIdle(bool run, bool autoReplan, PlanningResult? cached, IGH_DataAccess da, int trajOut, int statusOut, int warnOut)
    {
        if (run || autoReplan) return false;
        if (cached?.Success == true) da.SetData(trajOut, new TrajectoryGoo(cached.Trajectory!));
        da.SetData(statusOut, "Idle (set Run or AutoReplan).");
        return true;
    }

    public static void Emit(PlanningResult result, IGH_DataAccess da, int trajOut, int statusOut, int warnOut)
    {
        if (result.Success) da.SetData(trajOut, new TrajectoryGoo(result.Trajectory!));
        da.SetData(statusOut, result.Success ? "Success." : string.Join("; ", result.Errors));
        da.SetDataList(warnOut, result.Warnings);
    }
}

public sealed class MotusPlanJointPathComponent : MotusComponentBase
{
    private PlanningResult? _cached;
    private string _cacheKey = "";

    public MotusPlanJointPathComponent() : base("Motus Plan Joint Path", "Plan", "Joint-space linear plan", "Plan", "flow-arrow") { }
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddBooleanParameter("Run", "R", "Plan when true", GH_ParamAccess.item, false);
        p.AddBooleanParameter("AutoReplan", "A", "Replan on every solution", GH_ParamAccess.item, false);
        p.AddGenericParameter("Robot", "Rb", "Robot model", GH_ParamAccess.item);
        p.AddGenericParameter("Start", "S", "Start joint state", GH_ParamAccess.item);
        p.AddGenericParameter("Goal", "G", "Goal joint state", GH_ParamAccess.item);
        p.AddNumberParameter("MaxStep", "St", "Max joint step (rad)", GH_ParamAccess.item, 0.05);
        p.AddGenericParameter("Collision", "C", "Collision scene (optional)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddGenericParameter("Trajectory", "T", "Planned trajectory", GH_ParamAccess.item);
        p.AddTextParameter("Status", "St", "Status message", GH_ParamAccess.item);
        p.AddTextParameter("Warnings", "W", "Warnings", GH_ParamAccess.list);
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        var run = false;
        var autoReplan = false;
        if (!da.GetData(0, ref run)) return;
        da.GetData(1, ref autoReplan);
        if (PlanCache.TryIdle(run, autoReplan, _cached, da, 0, 1, 2)) return;

        if (!GhExtract.TryRobot(da, 2, out var robot) || !GhExtract.TryJointState(da, 3, out var start) || !GhExtract.TryJointState(da, 4, out var goal)) return;
        var maxStep = 0.05;
        da.GetData(5, ref maxStep);
        var scene = GhExtract.OptionalCollisionScene(da, 6);

        var key = $"{robot.DisplayName}|{string.Join(",", start.Positions)}|{string.Join(",", goal.Positions)}|{maxStep}|{scene?.Objects.Count}";
        if (!autoReplan && key == _cacheKey && _cached is not null)
        {
            PlanCache.Emit(_cached, da, 0, 1, 2);
            da.SetData(1, _cached.Success ? "Success (cached)." : string.Join("; ", _cached.Errors));
            return;
        }

        var req = new PlanningRequest(robot, start, goal, GhExtract.BuildOptions(maxStep, scene));
        _cached = new JointLinearPlanner().Plan(req);
        _cacheKey = key;
        PlanCache.Emit(_cached, da, 0, 1, 2);
    }
    public override Guid ComponentGuid => new Guid("8bb0bae3-527f-4e80-a8a4-c8a88b7276de");
}

public sealed class MotusPlanCartesianPathComponent : MotusComponentBase
{
    private PlanningResult? _cached;
    private string _cacheKey = "";

    public MotusPlanCartesianPathComponent() : base("Motus Plan Cartesian Path", "PlanCart", "Cartesian goal via IK + joint path", "Plan", "compass-tool") { }
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddBooleanParameter("Run", "R", "Plan when true", GH_ParamAccess.item, false);
        p.AddBooleanParameter("AutoReplan", "A", "Replan on every solution", GH_ParamAccess.item, false);
        p.AddGenericParameter("Robot", "Rb", "Robot model", GH_ParamAccess.item);
        p.AddGenericParameter("Start", "S", "Start joint state", GH_ParamAccess.item);
        p.AddGenericParameter("Goal", "G", "Cartesian TCP goal", GH_ParamAccess.item);
        p.AddNumberParameter("MaxStep", "St", "Max joint step (rad)", GH_ParamAccess.item, 0.05);
        p.AddGenericParameter("Collision", "C", "Collision scene (optional)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddGenericParameter("Trajectory", "T", "Planned trajectory", GH_ParamAccess.item);
        p.AddTextParameter("Status", "St", "Status message", GH_ParamAccess.item);
        p.AddTextParameter("Warnings", "W", "Warnings", GH_ParamAccess.list);
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        var run = false;
        var autoReplan = false;
        if (!da.GetData(0, ref run)) return;
        da.GetData(1, ref autoReplan);
        if (PlanCache.TryIdle(run, autoReplan, _cached, da, 0, 1, 2)) return;

        if (!GhExtract.TryRobot(da, 2, out var robot) || !GhExtract.TryJointState(da, 3, out var start)) return;
        CartesianPoseGoo? goalGoo = null;
        if (!da.GetData(4, ref goalGoo)) return;
        var maxStep = 0.05;
        da.GetData(5, ref maxStep);
        var scene = GhExtract.OptionalCollisionScene(da, 6);

        var key = $"{robot.DisplayName}|cart|{string.Join(",", start.Positions)}|{goalGoo!.Value.Tcp}|{maxStep}";
        if (!autoReplan && key == _cacheKey && _cached is not null)
        {
            PlanCache.Emit(_cached, da, 0, 1, 2);
            da.SetData(1, _cached.Success ? "Success (cached)." : string.Join("; ", _cached.Errors));
            return;
        }

        var planner = new CartesianLinearPlanner(robot.Preset);
        var req = new CartesianPlanningRequest(robot, start, goalGoo.Value, GhExtract.BuildOptions(maxStep, scene), scene);
        _cached = planner.Plan(req);
        _cacheKey = key;
        PlanCache.Emit(_cached, da, 0, 1, 2);
    }
    public override Guid ComponentGuid => new Guid("a5b6c7d8-e9f0-4123-a456-789abcdef012");
}

public sealed class MotusPlanRrtConnectComponent : MotusComponentBase
{
    private PlanningResult? _cached;
    private string _cacheKey = "";

    public MotusPlanRrtConnectComponent() : base("Motus Plan RRT Connect", "RRT", "RRT-Connect with collision", "Plan", "graph") { }
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddBooleanParameter("Run", "R", "Plan when true", GH_ParamAccess.item, false);
        p.AddBooleanParameter("AutoReplan", "A", "Replan on every solution", GH_ParamAccess.item, false);
        p.AddGenericParameter("Robot", "Rb", "Robot model", GH_ParamAccess.item);
        p.AddGenericParameter("Start", "S", "Start joint state", GH_ParamAccess.item);
        p.AddGenericParameter("Goal", "G", "Goal joint state", GH_ParamAccess.item);
        p.AddGenericParameter("Collision", "C", "Collision scene", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddIntegerParameter("MaxIter", "I", "Max RRT iterations", GH_ParamAccess.item, 4000);
        p.AddIntegerParameter("Seed", "Sd", "Random seed", GH_ParamAccess.item, 42);
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddGenericParameter("Trajectory", "T", "Planned trajectory", GH_ParamAccess.item);
        p.AddTextParameter("Status", "St", "Status message", GH_ParamAccess.item);
        p.AddTextParameter("Warnings", "W", "Warnings", GH_ParamAccess.list);
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        var run = false;
        var autoReplan = false;
        if (!da.GetData(0, ref run)) return;
        da.GetData(1, ref autoReplan);
        if (PlanCache.TryIdle(run, autoReplan, _cached, da, 0, 1, 2)) return;

        if (!GhExtract.TryRobot(da, 2, out var robot) || !GhExtract.TryJointState(da, 3, out var start) || !GhExtract.TryJointState(da, 4, out var goal)) return;
        var scene = GhExtract.OptionalCollisionScene(da, 5) ?? new CollisionScene();
        var maxIter = 4000;
        var seed = 42;
        da.GetData(6, ref maxIter);
        da.GetData(7, ref seed);

        var key = $"{robot.DisplayName}|rrt|{string.Join(",", start.Positions)}|{string.Join(",", goal.Positions)}|{scene.Objects.Count}|{maxIter}|{seed}";
        if (!autoReplan && key == _cacheKey && _cached is not null)
        {
            PlanCache.Emit(_cached, da, 0, 1, 2);
            da.SetData(1, _cached.Success ? "Success (cached)." : string.Join("; ", _cached.Errors));
            return;
        }

        var opts = new RrtConnectOptions
        {
            MaxIterations = Math.Max(100, maxIter),
            RandomSeed = seed,
            ShouldCancel = () => OnPingDocument() is null
        };
        var req = new PlanningRequest(robot, start, goal, new PlanningOptions { CollisionScene = scene });
        _cached = new RrtConnectPlanner(robot.Preset, opts).Plan(req);
        _cacheKey = key;
        PlanCache.Emit(_cached, da, 0, 1, 2);
    }
    public override Guid ComponentGuid => new Guid("b6c7d8e9-f0a1-4234-b567-89abcdef0123");
}

public sealed class MotusValidateTrajectoryComponent : MotusComponentBase
{
    public MotusValidateTrajectoryComponent() : base("Motus Validate Trajectory", "Valid", "Validate trajectory", "Plan", "check-circle") { }
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Trajectory", "T", "Trajectory", GH_ParamAccess.item);
        p.AddGenericParameter("Collision", "C", "Collision scene (optional)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddBooleanParameter("CheckAccel", "A", "Check acceleration limits", GH_ParamAccess.item, true);
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddBooleanParameter("Valid", "V", "Is valid", GH_ParamAccess.item);
        p.AddTextParameter("Errors", "E", "Errors", GH_ParamAccess.list);
        p.AddTextParameter("Warnings", "W", "Warnings", GH_ParamAccess.list);
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!GhExtract.TryTrajectory(da, 0, out var trajectory)) return;
        var scene = GhExtract.OptionalCollisionScene(da, 1);
        var checkAccel = true;
        da.GetData(2, ref checkAccel);

        ICollisionChecker? checker = null;
        if (scene is not null && KinematicsProfiles.TryGet(trajectory.Robot.Preset, out _))
            checker = new SphereCollisionChecker(trajectory.Robot.Preset);

        var r = new TrajectoryValidator().Validate(trajectory, new TrajectoryValidationOptions
        {
            CollisionChecker = checker,
            CollisionScene = scene,
            CheckAcceleration = checkAccel
        });
        da.SetData(0, r.IsValid);
        da.SetDataList(1, r.Errors);
        da.SetDataList(2, r.Warnings);
    }
    public override Guid ComponentGuid => new Guid("81caa6c6-166e-4e58-8325-8c6df7270ce0");
}
