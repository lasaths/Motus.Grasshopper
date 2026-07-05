using Grasshopper.Kernel;

using Motus.Core;

using Motus.Geometry;

using Motus.GH;

using Motus.GH.Data;

using Motus.GH.UI;

using Motus.OMPL.NET;

using Motus.Rhino;

using Rhino.Geometry;



namespace Motus.GH.Components;



public sealed class MotusPlanComponent : MotusComponentBase

{

    private const double MaxJointStep = 0.05;

    private const double LinStepMeters = 0.005;

    private PlanningResult? _cached;

    private TrajectoryGoo? _cachedGoo;

    private bool _run;



    public MotusPlanComponent() : base("Motus Plan", "Plan", "Plan motion to a plane (TCP LIN) or joint goal; click Plan", "Plan", "flow-arrow") { }



    public override void CreateAttributes() =>

        m_attributes = new ButtonAttributes(this, () => "Plan", () => false, RequestRun);



    protected override void RegisterInputParams(GH_InputParamManager p)

    {

        p.AddGenericParameter("Robot", "Rb", "Robot model", GH_ParamAccess.item);

        p.AddGenericParameter("Goal", "G", "Target as a Plane (TCP LIN) or a Joint State", GH_ParamAccess.item);

        p.AddGenericParameter("Start", "S", "Start joint state (defaults to home from viewer_presets or zeros)", GH_ParamAccess.item);

        p[p.ParamCount - 1].Optional = true;

        p.AddGenericParameter("Collision", "C", "Collision scene; joint goals use RRT-Connect; plane goals validate LIN against scene", GH_ParamAccess.item);

        p[p.ParamCount - 1].Optional = true;

    }



    protected override void RegisterOutputParams(GH_OutputParamManager p)

    {

        p.AddGenericParameter("Trajectory", "T", "Planned trajectory", GH_ParamAccess.item);

        p.AddTextParameter("Status", "St", "Status message", GH_ParamAccess.item);

        p.AddTextParameter("Warnings", "W", "Warnings", GH_ParamAccess.list);

    }



    private void RequestRun()

    {

        _run = true;

        ExpireSolution(true);

    }



    private static TrajectoryGoo TrajectoryFrom(RobotModelGoo robotGoo, Trajectory trajectory) =>

        new(trajectory)

        {

            Chain = robotGoo.Chain,

            BaseFrameOverride = robotGoo.BaseFrameOverride,

            ToolFrameOverride = robotGoo.ToolFrameOverride

        };



    protected override void SolveInstance(IGH_DataAccess da)

    {

        if (!GhExtract.TryRobotGoo(da, 0, out var robotGoo)) return;

        var ctx = RobotContext.FromGoo(robotGoo);



        if (!_run)

        {

            if (_cached?.Success == true && _cachedGoo is not null) da.SetData(0, _cachedGoo);

            da.SetData(1, _cached is null ? "Press Plan to compute."

                : _cached.Success ? "Success (cached)." : string.Join("; ", _cached.Errors));

            if (_cached is not null) da.SetDataList(2, _cached.Warnings);

            return;

        }

        _run = false;



        if (!GhExtract.TryGoal(da, 1, out var jointGoal, out var planeGoal))

        {

            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Goal must be a Plane or a Joint State.");

            return;

        }

        var start = GhExtract.StartOrHome(da, 2, ctx.Model);

        var scene = GhExtract.OptionalCollisionScene(da, 3);



        _cached = planeGoal is { } plane

            ? PlanCartesianLin(ctx, start, plane, scene)

            : scene is not null

                ? PlanRrt(ctx, start, jointGoal!, scene)

                : new JointLinearPlanner().Plan(new PlanningRequest(ctx.Model, start, jointGoal!, GhExtract.BuildOptions(ctx.Model, ctx.Chain, MaxJointStep, scene)));



        if (_cached.Success)

        {

            _cachedGoo = TrajectoryFrom(robotGoo, _cached.Trajectory!);

            da.SetData(0, _cachedGoo);

        }

        da.SetData(1, _cached.Success ? "Success." : string.Join("; ", _cached.Errors));

        da.SetDataList(2, _cached.Warnings);

    }



    private static PlanningResult PlanCartesianLin(RobotContext ctx, JointState start, Plane plane, CollisionScene? scene)

    {

        var goal = new CartesianPose(FrameConversion.FromPlane(plane));

        var req = new CartesianPlanningRequest(ctx.Model, start, goal, GhExtract.BuildOptions(ctx.Model, ctx.Chain, MaxJointStep, scene), scene);

        return new CartesianLinearPathPlanner(ctx.Model.Preset, ctx.Chain).PlanToResult(req, LinStepMeters);

    }



    private PlanningResult PlanRrt(RobotContext ctx, JointState start, JointState goal, CollisionScene scene)

    {

        var checker = GhExtract.TryCollisionChecker(ctx.Model, ctx.Chain, scene);

        if (checker is null)

            return PlanningResult.Failed(new[] { "No collision checker available for this robot model." });

        var opts = new RrtConnectOptions { MaxIterations = 4000, RandomSeed = 42, ShouldCancel = () => OnPingDocument() is null };

        var req = new PlanningRequest(ctx.Model, start, goal, new PlanningOptions { CollisionScene = scene, CollisionChecker = checker });

        return new RrtConnectPlanner(checker, opts).Plan(req);

    }



    public override Guid ComponentGuid => new Guid("8bb0bae3-527f-4e80-a8a4-c8a88b7276de");

}


