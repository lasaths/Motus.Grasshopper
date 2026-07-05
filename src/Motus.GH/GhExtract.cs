using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Motus.Core;
using Motus.Geometry;
using Motus.GH.Data;
using Rhino.Geometry;

namespace Motus.GH;

internal static class GhExtract
{
    public static bool TryRobotGoo(IGH_DataAccess da, int index, out RobotModelGoo goo)
    {
        goo = null!;
        RobotModelGoo? g = null;
        if (!da.GetData(index, ref g) || g?.Value is null) return false;
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

    public static PlanningOptions BuildOptions(RobotModel robot, SerialJointChain? chain, double maxStep, CollisionScene? scene) =>
        new()
        {
            MaxJointStepRadians = maxStep,
            CollisionScene = scene,
            CollisionChecker = scene is not null ? TryCollisionChecker(robot, chain, scene) : null
        };

    public static ICollisionChecker? TryCollisionChecker(RobotModel robot, SerialJointChain? chain = null, CollisionScene? scene = null)
    {
        try
        {
            if (!KinematicsResolver.SupportsModel(robot.Preset, chain)) return null;
            if (robot.CollisionModel is not null)
                return new RobotMeshCollisionChecker(robot, chain);
            if (scene?.Objects.Any(o => o.Shape == CollisionShape.Mesh) == true)
            {
                if (chain is null)
                    return new MeshCollisionChecker(robot.Preset);
            }
            return new SphereCollisionChecker(KinematicsResolver.CreateFkSolver(robot.Preset, chain), robot.Preset.BaseFrame);
        }
        catch (InvalidOperationException)
        {
            return null;
        }
    }
}
