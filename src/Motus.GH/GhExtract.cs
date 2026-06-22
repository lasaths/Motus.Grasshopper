using Grasshopper.Kernel;
using Motus.Core;
using Motus.GH.Data;

namespace Motus.GH;

internal static class GhExtract
{
    public static bool TryRobot(IGH_DataAccess da, int index, out RobotModel robot)
    {
        robot = null!;
        RobotModelGoo? goo = null;
        if (!da.GetData(index, ref goo) || goo?.Value is null) return false;
        robot = goo.Value;
        return true;
    }

    public static bool TryJointState(IGH_DataAccess da, int index, out JointState state)
    {
        state = null!;
        JointStateGoo? goo = null;
        if (!da.GetData(index, ref goo) || goo?.Value is null) return false;
        state = goo.Value;
        return true;
    }

    public static bool TryTrajectory(IGH_DataAccess da, int index, out Trajectory trajectory)
    {
        trajectory = null!;
        TrajectoryGoo? goo = null;
        if (!da.GetData(index, ref goo) || goo?.Value is null) return false;
        trajectory = goo.Value;
        return true;
    }

    public static CollisionScene? OptionalCollisionScene(IGH_DataAccess da, int index)
    {
        CollisionSceneGoo? goo = null;
        if (!da.GetData(index, ref goo) || goo?.Value is null) return null;
        return goo.Value;
    }

    public static BaseFrame? OptionalBaseFrame(IGH_DataAccess da, int index)
    {
        BaseFrameGoo? goo = null;
        if (!da.GetData(index, ref goo) || goo?.Value is null) return null;
        return goo.Value;
    }

    public static ToolFrame? OptionalToolFrame(IGH_DataAccess da, int index)
    {
        ToolFrameGoo? goo = null;
        if (!da.GetData(index, ref goo) || goo?.Value is null) return null;
        return goo.Value;
    }

    public static PlanningOptions BuildOptions(double maxStep, CollisionScene? scene) =>
        new() { MaxJointStepRadians = maxStep, CollisionScene = scene };
}
