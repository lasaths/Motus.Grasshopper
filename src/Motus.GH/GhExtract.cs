using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Motus.Core;
using Motus.GH.Data;
using Rhino.Geometry;

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

    /// <summary>Reads a goal that is either a Joint State or a Rhino Plane (Cartesian target).</summary>
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

    /// <summary>Optional start joint state; falls back to the robot home (all zeros).</summary>
    public static JointState StartOrHome(IGH_DataAccess da, int index, RobotModel robot)
    {
        JointStateGoo? goo = null;
        if (da.GetData(index, ref goo) && goo?.Value is not null) return goo.Value;
        return new JointState(new double[robot.Preset.AxisCount]);
    }

    public static CollisionScene? OptionalCollisionScene(IGH_DataAccess da, int index)
    {
        CollisionSceneGoo? goo = null;
        if (!da.GetData(index, ref goo) || goo?.Value is null) return null;
        return goo.Value;
    }

    public static PlanningOptions BuildOptions(double maxStep, CollisionScene? scene) =>
        new() { MaxJointStepRadians = maxStep, CollisionScene = scene };
}
