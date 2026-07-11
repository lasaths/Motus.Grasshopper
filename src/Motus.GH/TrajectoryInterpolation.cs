using Motus.Core;

namespace Motus.GH;

internal static class TrajectoryInterpolation
{
    public static JointState AtTime(Trajectory trajectory, double elapsedSeconds, out int segmentIndex) =>
        TrajectorySampler.AtTime(trajectory, elapsedSeconds, out segmentIndex);

    public static EndEffectorState? AtTimeToolState(Trajectory trajectory, double elapsedSeconds) =>
        TrajectorySampler.AtTimeToolState(trajectory, elapsedSeconds);
}
