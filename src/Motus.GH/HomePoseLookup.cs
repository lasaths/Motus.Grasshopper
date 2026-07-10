using Motus.Core;
using Motus.Presets;

namespace Motus.GH;

internal static class HomePoseLookup
{
    public static Task PreloadAsync() => Task.CompletedTask;

    public static JointState HomeOrZeros(RobotModel robot) => HomePoseResolver.HomeOrZeros(robot);
}
