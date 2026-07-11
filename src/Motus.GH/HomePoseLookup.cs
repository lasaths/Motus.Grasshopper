using Motus.Core;

namespace Motus.GH;

internal static class HomePoseLookup
{
  // UR10e "ready" pose (matches former viewer_presets ur10e defaultPose).
  private static readonly double[] Ur10eReadyRadians =
  [
    0,
    -Math.PI / 2,
    Math.PI / 2,
    -Math.PI / 2,
    0,
    0
  ];

  public static JointState HomeOrZeros(RobotModel robot, string? sourcePath = null)
  {
    if (IsUr10e(robot, sourcePath))
      return new JointState((double[])Ur10eReadyRadians.Clone());
    return new JointState(new double[robot.Preset.AxisCount]);
  }

  internal static bool IsUr10e(RobotModel robot, string? sourcePath = null)
  {
    if (robot.Preset.AxisCount != 6) return false;

    var name = robot.Preset.ModelName ?? robot.DisplayName ?? "";
    if (name.Contains("ur10e", StringComparison.OrdinalIgnoreCase)) return true;
    if (sourcePath?.Contains("ur10e", StringComparison.OrdinalIgnoreCase) == true) return true;

    var names = robot.JointNames;
    if (names is null || names.Count != 6) return false;
    return names.Any(j =>
      j.Contains("shoulder_pan", StringComparison.OrdinalIgnoreCase) ||
      j.Contains("shoulder_lift", StringComparison.OrdinalIgnoreCase));
  }
}
