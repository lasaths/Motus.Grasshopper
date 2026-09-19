using Motus.Core;

namespace Motus.GH.Rhino;

public static class JointCoordinateInput
{
    public static JointState Create(IReadOnlyList<double> values, bool degrees, RobotModel? robot = null)
    {
        if (values.Count == 0 || values.Any(v => !double.IsFinite(v)))
            throw new ArgumentException("Joint coordinates must be non-empty and finite.");
        if (robot is not null && values.Count != robot.Preset.AxisCount)
            throw new ArgumentException($"Expected {robot.Preset.AxisCount} joint coordinates, got {values.Count}.");
        var q = values.Select((v, i) => degrees &&
            (robot is null || robot.Preset.JointLimits[i].Unit == JointCoordinateUnit.Radians)
                ? v * Math.PI / 180 : v).ToArray();
        var state = new JointState(q);
        if (robot is not null && state.Validate(robot.Preset.JointLimits) is { IsValid: false } validation)
            throw new ArgumentException(string.Join("; ", validation.Errors));
        return state;
    }
}
