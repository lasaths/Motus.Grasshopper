using Motus.Core;

namespace Motus.GH.Rhino;

/// <summary>ponytail: thin alias — logic lives in Motus.NET <see cref="ToolParameterBinding"/>.</summary>
internal static class Robotiq2F85Kinematics
{
    public const double OpenWidthMeters = ToolParameterBinding.Robotiq2F85OpenWidthMeters;
    public const double ClosedDriverRadians = ToolParameterBinding.Robotiq2F85ClosedDriverRadians;

    public static double DriverAngleRadians(double widthMeters, double openWidthMeters = OpenWidthMeters) =>
        ToolParameterBinding.Robotiq2F85DriverAngleRadians(widthMeters, openWidthMeters);
}
