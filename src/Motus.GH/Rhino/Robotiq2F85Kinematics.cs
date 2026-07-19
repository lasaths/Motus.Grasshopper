namespace Motus.GH.Rhino;

/// <summary>Width → Robotiq 2F-85 driver joint q (Motus.NET TreeFK + URDF mimic own finger FK).</summary>
internal static class Robotiq2F85Kinematics
{
    public const double OpenWidthMeters = 0.085;
    public const double ClosedDriverRadians = 0.8;

    public static double DriverAngleRadians(double widthMeters, double openWidthMeters = OpenWidthMeters)
    {
        var open = openWidthMeters > 1e-9 ? openWidthMeters : OpenWidthMeters;
        var ratio = Math.Clamp(widthMeters / open, 0, 1);
        return (1.0 - ratio) * ClosedDriverRadians;
    }
}
