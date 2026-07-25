using Rhino.Geometry;

namespace Motus.GH.Planning;

/// <summary>
/// Analytic 3-DOF leg IK (coxa yaw + femur/tibia pitch in coxa vertical plane).
/// Matches TreeFK / WalkingHexPreview: q0 = coxa yaw in body XY (includes mount yaw φᵢ).
/// </summary>
internal static class WalkingHexLegIk
{
    public static bool TrySolve(
        Point3d hip,
        Point3d foot,
        double coxa,
        double femur,
        double tibia,
        out double q0,
        out double q1,
        out double q2)
    {
        q0 = q1 = q2 = 0;
        if (coxa <= 0 || femur <= 0 || tibia <= 0)
            return false;

        var v = foot - hip;
        if (!IsFinite(v))
            return false;

        q0 = Math.Atan2(v.Y, v.X);
        var u = new Vector3d(Math.Cos(q0), Math.Sin(q0), 0);
        var w = v - u * coxa;
        var x = Vector3d.Multiply(w, u);
        var z = w.Z;
        var d2 = x * x + z * z;
        if (d2 < 1e-14)
            return false;

        var d = Math.Sqrt(d2);
        var maxReach = femur + tibia;
        var minReach = Math.Abs(femur - tibia);
        if (d > maxReach + 1e-9 || d < minReach - 1e-9)
            return false;

        var cosKnee = (femur * femur + tibia * tibia - d2) / (2.0 * femur * tibia);
        cosKnee = Math.Clamp(cosKnee, -1.0, 1.0);
        q2 = Math.Acos(cosKnee) - Math.PI;

        var cosFemur = (femur * femur + d2 - tibia * tibia) / (2.0 * femur * d);
        cosFemur = Math.Clamp(cosFemur, -1.0, 1.0);
        q1 = Math.Atan2(z, x) - Math.Acos(cosFemur);

        return double.IsFinite(q0) && double.IsFinite(q1) && double.IsFinite(q2);
    }

    public static Point3d FootPosition(
        Point3d hip,
        double coxa,
        double femur,
        double tibia,
        double q0,
        double q1,
        double q2)
    {
        var coxaDir = new Vector3d(Math.Cos(q0), Math.Sin(q0), 0);
        var knee = hip + coxaDir * coxa;
        var femurDir = coxaDir * Math.Cos(q1) + Vector3d.ZAxis * Math.Sin(q1);
        if (!femurDir.Unitize()) femurDir = coxaDir;
        var ankle = knee + femurDir * femur;
        var tibiaDir = coxaDir * Math.Cos(q1 + q2) + Vector3d.ZAxis * Math.Sin(q1 + q2);
        if (!tibiaDir.Unitize()) tibiaDir = femurDir;
        return ankle + tibiaDir * tibia;
    }

    public static Point3d KneePosition(Point3d hip, double coxa, double femur, double q0, double q1)
    {
        var coxaDir = new Vector3d(Math.Cos(q0), Math.Sin(q0), 0);
        var knee = hip + coxaDir * coxa;
        var femurDir = coxaDir * Math.Cos(q1) + Vector3d.ZAxis * Math.Sin(q1);
        if (!femurDir.Unitize()) femurDir = coxaDir;
        return knee + femurDir * femur;
    }

    private static bool IsFinite(Vector3d v) =>
        double.IsFinite(v.X) && double.IsFinite(v.Y) && double.IsFinite(v.Z);
}
