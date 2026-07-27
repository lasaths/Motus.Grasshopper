using Motus.Core;
using Motus.Geometry;
using Motus.GH.Rhino;
using Rhino.Geometry;
using NetLeggedGait = Motus.Geometry.LeggedGait;
using NetLeggedLayout = Motus.Geometry.LeggedLayout;

namespace Motus.GH.Planning;

/// <summary>
/// Thin Rhino adapter: Curve/Planes → polyline for Motus.NET <see cref="Motus.Geometry.LeggedGait"/>.
/// Algorithms and DOI provenance live in Motus.NET (<see cref="LeggedMethodRefs"/>).
/// </summary>
internal static class LeggedGaitRhino
{
    public sealed record Result(
        Trajectory Trajectory,
        IReadOnlyList<Frame> BasePath,
        Curve PathCurve,
        IReadOnlyList<Plane> PathPlanes,
        string? Warning,
        double MinStaticStabilityMarginMeters,
        string MethodProvenance);

    public static bool TryBuild(
        NetLeggedLayout layout,
        Curve? pathCurve,
        IReadOnlyList<Plane>? pathPlanes,
        double speed,
        double stepLength,
        double stepHeight,
        double hipStance,
        double femurStance,
        double tibiaStance,
        RobotModel model,
        out Result? result,
        out string error,
        NetLeggedGait.TerrainHeight? terrain = null)
    {
        result = null;
        if (!TryResolvePath(pathCurve, pathPlanes, out var curve, out error))
            return false;

        var poly = SamplePolyline(curve);
        if (!NetLeggedGait.TryBuild(
                layout, poly, speed, stepLength, stepHeight,
                hipStance, femurStance, tibiaStance, model,
                out var net, out error, terrain))
            return false;

        var planes = new List<Plane>();
        for (var i = 0; i < net!.BasePath.Count; i++)
        {
            if (i == 0 || i == net.BasePath.Count - 1 || i % 15 == 0)
                planes.Add(FrameConversion.ToPlanePlate(net.BasePath[i]));
        }

        result = new Result(
            net.Trajectory,
            net.BasePath,
            curve,
            planes,
            net.Warning,
            net.MinStaticStabilityMarginMeters,
            net.MethodProvenance);
        return true;
    }

    internal static double[] BuildStanceQ(NetLeggedLayout layout, double hip, double femur, double tibia) =>
        NetLeggedGait.BuildStanceQ(layout, hip, femur, tibia);

    private static List<Vec3> SamplePolyline(Curve curve)
    {
        var len = curve.GetLength();
        var spacing = Math.Clamp(len / 64.0, 0.01, 0.05);
        var n = Math.Max(2, (int)Math.Ceiling(len / spacing) + 1);
        var pts = new List<Vec3>(n);
        for (var i = 0; i < n; i++)
        {
            var s = len * i / (n - 1.0);
            if (!curve.LengthParameter(s, out var t))
                t = curve.Domain.ParameterAt(i / (n - 1.0));
            var p = curve.PointAt(t);
            pts.Add(new Vec3(p.X, p.Y, 0));
        }
        return pts;
    }

    private static bool TryResolvePath(Curve? pathCurve, IReadOnlyList<Plane>? pathPlanes, out Curve curve, out string error)
    {
        error = "";
        curve = null!;

        if (pathCurve is not null && pathCurve.IsValid && pathCurve.GetLength() > 1e-6)
        {
            curve = pathCurve.DuplicateCurve();
            return true;
        }

        if (pathPlanes is { Count: >= 2 })
        {
            var pts = new List<Point3d>(pathPlanes.Count);
            foreach (var pl in pathPlanes)
            {
                if (!pl.IsValid) continue;
                pts.Add(pl.Origin);
            }

            if (pts.Count < 2)
            {
                error = "Path / Planes: need ≥ 2 valid plane origins (m).";
                return false;
            }

            curve = pts.Count == 2
                ? new LineCurve(pts[0], pts[1])
                : Curve.CreateInterpolatedCurve(pts, 3) ?? new PolylineCurve(pts);
            return curve.IsValid;
        }

        error = "Path empty — wire a Curve or list of Planes (≥ 2 origins, m).";
        return false;
    }
}
