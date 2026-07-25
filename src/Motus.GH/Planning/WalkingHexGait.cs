using Motus.Core;
using Motus.Geometry;
using Motus.GH.Rhino;
using Rhino.Geometry;

namespace Motus.GH.Planning;

/// <summary>
/// ponytail: heuristic tripod gait + SE(2) base path — not terrain IK or Motus.NET planner.
/// </summary>
internal static class WalkingHexGait
{
    /// <summary>Leg order (mithi-style labels).</summary>
    private static readonly string[] LegNames =
    [
        "right-middle", "right-front", "left-front",
        "left-middle", "left-back", "right-back",
    ];

    /// <summary>Tripod swing groups (mithi leg order).</summary>
    private static readonly int[][] TripodGroups =
    [
        [1, 3, 4], // RF, LM, LB swing first half-cycle
        [0, 2, 5], // RM, LF, RB swing second half-cycle
    ];

    public sealed record Result(
        Trajectory Trajectory,
        IReadOnlyList<Frame> BasePath,
        Curve PathCurve,
        IReadOnlyList<Plane> PathPlanes,
        string? Warning);

    public static bool TryBuild(
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
        out string error)
    {
        result = null;
        error = "";

        if (!double.IsFinite(speed) || speed <= 0)
        {
            error = "Speed must be finite and > 0 (m/s).";
            return false;
        }

        if (!double.IsFinite(stepLength) || stepLength <= 0)
        {
            error = "Step must be finite and > 0 (m).";
            return false;
        }

        if (!double.IsFinite(stepHeight) || stepHeight < 0)
        {
            error = "Lift must be finite and ≥ 0 (m).";
            return false;
        }

        if (!TryResolvePath(pathCurve, pathPlanes, out var curve, out error))
            return false;

        var pathLength = curve.GetLength();
        if (pathLength < 0.05)
        {
            error = $"Path too short ({pathLength:F3} m) — need ≥ 0.05 m for gait preview.";
            return false;
        }

        var stanceQ = BuildStanceQ(hipStance, femurStance, tibiaStance);
        var duration = pathLength / speed;
        const double sampleHz = 30.0;
        var dt = 1.0 / sampleHz;
        var sampleCount = Math.Max(2, (int)Math.Ceiling(duration / dt) + 1);

        var points = new List<TrajectoryPoint>(sampleCount);
        var basePath = new List<Frame>(sampleCount);
        var pathPlanesOut = new List<Plane>();

        for (var i = 0; i < sampleCount; i++)
        {
            var tSec = Math.Min(duration, i * dt);
            var arcLen = speed * tSec;
            if (!curve.LengthParameter(arcLen, out var crvT))
                crvT = curve.Domain.Max;

            var pt = curve.PointAt(crvT);
            var tan = curve.TangentAt(crvT);
            if (!tan.Unitize()) tan = Vector3d.XAxis;
            var yaw = Math.Atan2(tan.Y, tan.X);
            var baseFrame = new MobilityModel.HolonomicSE2(pt.X, pt.Y, yaw).BaseFrame;

            var phase = duration > 1e-9 ? (tSec / duration) : 0;
            var q = SampleLegQ(stanceQ, phase, stepHeight);

            if (!AllFinite(q))
            {
                error = "Gait sample produced non-finite joint values (NaN/Inf).";
                return false;
            }

            points.Add(new TrajectoryPoint(tSec, new JointState(q)));
            basePath.Add(baseFrame);

            if (i == 0 || i == sampleCount - 1 || i % 15 == 0)
                pathPlanesOut.Add(FrameConversion.ToPlanePlate(baseFrame));
        }

        result = new Result(
            new Trajectory(model, points),
            basePath,
            curve,
            pathPlanesOut,
            "Heuristic tripod gait (preview only — not Motus Plan / no terrain IK).");
        return true;
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

    private static double[] SampleLegQ(double[] stanceQ, double pathPhase01, double liftMeters)
    {
        var q = new double[18];
        var cyclePhase = (pathPhase01 * 3.0) % 1.0; // three tripod cycles along path

        for (var leg = 0; leg < 6; leg++)
        {
            var inGroup0 = Array.IndexOf(TripodGroups[0], leg) >= 0;
            var swinging = inGroup0 ? cyclePhase < 0.5 : cyclePhase >= 0.5;
            var local = inGroup0
                ? (cyclePhase < 0.5 ? cyclePhase * 2.0 : 0)
                : (cyclePhase >= 0.5 ? (cyclePhase - 0.5) * 2.0 : 0);
            var swing = swinging ? Math.Sin(local * Math.PI) : 0;
            var lift = liftMeters > 0 ? swing * Math.Min(liftMeters, 0.08) : 0;

            var side = LegNames[leg].StartsWith("left", StringComparison.Ordinal) ? 1.0 : -1.0;
            q[leg * 3 + 0] = stanceQ[leg * 3 + 0] + side * 0.35 * swing;
            q[leg * 3 + 1] = stanceQ[leg * 3 + 1] + 0.8 * lift;
            q[leg * 3 + 2] = stanceQ[leg * 3 + 2] - 1.2 * lift;
        }

        return q;
    }

    private static double[] BuildStanceQ(double hip, double femur, double tibia)
    {
        var q = new double[18];
        for (var leg = 0; leg < 6; leg++)
        {
            var side = LegNames[leg].StartsWith("left", StringComparison.Ordinal) ? 1.0 : -1.0;
            q[leg * 3 + 0] = leg * (Math.PI / 3.0) + side * hip;
            q[leg * 3 + 1] = femur;
            q[leg * 3 + 2] = tibia;
        }
        return q;
    }

    private static bool AllFinite(IReadOnlyList<double> v)
    {
        foreach (var x in v)
            if (!double.IsFinite(x)) return false;
        return true;
    }
}
