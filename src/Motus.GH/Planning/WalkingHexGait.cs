using Motus.Core;
using Motus.Geometry;
using Motus.GH.Rhino;
using Rhino.Geometry;

namespace Motus.GH.Planning;

/// <summary>
/// Tripod gait with planted foot targets + per-leg analytic IK (preview only — not Motus Plan).
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
        double bodyR,
        double coxa,
        double femur,
        double tibia,
        double bodyZ,
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

        if (bodyR <= 0 || coxa <= 0 || femur <= 0 || tibia <= 0 || bodyZ <= 0)
        {
            error = "BodyR / Coxa / Femur / Tibia / BodyZ must be > 0.";
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

        if (!curve.LengthParameter(0, out var t0))
            t0 = curve.Domain.Min;
        var startPt = curve.PointAt(t0);
        var startTan = curve.TangentAt(t0);
        if (!startTan.Unitize()) startTan = Vector3d.XAxis;
        var startYaw = Math.Atan2(startTan.Y, startTan.X);
        var startFrame = new MobilityModel.HolonomicSE2(startPt.X, startPt.Y, startYaw).BaseFrame;

        var plants = InitializePlants(
            startFrame, bodyR, bodyZ, coxa, femur, tibia, stanceQ, out var initErr);
        if (plants is null)
        {
            error = initErr;
            return false;
        }

        var qPrev = (double[])stanceQ.Clone();
        var wasSwinging = new bool[6];
        var swingEnd = new Point3d[6];
        var ikFailSamples = 0;
        string? ikWarning = null;

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

            var pathPhase = duration > 1e-9 ? tSec / duration : 0;
            var cyclePhase = (pathPhase * 3.0) % 1.0;
            var q = (double[])qPrev.Clone();

            for (var leg = 0; leg < 6; leg++)
            {
                var (swinging, swingLocal) = LegSwingPhase(leg, cyclePhase);
                Point3d footWorld;

                if (!swinging)
                {
                    footWorld = new Point3d(plants[leg].X, plants[leg].Y, 0);
                    wasSwinging[leg] = false;
                }
                else
                {
                    if (!wasSwinging[leg])
                    {
                        swingEnd[leg] = new Point3d(
                            plants[leg].X + stepLength * Math.Cos(yaw),
                            plants[leg].Y + stepLength * Math.Sin(yaw),
                            0);
                    }

                    footWorld = SwingFoot(plants[leg], swingEnd[leg], swingLocal, stepHeight);
                    if (swingLocal >= 0.999)
                        plants[leg] = swingEnd[leg];
                    wasSwinging[leg] = true;
                }

                var footBody = WorldToBody(footWorld, baseFrame);
                var hipBody = HipBody(leg, bodyR, bodyZ);

                if (WalkingHexLegIk.TrySolve(hipBody, footBody, coxa, femur, tibia, out var q0, out var q1, out var q2))
                {
                    q[leg * 3 + 0] = q0;
                    q[leg * 3 + 1] = q1;
                    q[leg * 3 + 2] = q2;
                }
                else
                {
                    q[leg * 3 + 0] = qPrev[leg * 3 + 0];
                    q[leg * 3 + 1] = qPrev[leg * 3 + 1];
                    q[leg * 3 + 2] = qPrev[leg * 3 + 2];
                    ikFailSamples++;
                }
            }

            if (!AllFinite(q))
            {
                error = "Gait sample produced non-finite joint values (NaN/Inf).";
                return false;
            }

            qPrev = q;
            points.Add(new TrajectoryPoint(tSec, new JointState(q)));
            basePath.Add(baseFrame);

            if (i == 0 || i == sampleCount - 1 || i % 15 == 0)
                pathPlanesOut.Add(FrameConversion.ToPlanePlate(baseFrame));
        }

        if (ikFailSamples > 0)
            ikWarning = $"Foot-target IK failed on {ikFailSamples} leg×sample(s); held previous q.";

        result = new Result(
            new Trajectory(model, points),
            basePath,
            curve,
            pathPlanesOut,
            ikWarning ?? "Foot-target tripod gait (preview only — wire Trajectory → Preview; not Motus Plan).");
        return true;
    }

    private static Point3d[]? InitializePlants(
        Frame startBase,
        double bodyR,
        double bodyZ,
        double coxa,
        double femur,
        double tibia,
        double[] stanceQ,
        out string error)
    {
        error = "";
        var plants = new Point3d[6];
        for (var leg = 0; leg < 6; leg++)
        {
            var hipBody = HipBody(leg, bodyR, bodyZ);
            var footBody = WalkingHexLegIk.FootPosition(
                hipBody, coxa, femur, tibia,
                stanceQ[leg * 3 + 0], stanceQ[leg * 3 + 1], stanceQ[leg * 3 + 2]);
            var footTargetBody = new Point3d(footBody.X, footBody.Y, 0);

            if (!WalkingHexLegIk.TrySolve(hipBody, footTargetBody, coxa, femur, tibia, out _, out _, out _))
            {
                error = $"Leg {LegNames[leg]}: stance foot at Z=0 unreachable (BodyZ={bodyZ:F3} m too low or geometry infeasible).";
                return null;
            }

            plants[leg] = BodyToWorld(footTargetBody, startBase);
        }

        return plants;
    }

    private static Point3d HipBody(int leg, double bodyR, double bodyZ)
    {
        var yaw = leg * (Math.PI / 3.0);
        return new Point3d(bodyR * Math.Cos(yaw), bodyR * Math.Sin(yaw), bodyZ);
    }

    private static Point3d WorldToBody(Point3d world, Frame baseFrame)
    {
        var yaw = YawFromFrame(baseFrame);
        var dx = world.X - baseFrame.X;
        var dy = world.Y - baseFrame.Y;
        var c = Math.Cos(-yaw);
        var s = Math.Sin(-yaw);
        return new Point3d(c * dx - s * dy, s * dx + c * dy, world.Z);
    }

    private static Point3d BodyToWorld(Point3d body, Frame baseFrame)
    {
        var yaw = YawFromFrame(baseFrame);
        var c = Math.Cos(yaw);
        var s = Math.Sin(yaw);
        return new Point3d(
            baseFrame.X + c * body.X - s * body.Y,
            baseFrame.Y + s * body.X + c * body.Y,
            body.Z);
    }

    private static double YawFromFrame(Frame f) => 2.0 * Math.Atan2(f.Qz, f.Qw);

    private static Point3d SwingFoot(Point3d start, Point3d end, double phase01, double liftMeters)
    {
        var t = Math.Clamp(phase01, 0, 1);
        var x = start.X + (end.X - start.X) * t;
        var y = start.Y + (end.Y - start.Y) * t;
        var z = liftMeters > 0 ? liftMeters * Math.Sin(t * Math.PI) : 0;
        return new Point3d(x, y, z);
    }

    private static (bool Swinging, double LocalPhase01) LegSwingPhase(int leg, double cyclePhase01)
    {
        var inGroup0 = Array.IndexOf(TripodGroups[0], leg) >= 0;
        var swinging = inGroup0 ? cyclePhase01 < 0.5 : cyclePhase01 >= 0.5;
        if (!swinging) return (false, 0);

        var local = inGroup0
            ? (cyclePhase01 < 0.5 ? cyclePhase01 * 2.0 : 0)
            : (cyclePhase01 >= 0.5 ? (cyclePhase01 - 0.5) * 2.0 : 0);
        return (true, local);
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
