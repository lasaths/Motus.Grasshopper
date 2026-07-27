using Motus.Core;

namespace Motus.GH.Planning;

/// <summary>Sample mobile-base frames parallel to a trajectory timeline.</summary>
internal static class BasePathSampler
{
    public static Frame AtTime(IReadOnlyList<Frame> basePath, Trajectory trajectory, double elapsedSeconds)
    {
        if (basePath.Count == 0) return Frame.Identity;
        var pts = trajectory.Points;
        if (basePath.Count != pts.Count)
        {
            var idx = Math.Clamp((int)Math.Round(elapsedSeconds / Math.Max(trajectory.DurationSeconds, 1e-9) * (basePath.Count - 1)), 0, basePath.Count - 1);
            return basePath[idx];
        }

        if (pts.Count == 1 || elapsedSeconds <= pts[0].TimeSeconds)
            return basePath[0];
        if (elapsedSeconds >= pts[^1].TimeSeconds)
            return basePath[^1];

        for (var i = 0; i < pts.Count - 1; i++)
        {
            var t0 = pts[i].TimeSeconds;
            var t1 = pts[i + 1].TimeSeconds;
            if (elapsedSeconds < t1 || i == pts.Count - 2)
            {
                if (t1 <= t0 + 1e-12) return basePath[i];
                var alpha = Math.Clamp((elapsedSeconds - t0) / (t1 - t0), 0, 1);
                return LerpFrame(basePath[i], basePath[i + 1], alpha);
            }
        }

        return basePath[^1];
    }

    /// <summary>Lerp translation + slerp orientation (keeps terrain Z / stance pitch — not SE2-only).</summary>
    private static Frame LerpFrame(Frame a, Frame b, double alpha)
    {
        var x = a.X + alpha * (b.X - a.X);
        var y = a.Y + alpha * (b.Y - a.Y);
        var z = a.Z + alpha * (b.Z - a.Z);
        Slerp(
            a.Qw, a.Qx, a.Qy, a.Qz,
            b.Qw, b.Qx, b.Qy, b.Qz,
            alpha,
            out var qw, out var qx, out var qy, out var qz);
        return new Frame(x, y, z, qw, qx, qy, qz);
    }

    private static void Slerp(
        double aw, double ax, double ay, double az,
        double bw, double bx, double by, double bz,
        double t,
        out double w, out double x, out double y, out double z)
    {
        var dot = aw * bw + ax * bx + ay * by + az * bz;
        if (dot < 0)
        {
            bw = -bw; bx = -bx; by = -by; bz = -bz;
            dot = -dot;
        }

        if (dot > 0.9995)
        {
            w = aw + t * (bw - aw);
            x = ax + t * (bx - ax);
            y = ay + t * (by - ay);
            z = az + t * (bz - az);
            var n = Math.Sqrt(w * w + x * x + y * y + z * z);
            if (n < 1e-15) { w = 1; x = y = z = 0; return; }
            w /= n; x /= n; y /= n; z /= n;
            return;
        }

        var theta0 = Math.Acos(Math.Clamp(dot, -1, 1));
        var theta = theta0 * t;
        var s0 = Math.Sin(theta0);
        var s1 = Math.Sin(theta0 - theta) / s0;
        var s2 = Math.Sin(theta) / s0;
        w = s1 * aw + s2 * bw;
        x = s1 * ax + s2 * bx;
        y = s1 * ay + s2 * by;
        z = s1 * az + s2 * bz;
    }
}
