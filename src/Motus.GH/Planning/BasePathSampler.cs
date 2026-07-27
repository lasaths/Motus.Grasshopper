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
                return LerpSe2(basePath[i], basePath[i + 1], alpha);
            }
        }

        return basePath[^1];
    }

    private static Frame LerpSe2(Frame a, Frame b, double alpha)
    {
        var ya = YawFromFrame(a);
        var yb = YawFromFrame(b);
        var dy = yb - ya;
        while (dy > Math.PI) dy -= 2 * Math.PI;
        while (dy < -Math.PI) dy += 2 * Math.PI;
        var yaw = ya + alpha * dy;
        var x = a.X + alpha * (b.X - a.X);
        var y = a.Y + alpha * (b.Y - a.Y);
        return new MobilityModel.HolonomicSE2(x, y, yaw).BaseFrame;
    }

    private static double YawFromFrame(Frame f)
    {
        // HolonomicSE2 stores yaw as Z quaternion component.
        return 2.0 * Math.Atan2(f.Qz, f.Qw);
    }
}
