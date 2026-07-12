using Grasshopper.Kernel;
using Motus.Core;
using Motus.GH.Components;
using Motus.GH.Data;
using Motus.GH.Params;

namespace Motus.GH.Preview;

/// <summary>Waypoint layout for Motus Scrub — display positions are evenly spaced by index; time fractions drive playback.</summary>
internal readonly struct ScrubTimeline
{
    public static ScrubTimeline Empty { get; } = new(0, Array.Empty<double>(), Array.Empty<double>(), Array.Empty<double>());

    public double DurationSeconds { get; }
    /// <summary>Elapsed-time fraction TimeSeconds[i] / duration.</summary>
    public IReadOnlyList<double> TimeFractions { get; }
    /// <summary>Evenly spaced index fraction i / (n - 1) for ticks and snap targets.</summary>
    public IReadOnlyList<double> DisplayFractions { get; }

    public bool IsEmpty => DisplayFractions.Count == 0;
    public int Count => DisplayFractions.Count;

    public IReadOnlyList<double> WaypointTimes { get; }

    public ScrubTimeline(double durationSeconds, IReadOnlyList<double> timeFractions, IReadOnlyList<double> displayFractions, IReadOnlyList<double> waypointTimes)
    {
        DurationSeconds = durationSeconds;
        TimeFractions = timeFractions;
        DisplayFractions = displayFractions;
        WaypointTimes = waypointTimes;
    }

    public static ScrubTimeline From(Trajectory trajectory)
    {
        var count = trajectory.Points.Count;
        if (count == 0) return Empty;

        var duration = trajectory.DurationSeconds;
        var timeFracs = new double[count];
        var displayFracs = new double[count];
        var waypointTimes = new double[count];
        for (var i = 0; i < count; i++)
        {
            waypointTimes[i] = trajectory.Points[i].TimeSeconds;
            timeFracs[i] = duration <= 1e-9
                ? 0
                : Math.Clamp(waypointTimes[i] / duration, 0, 1);
            displayFracs[i] = count <= 1 ? 0 : (double)i / (count - 1);
        }
        return new ScrubTimeline(duration, timeFracs, displayFracs, waypointTimes);
    }

    public double TimeAt(double displayFraction) =>
        DurationSeconds * Math.Clamp(displayFraction, 0, 1);

    public int NearestDisplayIndex(double displayFraction)
    {
        if (DisplayFractions.Count == 0) return 0;
        var t = Math.Clamp(displayFraction, 0, 1);
        var best = 0;
        var bestDist = double.MaxValue;
        for (var i = 0; i < DisplayFractions.Count; i++)
        {
            var d = Math.Abs(DisplayFractions[i] - t);
            if (d < bestDist)
            {
                bestDist = d;
                best = i;
            }
        }
        return best;
    }

    public double DisplayFractionAt(int index)
    {
        if (DisplayFractions.Count == 0) return 0;
        index = Math.Clamp(index, 0, DisplayFractions.Count - 1);
        return DisplayFractions[index];
    }

    public double Snap(double displayFraction, float trackWidthPx, float snapPx = 12f)
    {
        if (DisplayFractions.Count == 0) return Math.Clamp(displayFraction, 0, 1);
        var idx = NearestDisplayIndex(displayFraction);
        var target = DisplayFractions[idx];
        var threshold = trackWidthPx <= 1f ? 1.0 : (double)snapPx / trackWidthPx;
        return Math.Abs(target - displayFraction) <= threshold ? target : Math.Clamp(displayFraction, 0, 1);
    }

    public double SnapToNearest(double displayFraction) =>
        DisplayFractionAt(NearestDisplayIndex(displayFraction));
}

internal static class ScrubTimelineProbe
{
    public static ScrubTimeline TryResolve(MotusScrubSlider scrub)
    {
        var doc = scrub.OnPingDocument();
        if (doc is null) return ScrubTimeline.Empty;

        foreach (var obj in doc.Objects)
        {
            if (obj is not MotusPreviewComponent preview) continue;
            if (!preview.Params.Input[2].Sources.Any(s => ReferenceEquals(s, scrub))) continue;

            var trajInput = preview.Params.Input[0];
            foreach (var source in trajInput.Sources)
            {
                var trajectory = ReadTrajectory(source);
                if (trajectory is not null) return ScrubTimeline.From(trajectory);
            }
        }
        return ScrubTimeline.Empty;
    }

    private static Trajectory? ReadTrajectory(IGH_Param param)
    {
        if (param.VolatileDataCount == 0) return null;
        foreach (var goo in param.VolatileData.AllData(true))
        {
            if (goo is TrajectoryGoo tg && tg.Value is not null)
                return tg.Value;
        }
        return null;
    }
}
