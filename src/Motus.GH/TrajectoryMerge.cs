using System;
using System.Linq;
using Grasshopper.Kernel;
using Motus.Core;
using Motus.GH.Data;
using System.Collections.Generic;

namespace Motus.GH;

/// <summary>Concatenates Motus Plan multi-goal trajectory lists for Preview / Export / Waypoints.</summary>
internal static class TrajectoryMerge
{
    /// <summary>
    /// Old documents may still serialize Trajectory as item after the list migration.
    /// Call from AddedToDocument so the first solve already uses list access.
    /// </summary>
    public static void EnsureListAccess(GH_Component owner, int index)
    {
        if (index < 0 || index >= owner.Params.Input.Count) return;
        var param = owner.Params.Input[index];
        if (param.Access != GH_ParamAccess.list)
            param.Access = GH_ParamAccess.list;
    }

    public static bool TryResolve(
        IGH_DataAccess da,
        int index,
        GH_Component owner,
        GH_RuntimeMessageLevel multiLevel,
        out TrajectoryGoo goo)
    {
        goo = null!;
        if (index < 0 || index >= owner.Params.Input.Count)
            return false;

        var param = owner.Params.Input[index];

        // Legacy item access: GetDataList would throw. Read once, then migrate.
        if (param.Access == GH_ParamAccess.item)
        {
            TrajectoryGoo? single = null;
            if (!da.GetData(index, ref single) || single?.Value is null)
                return false;
            param.Access = GH_ParamAccess.list;
            goo = single;
            return true;
        }

        var list = new List<TrajectoryGoo>();
        if (!da.GetDataList(index, list) || list.Count == 0)
            return false;

        var valid = list.Where(item => item?.Value is not null).ToList();

        if (valid.Count == 0)
            return false;

        if (valid.Count == 1)
        {
            goo = valid[0];
            return true;
        }

        if (valid.Any(g => !SameAgent(g.Value!, valid[0].Value!)))
        {
            goo = ConcatenateSequence(valid);
            owner.AddRuntimeMessage(
                multiLevel,
                $"One Play: concatenated {valid.Count} agents (shared scrub; hold outside each window).");
            return true;
        }

        goo = Concatenate(valid);
        owner.AddRuntimeMessage(
            multiLevel,
            $"Concatenated {valid.Count} trajectories from Motus Plan (sequential goals).");
        return true;
    }

    private static bool SameAgent(Trajectory a, Trajectory b) =>
        a.Robot.Preset.AxisCount == b.Robot.Preset.AxisCount
        && string.Equals(a.Robot.Preset.Family, b.Robot.Preset.Family, StringComparison.OrdinalIgnoreCase)
        && string.Equals(a.Robot.Preset.ModelName, b.Robot.Preset.ModelName, StringComparison.OrdinalIgnoreCase);

    /// <summary>
    /// Dual-agent pass-off clock: scrub duration = sum of segment times; <see cref="TrajectoryGoo.SequenceAgents"/>
    /// holds the real robots. Clock <see cref="Trajectory"/> uses the last agent (arm) for pin outputs.
    /// </summary>
    public static TrajectoryGoo ConcatenateSequence(IReadOnlyList<TrajectoryGoo> goos)
    {
        var agents = new List<TrajectorySequenceAgent>(goos.Count);
        var clockPoints = new List<TrajectoryPoint>();
        var timeOffset = 0.0;
        var mergedSpans = new List<AttachPreviewSpan>();

        for (var i = 0; i < goos.Count; i++)
        {
            var traj = goos[i].Value!;
            if (traj.Points.Count == 0)
                continue;

            var start = timeOffset;
            var end = timeOffset + traj.Points[^1].TimeSeconds;
            agents.Add(new TrajectorySequenceAgent(goos[i], start, end));

            if (goos[i].AttachSpans is { Count: > 0 } spans)
            {
                foreach (var span in spans)
                {
                    mergedSpans.Add(new AttachPreviewSpan
                    {
                        StartSeconds = start + span.StartSeconds,
                        EndSeconds = start + span.EndSeconds,
                        Bodies = span.Bodies,
                        ReleaseWorldPose = span.ReleaseWorldPose
                    });
                }
            }

            // Clock samples at each agent boundary (times only — Preview poses via SequenceAgents).
            var last = goos[^1].Value!.Points[^1];
            clockPoints.Add(new TrajectoryPoint(start, last.JointState, last.MotionType, last.SegmentIndex,
                last.BlendRadiusMeters, last.ToolState, last.BaseFrameOverride));
            clockPoints.Add(new TrajectoryPoint(end, last.JointState, last.MotionType, last.SegmentIndex,
                last.BlendRadiusMeters, last.ToolState, last.BaseFrameOverride));
            timeOffset = end;
        }

        if (clockPoints.Count == 0)
            return goos[0];

        var lastGoo = goos[^1];
        var clock = new Trajectory(lastGoo.Value!.Robot, clockPoints, mergedSpans.Select(s =>
            new AttachTimeSpan(s.StartSeconds, s.EndSeconds, s.Bodies, s.ReleaseWorldPose)).ToArray());
        return new TrajectoryGoo(clock)
        {
            Chain = lastGoo.Chain,
            Tree = lastGoo.Tree,
            Stewart = lastGoo.Stewart,
            PreviewGeometry = lastGoo.PreviewGeometry,
            PreviewMeshColors = lastGoo.PreviewMeshColors,
            BaseFrameOverride = lastGoo.BaseFrameOverride,
            MobilityGoal = lastGoo.MobilityGoal,
            ToolSnapshot = lastGoo.ToolSnapshot,
            ToolCapabilitiesSnapshot = lastGoo.ToolCapabilitiesSnapshot,
            DiagnosticsSnapshot = lastGoo.DiagnosticsSnapshot,
            ProvenanceSnapshot = lastGoo.ProvenanceSnapshot,
            TreeDriverHome = lastGoo.TreeDriverHome,
            BasePath = lastGoo.BasePath,
            TerrainSampler = lastGoo.TerrainSampler,
            AttachSpans = mergedSpans.Count > 0 ? mergedSpans : null,
            SequenceAgents = agents
        };
    }

    public static TrajectoryGoo Concatenate(IReadOnlyList<TrajectoryGoo> goos)
    {
        var first = goos[0];
        var points = new List<TrajectoryPoint>();
        var mergedSpans = new List<AttachPreviewSpan>();
        var timeOffset = 0.0;

        for (var i = 0; i < goos.Count; i++)
        {
            var segmentStart = timeOffset;
            var traj = goos[i].Value!;
            if (traj.Points.Count == 0)
                continue;

            if (goos[i].AttachSpans is { Count: > 0 } spans)
            {
                foreach (var span in spans)
                {
                    mergedSpans.Add(new AttachPreviewSpan
                    {
                        StartSeconds = segmentStart + span.StartSeconds,
                        EndSeconds = segmentStart + span.EndSeconds,
                        Bodies = span.Bodies,
                        ReleaseWorldPose = span.ReleaseWorldPose
                    });
                }
            }

            var startIndex = 0;
            if (i > 0 && points.Count > 0)
            {
                // Skip duplicate join waypoint when sequential goals share the previous end.
                var prev = points[^1].JointState;
                var next = traj.Points[0].JointState;
                if (JointsNearlyEqual(prev, next))
                    startIndex = 1;
            }

            for (var p = startIndex; p < traj.Points.Count; p++)
            {
                var pt = traj.Points[p];
                points.Add(new TrajectoryPoint(
                    pt.TimeSeconds + timeOffset,
                    pt.JointState,
                    pt.MotionType,
                    pt.SegmentIndex,
                    pt.BlendRadiusMeters,
                    pt.ToolState, pt.BaseFrameOverride));
            }

            if (points.Count > 0)
                timeOffset = points[^1].TimeSeconds;
        }

        var merged = new Trajectory(first.Value!.Robot, points, mergedSpans.Select(s =>
            new AttachTimeSpan(s.StartSeconds, s.EndSeconds, s.Bodies, s.ReleaseWorldPose)).ToArray());
        return new TrajectoryGoo(merged)
        {
            Chain = first.Chain,
            Tree = first.Tree,
            Stewart = first.Stewart,
            PreviewGeometry = first.PreviewGeometry,
            PreviewMeshColors = first.PreviewMeshColors,
            BaseFrameOverride = first.BaseFrameOverride,
            MobilityGoal = first.MobilityGoal,
            ToolSnapshot = first.ToolSnapshot,
            ToolCapabilitiesSnapshot = first.ToolCapabilitiesSnapshot,
            DiagnosticsSnapshot = first.DiagnosticsSnapshot,
            ProvenanceSnapshot = first.ProvenanceSnapshot,
            TreeDriverHome = first.TreeDriverHome,
            BasePath = first.BasePath,
            TerrainSampler = first.TerrainSampler,
            AttachSpans = mergedSpans.Count > 0 ? mergedSpans : null
        };
    }

    private static bool JointsNearlyEqual(JointState a, JointState b)
    {
        if (a.AxisCount != b.AxisCount) return false;
        for (var i = 0; i < a.AxisCount; i++)
        {
            if (Math.Abs(a.Positions[i] - b.Positions[i]) > 1e-6)
                return false;
        }
        return true;
    }
}
