using Grasshopper.Kernel;
using Motus.Core;
using Motus.GH.Data;
using System.Collections.Generic;

namespace Motus.GH;

/// <summary>Concatenates Motus Plan multi-goal trajectory lists for Preview / Export / Data.</summary>
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

        var valid = new List<TrajectoryGoo>(list.Count);
        foreach (var item in list)
        {
            if (item?.Value is not null)
                valid.Add(item);
        }

        if (valid.Count == 0)
            return false;

        if (valid.Count == 1)
        {
            goo = valid[0];
            return true;
        }

        goo = Concatenate(valid);
        owner.AddRuntimeMessage(
            multiLevel,
            $"Concatenated {valid.Count} trajectories from Motus Plan (sequential goals).");
        return true;
    }

    public static TrajectoryGoo Concatenate(IReadOnlyList<TrajectoryGoo> goos)
    {
        var first = goos[0];
        var points = new List<TrajectoryPoint>();
        var timeOffset = 0.0;

        for (var i = 0; i < goos.Count; i++)
        {
            var traj = goos[i].Value!;
            if (traj.Points.Count == 0)
                continue;

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
                    pt.ToolState));
            }

            if (points.Count > 0)
                timeOffset = points[^1].TimeSeconds;
        }

        var merged = new Trajectory(first.Value!.Robot, points);
        return new TrajectoryGoo(merged)
        {
            Chain = first.Chain,
            PreviewGeometry = first.PreviewGeometry,
            PreviewMeshColors = first.PreviewMeshColors,
            BaseFrameOverride = first.BaseFrameOverride,
            ToolSnapshot = first.ToolSnapshot,
            ToolCapabilitiesSnapshot = first.ToolCapabilitiesSnapshot,
            DiagnosticsSnapshot = first.DiagnosticsSnapshot,
            ProvenanceSnapshot = first.ProvenanceSnapshot
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
