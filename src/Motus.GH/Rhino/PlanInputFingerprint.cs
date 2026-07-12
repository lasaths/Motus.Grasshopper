using Motus.Core;
using Motus.GH.Planning;
using Rhino.Geometry;
using System.Globalization;
using System.Text;

namespace Motus.GH.Rhino;

/// <summary>Stable fingerprint of Motus Plan inputs for auto-replan detection.</summary>
public static class PlanInputFingerprint
{
    public static string Compute(
        RobotModel model,
        Frame? baseFrameOverride,
        ToolDefinition? toolOverride,
        IReadOnlyList<(JointState? joints, Plane? plane)> goals,
        JointState start,
        PlanningContext context,
        double linStepMeters = 0.005,
        RrtPlanSettings? rrtSettings = null)
    {
        rrtSettings ??= RrtPlanSettings.Defaults;
        var rrt = rrtSettings.Value;

        var sb = new StringBuilder(512);
        sb.Append("model:").Append(model.Preset.ModelName).Append('|');
        sb.Append("linStep:").Append(linStepMeters.ToString("R", CultureInfo.InvariantCulture)).Append('|');
        sb.Append("rrt:").Append(rrt.PlannerId).Append(',')
            .Append(rrt.MaxIterations).Append(',')
            .Append(rrt.MaxPlanTimeSeconds.ToString("R", CultureInfo.InvariantCulture)).Append(',')
            .Append(rrt.GoalBias.ToString("R", CultureInfo.InvariantCulture)).Append(',')
            .Append(rrt.StepRadians.ToString("R", CultureInfo.InvariantCulture)).Append('|');
        AppendFrame(sb, "base", baseFrameOverride);
        if (toolOverride is not null)
        {
            sb.Append("toolName:").Append(toolOverride.Name).Append('|');
            AppendFrame(sb, "toolTcp", toolOverride.Tcp);
            if (toolOverride.Geometry is { } geom)
                AppendCollisionObject(sb, geom);
        }
        AppendJoints(sb, "start", start.Positions);
        for (var i = 0; i < goals.Count; i++)
        {
            var goal = goals[i];
            if (goal.plane is { } plane)
                AppendPlane(sb, $"g{i}", plane);
            else if (goal.joints is { } joints)
                AppendJoints(sb, $"g{i}", joints.Positions);
        }

        var objects = context.Scene.Objects.OrderBy(o => o.Name, StringComparer.Ordinal).ToList();
        sb.Append("scene:").Append(objects.Count).Append('|');
        foreach (var obj in objects)
            AppendCollisionObject(sb, obj);

        var group = context.ActiveGroup;
        if (group is not null)
        {
            sb.Append("group:").Append(group.Name).Append('|')
                .Append(group.BaseLink).Append('|').Append(group.TipLink).Append('|');
            foreach (var joint in group.JointNames)
                sb.Append(joint).Append(',');
            sb.Append('|');
        }

        var attached = context.Attached.OrderBy(a => a.Name, StringComparer.Ordinal).ToList();
        sb.Append("attach:").Append(attached.Count).Append('|');
        foreach (var body in attached)
        {
            sb.Append(body.Name).Append('|');
            AppendFrame(sb, "tcp", body.TcpLocalPose);
            if (body.SourceSceneObjectName is { } source)
                sb.Append("src:").Append(source).Append('|');
            AppendCollisionObject(sb, body.Geometry);
        }

        return sb.ToString();
    }

    private static void AppendJoints(StringBuilder sb, string label, IReadOnlyList<double> joints)
    {
        sb.Append(label).Append(':');
        foreach (var q in joints)
            sb.Append(q.ToString("R", CultureInfo.InvariantCulture)).Append(',');
        sb.Append('|');
    }

    private static void AppendPlane(StringBuilder sb, string label, Plane plane)
    {
        sb.Append(label).Append(":p:")
            .Append(plane.OriginX.ToString("R", CultureInfo.InvariantCulture)).Append(',')
            .Append(plane.OriginY.ToString("R", CultureInfo.InvariantCulture)).Append(',')
            .Append(plane.OriginZ.ToString("R", CultureInfo.InvariantCulture)).Append(',')
            .Append(plane.XAxis.X.ToString("R", CultureInfo.InvariantCulture)).Append(',')
            .Append(plane.XAxis.Y.ToString("R", CultureInfo.InvariantCulture)).Append(',')
            .Append(plane.XAxis.Z.ToString("R", CultureInfo.InvariantCulture)).Append(',')
            .Append(plane.YAxis.X.ToString("R", CultureInfo.InvariantCulture)).Append(',')
            .Append(plane.YAxis.Y.ToString("R", CultureInfo.InvariantCulture)).Append(',')
            .Append(plane.YAxis.Z.ToString("R", CultureInfo.InvariantCulture)).Append('|');
    }

    private static void AppendFrame(StringBuilder sb, string label, Frame? frame)
    {
        if (frame is not { } f) return;
        sb.Append(label).Append(":f:")
            .Append(f.X.ToString("R", CultureInfo.InvariantCulture)).Append(',')
            .Append(f.Y.ToString("R", CultureInfo.InvariantCulture)).Append(',')
            .Append(f.Z.ToString("R", CultureInfo.InvariantCulture)).Append(',')
            .Append(f.Qw.ToString("R", CultureInfo.InvariantCulture)).Append(',')
            .Append(f.Qx.ToString("R", CultureInfo.InvariantCulture)).Append(',')
            .Append(f.Qy.ToString("R", CultureInfo.InvariantCulture)).Append(',')
            .Append(f.Qz.ToString("R", CultureInfo.InvariantCulture)).Append('|');
    }

    private static void AppendCollisionObject(StringBuilder sb, CollisionObject obj)
    {
        sb.Append(obj.Name).Append('|').Append(obj.Shape).Append('|');
        AppendFrame(sb, "pose", obj.Pose);
        sb.Append(obj.ExtentX.ToString("R", CultureInfo.InvariantCulture)).Append(',')
            .Append(obj.ExtentY.ToString("R", CultureInfo.InvariantCulture)).Append(',')
            .Append(obj.ExtentZ.ToString("R", CultureInfo.InvariantCulture)).Append('|');
        if (obj.MeshVertices is { Count: > 0 } vertices)
        {
            sb.Append("mv:").Append(vertices.Count).Append('|');
            foreach (var v in vertices)
            {
                sb.Append(v[0].ToString("R", CultureInfo.InvariantCulture)).Append(',')
                    .Append(v[1].ToString("R", CultureInfo.InvariantCulture)).Append(',')
                    .Append(v[2].ToString("R", CultureInfo.InvariantCulture)).Append(';');
            }
        }

        if (obj.MeshIndices is { Count: > 0 } indices)
        {
            sb.Append("mi:").Append(indices.Count).Append('|');
            foreach (var index in indices)
                sb.Append(index).Append(',');
        }

        sb.Append('|');
    }
}
