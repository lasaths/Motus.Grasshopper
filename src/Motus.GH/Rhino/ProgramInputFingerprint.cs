using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.NET;
using System.Text.Json;

namespace Motus.GH.Rhino;

/// <summary>Identity of the resolved program request, including state inherited from Prior.</summary>
public static class ProgramInputFingerprint
{
    public static string Compute(RobotModel model, Frame? baseFrame, ToolDefinition? tool,
        IReadOnlyList<MotionSegment> segments, JointState start, EndEffectorState? initialToolState,
        PlanningContext context, long? treeFingerprint = null, SerialJointChain? chain = null,
        SamplingPlannerId planner = SamplingPlannerId.RrtConnect, int maxIterations = 4000,
        double timeLimit = 30, double goalBias = 0.08, double step = 0.12)
    {
        var common = PlanInputFingerprint.Compute(model, baseFrame, tool, [], start, context,
            plannerId: planner, rrtMaxIterations: maxIterations, rrtMaxPlanTimeSeconds: timeLimit,
            rrtGoalBias: goalBias, rrtStepRadians: step, treeFingerprint: treeFingerprint);
        // Serialize metadata only: mesh content is represented by its construction-time hash.
        var metadata = segments.Select(s => new
        {
            s.Type, s.BlendRadiusMeters, s.ToolStateMode,
            Target = State(s.TargetState),
            Contacts = s.AllowedCollisionPairs.Select(p => new[] { p.A, p.B }).ToArray(),
            Payload = s switch
            {
                PtpSegment p => JsonSerializer.Serialize(p.Goal.Positions),
                LinSegment l => JsonSerializer.Serialize(new { l.Goal, l.StepMeters }),
                CircSegment c => JsonSerializer.Serialize(new { c.Via, c.Goal, c.ArcSamples }),
                TransferSegment t => JsonSerializer.Serialize(t.Goal),
                SetToolStateSegment t => JsonSerializer.Serialize(new { t.DurationSeconds, State = State(t.State) }),
                WaitSegment w => JsonSerializer.Serialize(w.DurationSeconds),
                AttachSegment a => JsonSerializer.Serialize(new { a.Name, a.TcpLocal, a.Geometry.ContentHash, a.Geometry.Pose }),
                DetachSegment d => JsonSerializer.Serialize(new { d.Name, d.WorldPose }),
                _ => throw new ArgumentException($"Unsupported program segment {s.GetType().Name}.")
            }
        });
        return common + JsonSerializer.Serialize(chain) + JsonSerializer.Serialize(State(initialToolState))
            + JsonSerializer.Serialize(metadata);
    }

    private static object? State(EndEffectorState? state) => state?.Values.OrderBy(p => p.Key, StringComparer.Ordinal)
        .Select(p => new { p.Key, p.Value }).ToArray();
}
