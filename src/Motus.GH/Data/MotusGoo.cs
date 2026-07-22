using Motus.Core;
using Motus.Geometry;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Motus.GH.Preview;
using System.Drawing;

namespace Motus.GH.Data;

public abstract class MotusGooBase<T> : GH_Goo<T> where T : class
{
    protected MotusGooBase(T value) => Value = value;
    protected MotusGooBase() => Value = null!;
    public override bool IsValid => Value != null;
    public override string TypeName => typeof(T).Name;
    public override string TypeDescription => $"Motus {typeof(T).Name}";
    public override IGH_Goo Duplicate() => MemberwiseClone() as IGH_Goo ?? this;
}

public sealed class ToolGoo : MotusGooBase<ToolDefinition>
{
    public ToolGoo() { }
    public ToolGoo(ToolDefinition tool) : base(tool) { }

    public override string ToString() => Value is null ? "Tool" : $"{Value.Name} ({Value.Tcp})";
}

public sealed class EndEffectorStateGoo : MotusGooBase<EndEffectorState>
{
    public EndEffectorStateGoo() { }
    public EndEffectorStateGoo(EndEffectorState state) : base(state) { }

    public override string ToString() => Value?.ToString() ?? "ToolState";
}

public sealed class JointStateGoo : MotusGooBase<JointState>
{
    public JointStateGoo() { }
    public JointStateGoo(JointState s) : base(s) { }
    public override string ToString() => $"Joints[{Value?.AxisCount}]";
}

public sealed class TrajectoryGoo : MotusGooBase<Trajectory>
{
    public SerialJointChain? Chain { get; set; }
    public KinematicTree? Tree { get; set; }
    public RobotCollisionModel? PreviewGeometry { get; set; }
    public Color?[]? PreviewMeshColors { get; set; }
    public Frame? BaseFrameOverride { get; set; }
    public ToolDefinition? ToolSnapshot { get; set; }
    public ToolCapabilities? ToolCapabilitiesSnapshot { get; set; }
    public IReadOnlyList<PlanningMessage>? DiagnosticsSnapshot { get; set; }
    public PlannerProvenance? ProvenanceSnapshot { get; set; }

    public TrajectoryGoo() { }
    public TrajectoryGoo(Trajectory t) : base(t) { }

    public RobotContext Context()
    {
        var model = Value!.Robot;
        var session = ApplyTool(model, ToolSnapshot, BaseFrameOverride);
        var preview = RobotPreviewGeometry.ForViewport(PreviewGeometry, ToolSnapshot);
        // Tree required so Robotiq tip-descendant meshes pose via TreeFK (not stuck at base).
        return new RobotContext(model, session, Chain, session.Preset.BaseFrame, session.Preset.ToolFrame, preview, PreviewMeshColors, Tree);
    }

    internal static RobotModel ApplyTool(RobotModel model, ToolDefinition? tool, Frame? baseOverride)
    {
        BaseFrame? baseFrame = baseOverride is { } bf ? new BaseFrame(bf) : null;
        return model.WithTool(tool, baseFrame);
    }

    public override string ToString() => $"Trajectory ({Value?.Points.Count} pts)";
}

public sealed class CollisionObjectGoo : MotusGooBase<CollisionObject>
{
    public CollisionObjectGoo() { }
    public CollisionObjectGoo(CollisionObject obj) : base(obj) { }
    public override string ToString() => Value?.Name ?? "CollisionObject";
}

public sealed class CollisionSceneGoo : MotusGooBase<CollisionScene>
{
    public CollisionSceneGoo() : base(new CollisionScene()) { }
    public CollisionSceneGoo(CollisionScene scene) : base(scene) { }
    public override string ToString() => $"CollisionScene ({Value.Objects.Count} objs)";
}

public sealed class PlanningGroupGoo : MotusGooBase<PlanningGroup>
{
    public PlanningGroupGoo() { }
    public PlanningGroupGoo(PlanningGroup group) : base(group) { }
    public override string ToString() => Value is null
        ? "PlanningGroup"
        : $"{Value.Name} ({Value.BaseLink}->{Value.TipLink})";
}

public sealed class AttachedBodyGoo : MotusGooBase<AttachedBody>
{
    public AttachedBodyGoo() { }
    public AttachedBodyGoo(AttachedBody body) : base(body) { }
    public override string ToString() => Value is null
        ? "AttachedBody"
        : $"{Value.Name} ({Value.Geometry.Shape})";
}

public sealed class MotionSegmentGoo : MotusGooBase<MotionSegment>
{
    public MotionSegmentGoo() { }
    public MotionSegmentGoo(MotionSegment segment) : base(segment) { }

    public override string ToString() => Value switch
    {
        PtpSegment ptp => $"PTP blend={ptp.BlendRadiusMeters:F3}m",
        LinSegment lin => $"LIN step={lin.StepMeters:F3}m blend={lin.BlendRadiusMeters:F3}m",
        CircSegment circ => $"CIRC samples={circ.ArcSamples} blend={circ.BlendRadiusMeters:F3}m",
        SetToolStateSegment set => $"SET dur={set.DurationSeconds:F2}s",
        WaitSegment wait => $"WAIT dur={wait.DurationSeconds:F2}s",
        _ => "Segment"
    };
}
