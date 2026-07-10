using Motus.Core;
using Motus.Geometry;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;

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

public sealed class JointStateGoo : MotusGooBase<JointState>
{
    public JointStateGoo() { }
    public JointStateGoo(JointState s) : base(s) { }
    public override string ToString() => $"Joints[{Value?.AxisCount}]";
}

public sealed class TrajectoryGoo : MotusGooBase<Trajectory>
{
    public SerialJointChain? Chain { get; set; }
    public RobotCollisionModel? PreviewGeometry { get; set; }
    public Frame? BaseFrameOverride { get; set; }
    public ToolDefinition? ToolSnapshot { get; set; }

    public TrajectoryGoo() { }
    public TrajectoryGoo(Trajectory t) : base(t) { }

    public RobotContext Context(RobotModelGoo? robotOverride = null)
    {
        var model = Value!.Robot;
        ToolDefinition? tool = ToolSnapshot;
        Frame? baseOverride = BaseFrameOverride;
        SerialJointChain? chain = Chain;
        RobotCollisionModel? preview = PreviewGeometry;

        if (robotOverride?.Value is not null)
        {
            if (tool is null)
                tool = robotOverride.Tool;
            if (baseOverride is null && robotOverride.BaseFrameOverride is { } bf)
                baseOverride = bf;
            chain ??= robotOverride.Chain;
            preview ??= robotOverride.PreviewGeometry;
            model = robotOverride.Value;
        }

        var session = ApplyTool(model, tool, baseOverride);
        return new RobotContext(session, session, chain, session.Preset.BaseFrame, session.Preset.ToolFrame, preview);
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
        _ => "Segment"
    };
}
