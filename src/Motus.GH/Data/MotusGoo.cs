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

public sealed class JointStateGoo : MotusGooBase<JointState>
{
    public JointStateGoo() { }
    public JointStateGoo(JointState s) : base(s) { }
    public override string ToString() => $"Joints[{Value?.AxisCount}]";
}

public sealed class TrajectoryGoo : MotusGooBase<Trajectory>
{
    public SerialJointChain? Chain { get; set; }
    public Frame? BaseFrameOverride { get; set; }
    public Frame? ToolFrameOverride { get; set; }

    public TrajectoryGoo() { }
    public TrajectoryGoo(Trajectory t) : base(t) { }

    public RobotContext Context() =>
        new(
            Value!.Robot,
            Chain,
            BaseFrameOverride is { } bf ? new BaseFrame(bf) : Value.Robot.Preset.BaseFrame,
            ToolFrameOverride is { } tf
                ? new ToolFrame(tf, Value.Robot.Preset.ToolFrame.Name)
                : Value.Robot.Preset.ToolFrame);

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
