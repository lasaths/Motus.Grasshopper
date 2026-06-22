using Motus.Core;
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

public sealed class RobotModelGoo : MotusGooBase<RobotModel>
{
    public RobotModelGoo() { }
    public RobotModelGoo(RobotModel m) : base(m) { }
    public override string ToString() => Value?.DisplayName ?? "RobotModel";
}

public sealed class JointStateGoo : MotusGooBase<JointState>
{
    public JointStateGoo() { }
    public JointStateGoo(JointState s) : base(s) { }
    public override string ToString() => $"Joints[{Value?.AxisCount}]";
    public override IGH_Goo Duplicate() => new JointStateGoo(Value);
}

public sealed class TrajectoryGoo : MotusGooBase<Trajectory>
{
    public TrajectoryGoo() { }
    public TrajectoryGoo(Trajectory t) : base(t) { }
    public override string ToString() => $"Trajectory ({Value?.Points.Count} pts)";
    public override IGH_Goo Duplicate() => new TrajectoryGoo(Value);
}

public sealed class FrameGoo : GH_Goo<Frame>
{
    public FrameGoo() => Value = Frame.Identity;
    public FrameGoo(Frame f) => Value = f;
    public override bool IsValid => true;
    public override string TypeName => "Frame";
    public override string TypeDescription => "Motus Frame";
    public override IGH_Goo Duplicate() => new FrameGoo(Value);
    public override string ToString() => Value.ToString();
}

public sealed class ToolFrameGoo : GH_Goo<ToolFrame>
{
    public ToolFrameGoo() => Value = ToolFrame.Identity;
    public ToolFrameGoo(ToolFrame f) => Value = f;
    public override bool IsValid => true;
    public override string TypeName => "ToolFrame";
    public override string TypeDescription => "Motus ToolFrame";
    public override IGH_Goo Duplicate() => new ToolFrameGoo(Value);
    public override string ToString() => Value.Name ?? "ToolFrame";
}

public sealed class BaseFrameGoo : GH_Goo<BaseFrame>
{
    public BaseFrameGoo() => Value = BaseFrame.Identity;
    public BaseFrameGoo(BaseFrame f) => Value = f;
    public override bool IsValid => true;
    public override string TypeName => "BaseFrame";
    public override string TypeDescription => "Motus BaseFrame";
    public override IGH_Goo Duplicate() => new BaseFrameGoo(Value);
    public override string ToString() => "BaseFrame";
}
