using Motus.Core;
using Motus.Geometry;
using Motus.GH.Data;

namespace Motus.GH;

public readonly struct RobotContext
{
    public RobotModel Model { get; }
    public SerialJointChain? Chain { get; }
    public BaseFrame Base { get; }
    public ToolFrame Tool { get; }

    public RobotContext(RobotModel model, SerialJointChain? chain, BaseFrame @base, ToolFrame tool)
    {
        Model = model;
        Chain = chain;
        Base = @base;
        Tool = tool;
    }

    public static RobotContext FromGoo(RobotModelGoo goo) =>
        new(goo.Value!, goo.Chain, goo.EffectiveBase(), goo.EffectiveTool());
}
