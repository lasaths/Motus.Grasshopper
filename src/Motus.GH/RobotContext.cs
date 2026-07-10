using Motus.Core;
using Motus.Geometry;
using Motus.GH.Data;

namespace Motus.GH;

public readonly struct RobotContext
{
    public RobotModel Model { get; }
    public RobotModel EffectiveModel { get; }
    public SerialJointChain? Chain { get; }
    public BaseFrame Base { get; }
    public ToolFrame Tool { get; }
    public RobotCollisionModel? PreviewGeometry { get; }

    public RobotContext(
        RobotModel model,
        RobotModel effectiveModel,
        SerialJointChain? chain,
        BaseFrame @base,
        ToolFrame tool,
        RobotCollisionModel? previewGeometry = null)
    {
        Model = model;
        EffectiveModel = effectiveModel;
        Chain = chain;
        Base = @base;
        Tool = tool;
        PreviewGeometry = previewGeometry;
    }

    public static RobotContext FromGoo(RobotModelGoo goo)
    {
        var session = goo.EffectiveModel();
        return new RobotContext(
            goo.Value!,
            session,
            goo.Chain,
            session.Preset.BaseFrame,
            session.Preset.ToolFrame,
            goo.EffectivePreviewGeometry());
    }
}
