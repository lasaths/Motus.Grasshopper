using Motus.Core;

using Motus.Geometry;

using Motus.GH.Data;

using System.Drawing;



namespace Motus.GH;



public readonly struct RobotContext

{

    public RobotModel Model { get; }

    public RobotModel EffectiveModel { get; }

    public SerialJointChain? Chain { get; }

    public BaseFrame Base { get; }

    public ToolFrame Tool { get; }

    public RobotCollisionModel? PreviewGeometry { get; }
    public Color?[]? PreviewMeshColors { get; }
    public KinematicTree? Tree { get; }

    public RobotContext(
        RobotModel model,
        RobotModel effectiveModel,
        SerialJointChain? chain,
        BaseFrame @base,
        ToolFrame tool,
        RobotCollisionModel? previewGeometry = null,
        Color?[]? previewMeshColors = null,
        KinematicTree? tree = null)
    {
        Model = model;
        EffectiveModel = effectiveModel;
        Chain = chain;
        Base = @base;
        Tool = tool;
        PreviewGeometry = previewGeometry;
        PreviewMeshColors = previewMeshColors;
        Tree = tree;
    }

    public static RobotContext FromGoo(RobotModelGoo goo)
    {
        goo.EnsureChainFromPath();
        goo.EnsureBundledTool();
        var session = goo.EffectiveModel();
        return new RobotContext(
            goo.Value!,
            session,
            goo.Chain,
            goo.EffectiveBase(),
            goo.EffectiveTool(),
            goo.EffectivePreviewGeometry(),
            goo.PreviewMeshColors,
            goo.Tree);
    }
}

