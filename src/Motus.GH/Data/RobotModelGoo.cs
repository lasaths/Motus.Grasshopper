using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.GH.Data;

public sealed class RobotModelGoo : MotusGooBase<RobotModel>
{
    public SerialJointChain? Chain { get; set; }
    public RobotCollisionModel? PreviewGeometry { get; set; }
    public Frame? BaseFrameOverride { get; set; }
    public ToolDefinition? Tool { get; set; }

    public RobotModelGoo() { }
    public RobotModelGoo(RobotModel m) : base(m) { }

    public BaseFrame EffectiveBase() =>
        BaseFrameOverride is { } f ? new BaseFrame(f) : Value!.Preset.BaseFrame;

    public ToolFrame EffectiveTool() => Tool?.ToToolFrame() ?? Value!.Preset.ToolFrame;

    public RobotModel EffectiveModel() =>
        TrajectoryGoo.ApplyTool(Value!, Tool, BaseFrameOverride);

    public RobotCollisionModel? EffectivePreviewGeometry()
    {
        if (Tool?.Geometry is null)
            return PreviewGeometry ?? Value!.CollisionModel;
        var links = PreviewGeometry?.Links ?? Value!.CollisionModel?.Links ?? Array.Empty<LinkCollisionGeometry>();
        return new RobotCollisionModel(PreviewGeometry?.Links ?? links, Tool.Geometry);
    }

    public static RobotModelGoo FromUrdf(UrdfRobot urdf, RobotCollisionModel? previewGeometry = null) =>
        new(urdf.ToModel()) { Chain = urdf.Chain, PreviewGeometry = previewGeometry };

    public override string ToString() => Value?.DisplayName ?? "RobotModel";
}
