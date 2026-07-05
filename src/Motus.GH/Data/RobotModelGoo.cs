using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.GH.Data;

public sealed class RobotModelGoo : MotusGooBase<RobotModel>
{
    public SerialJointChain? Chain { get; set; }
    public Frame? BaseFrameOverride { get; set; }
    public Frame? ToolFrameOverride { get; set; }

    public RobotModelGoo() { }
    public RobotModelGoo(RobotModel m) : base(m) { }

    public BaseFrame EffectiveBase() =>
        BaseFrameOverride is { } f ? new BaseFrame(f) : Value!.Preset.BaseFrame;

    public ToolFrame EffectiveTool() =>
        ToolFrameOverride is { } f
            ? new ToolFrame(f, Value!.Preset.ToolFrame.Name)
            : Value!.Preset.ToolFrame;

    public static RobotModelGoo FromUrdf(UrdfRobot urdf) =>
        new(urdf.ToModel()) { Chain = urdf.Chain };

    public RobotModelGoo WithFrames(Frame? baseFrame, Frame? toolFrame) =>
        new(Value!) { Chain = Chain, BaseFrameOverride = baseFrame, ToolFrameOverride = toolFrame };

    public override string ToString() => Value?.DisplayName ?? "RobotModel";
}
