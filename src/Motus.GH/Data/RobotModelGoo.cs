using Motus.Core;
using Motus.Geometry;
using Motus.GH;
using Motus.GH.Urdf;
using Motus.Presets;
using GH_IO.Serialization;
using System.Drawing;

namespace Motus.GH.Data;

public sealed class RobotModelGoo : MotusGooBase<RobotModel>
{
    public SerialJointChain? Chain { get; set; }
    public RobotCollisionModel? PreviewGeometry { get; set; }
    public Color?[]? PreviewMeshColors { get; set; }
    public Frame? BaseFrameOverride { get; set; }
    public ToolDefinition? Tool { get; set; }
    public string? UrdfSourcePath { get; set; }

    public RobotModelGoo() { }
    public RobotModelGoo(RobotModel m) : base(m) { }

    public void EnsureChainFromPath(string? path = null)
    {
        if (Chain is not null) return;
        path = UrdfPathResolver.ResolveUrdfPath(path ?? UrdfSourcePath ?? "");
        if (string.IsNullOrWhiteSpace(path) || !File.Exists(path)) return;
        var urdf = UrdfRobotLoader.Load(path, new UrdfLoadOptions
        {
            BaseLink = "base_link",
            TipLink = "tool0",
            ModelName = Value?.Preset.ModelName ?? Path.GetFileNameWithoutExtension(path)
        });
        Chain = urdf.Chain;
        if (PreviewGeometry is null && !string.IsNullOrWhiteSpace(path))
        {
            var visuals = UrdfRobotLoad.LoadPreviewVisuals(path);
            PreviewGeometry = visuals?.Geometry;
            PreviewMeshColors ??= visuals?.MeshColors;
        }
        UrdfSourcePath ??= path;
    }

    public BaseFrame EffectiveBase() =>
        BaseFrameOverride is { } f ? new BaseFrame(f) : Value!.Preset.BaseFrame;

    public ToolFrame EffectiveTool() => Tool?.ToToolFrame() ?? Value!.Preset.ToolFrame;

    public RobotModel EffectiveModel() =>
        TrajectoryGoo.ApplyTool(Value!, Tool, BaseFrameOverride);

    public void EnsureBundledTool()
    {
        if (Tool?.Geometry is not null) return;
        var path = UrdfSourcePath;
        if (string.IsNullOrWhiteSpace(path)) return;
        if (BundledToolLoader.TryDefaultForUrdfPath(path) is { } tool)
            Tool = tool;
    }

    public RobotCollisionModel? EffectivePreviewGeometry() =>
        PreviewGeometry ?? Value!.CollisionModel;

    public static RobotModelGoo FromUrdf(UrdfRobot urdf, RobotCollisionModel? previewGeometry = null, Color?[]? previewMeshColors = null) =>
        new(urdf.ToModel()) { Chain = urdf.Chain, PreviewGeometry = previewGeometry, PreviewMeshColors = previewMeshColors };

    public override string ToString() => Value?.DisplayName ?? "RobotModel";

    public override bool Write(GH_IWriter writer)
    {
        if (!string.IsNullOrWhiteSpace(UrdfSourcePath))
            writer.SetString("UrdfSourcePath", UrdfSourcePath);
        return true;
    }

    public override bool Read(GH_IReader reader)
    {
        if (reader.ItemExists("UrdfSourcePath"))
            UrdfSourcePath = reader.GetString("UrdfSourcePath");
        return true;
    }
}
