using Motus.Core;
using Motus.Geometry;

namespace Motus.GH.Preview;

internal static class RobotPreviewGeometry
{
    /// <summary>Viewport meshes only — never includes tool collision hull (planning-only, TCP-framed).</summary>
    public static RobotCollisionModel? ForViewport(RobotCollisionModel? preview, ToolDefinition? tool)
    {
        _ = tool;
        return StripToolGeometry(preview);
    }

    private static RobotCollisionModel? StripToolGeometry(RobotCollisionModel? model) =>
        model is null
            ? null
            : model.ToolGeometry is null
                ? model
                : new RobotCollisionModel(model.Links);
}
