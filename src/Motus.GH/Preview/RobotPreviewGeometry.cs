using Motus.Core;
using Motus.Geometry;

namespace Motus.GH.Preview;

internal static class RobotPreviewGeometry
{
    public static RobotCollisionModel? ForViewport(RobotCollisionModel? preview, ToolDefinition? tool)
    {
        if (preview is null)
            return tool?.Geometry is { } onlyTool ? new RobotCollisionModel([], onlyTool) : null;

        if (tool?.Geometry is not { } toolGeom || HasUrdfGripperVisuals(preview))
            return preview;

        return new RobotCollisionModel(preview.Links, toolGeom);
    }

    internal static bool HasUrdfGripperVisuals(RobotCollisionModel preview) =>
        preview.Links.Any(l => l.LinkName.Contains("robotiq", StringComparison.OrdinalIgnoreCase));
}
