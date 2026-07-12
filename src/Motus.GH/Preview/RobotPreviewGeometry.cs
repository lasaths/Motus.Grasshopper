using Motus.Core;
using Motus.Geometry;

namespace Motus.GH.Preview;

internal static class RobotPreviewGeometry
{
    public static RobotCollisionModel? ForViewport(RobotCollisionModel? preview, ToolDefinition? tool)
    {
        if (preview is null)
            return tool?.Geometry is { } onlyTool ? new RobotCollisionModel([], onlyTool) : null;

        // Always include tool collision mesh in preview when present — bundled Robotiq STL
        // is used for planning but was previously hidden when URDF gripper visuals load.
        if (tool?.Geometry is { } toolGeom)
            return new RobotCollisionModel(preview.Links, toolGeom);

        return preview;
    }
}
