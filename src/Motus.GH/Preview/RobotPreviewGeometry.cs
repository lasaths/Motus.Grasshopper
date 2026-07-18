using Motus.Core;
using Motus.Geometry;

namespace Motus.GH.Preview;

internal static class RobotPreviewGeometry
{
    /// <summary>
    /// Viewport meshes: URDF visuals by default. When the session tool has a morphable
    /// jaw-width mesh, attach that tool mesh and drop static URDF gripper visuals so SET
    /// can scale fingers in Motus Preview.
    /// </summary>
    public static RobotCollisionModel? ForViewport(RobotCollisionModel? preview, ToolDefinition? tool)
    {
        if (CanMorphJawWidth(tool))
        {
            var links = preview?.Links
                .Where(l => !IsStaticGripperVisual(l.LinkName))
                .ToList()
                ?? [];
            return new RobotCollisionModel(
                links,
                tool!.Geometry,
                tool.GeometryInFlangeFrame,
                tool.GeometryAttachOffset);
        }

        // Planning-only tool hulls stay off the viewport when they cannot morph.
        return StripToolGeometry(preview);
    }

    private static bool CanMorphJawWidth(ToolDefinition? tool) =>
        tool?.Geometry is { Shape: CollisionShape.Mesh } &&
        tool.Capabilities?.Parameters.Any(p =>
            string.Equals(p.Name, "width", StringComparison.Ordinal)) == true;

    private static bool IsStaticGripperVisual(string linkName) =>
        linkName.StartsWith("robotiq", StringComparison.OrdinalIgnoreCase);

    private static RobotCollisionModel? StripToolGeometry(RobotCollisionModel? model) =>
        model is null
            ? null
            : model.ToolGeometry is null
                ? model
                : new RobotCollisionModel(model.Links);
}
