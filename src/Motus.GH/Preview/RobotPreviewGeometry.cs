using Motus.Core;

namespace Motus.GH.Preview;

internal static class RobotPreviewGeometry
{
    /// <summary>
    /// Viewport meshes: URDF visuals only. Jaw motion uses Robotiq URDF/PickNik FK in
    /// <see cref="Rhino.KinematicsPreview"/> — not a flattened tool collision mesh.
    /// </summary>
    public static RobotCollisionModel? ForViewport(RobotCollisionModel? preview, ToolDefinition? tool)
    {
        // ponytail: planning tool hull stays off viewport; finger pose is FK, not mesh squash
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
