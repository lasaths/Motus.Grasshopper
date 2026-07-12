using Motus.Core;
using Motus.Geometry;

namespace Motus.GH.Rhino;

internal static class ToolCollisionPreview
{
    public static double[] WorldMatrix(
        IFkSolver fk,
        IReadOnlyList<double> joints,
        BaseFrame baseFrame,
        ToolFrame toolFrame,
        RobotCollisionModel geometry) =>
        ToolCollisionPlacement.WorldMatrix(
            fk,
            joints,
            baseFrame,
            toolFrame,
            geometry.ToolGeometry,
            geometry.ToolGeometryInFlangeFrame,
            geometry.ToolGeometryAttachOffset);
}
