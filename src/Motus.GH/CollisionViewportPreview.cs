using Grasshopper.Kernel;
using Motus.Core;
using Motus.Rhino;
using Rhino.Display;
using Rhino.Geometry;
using System.Drawing;

namespace Motus.GH;

internal static class CollisionViewportPreview
{
    private static readonly Color ObstacleColor = Color.FromArgb(160, 220, 70, 70);

    public static List<Mesh> MeshesFor(CollisionObject obj)
    {
        var mesh = KinematicsPreview.CollisionObjectMesh(obj);
        return mesh is null ? [] : [mesh];
    }

    public static List<Mesh> MeshesFor(CollisionScene scene) =>
        KinematicsPreview.CollisionSceneMeshes(scene).ToList();

    public static BoundingBox MeshesBoundingBox(IReadOnlyList<Mesh> meshes)
    {
        var bb = BoundingBox.Empty;
        foreach (var mesh in meshes)
            bb.Union(mesh.GetBoundingBox(false));
        return bb.IsValid ? bb : BoundingBox.Unset;
    }

    public static void DrawMeshes(IGH_PreviewArgs args, IReadOnlyList<Mesh> meshes)
    {
        if (meshes.Count == 0) return;
        var mat = new DisplayMaterial(ObstacleColor) { Transparency = 0.25 };
        foreach (var mesh in meshes)
            args.Display.DrawMeshShaded(mesh, mat);
    }
}
