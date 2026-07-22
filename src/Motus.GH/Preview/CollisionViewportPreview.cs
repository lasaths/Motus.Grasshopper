using Grasshopper.Kernel;
using Motus.Core;
using Motus.GH.Resources;
using Motus.GH.Rhino;
using Rhino.Display;
using Rhino.Geometry;
using System.Drawing;

namespace Motus.GH.Preview;

internal static class CollisionViewportPreview
{
    private static readonly Color ObstacleColor = Color.FromArgb(160, MotusPalette.Collision);
    // ponytail: lazy — avoid DisplayMaterial during GHA type-scan
    private static DisplayMaterial? _obstacleMaterial;
    private static DisplayMaterial ObstacleMaterial =>
        _obstacleMaterial ??= new(ObstacleColor) { Transparency = 0.25 };

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
        foreach (var mesh in meshes)
            args.Display.DrawMeshShaded(mesh, ObstacleMaterial);
    }
}
