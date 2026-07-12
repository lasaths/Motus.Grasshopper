using Grasshopper.Kernel;
using Motus.Core;
using Motus.GH.Data;
using Motus.GH.Rhino;
using Motus.GH.Urdf;
using Rhino.Display;
using Rhino.Geometry;
using System.Drawing;

namespace Motus.GH.Preview;

internal static class RobotViewportPreview
{
  // White ghost for robot-source components; Motus Preview keeps its own teal styling.
  private static readonly Color MeshColor = Color.FromArgb(180, 255, 255, 255);
  private static readonly Color WireColor = Color.FromArgb(200, 255, 255, 255);
  private static readonly DisplayMaterial MeshMaterial = new(MeshColor) { Transparency = 0.55 };

  public static void Build(
    RobotModelGoo goo,
    string? sourcePath,
    out List<Mesh> meshes,
    out List<Line> wires)
  {
    meshes = [];
    wires = [];
    if (goo.Value is null) return;

    goo.EnsureChainFromPath(sourcePath);
    goo.UrdfSourcePath ??= sourcePath;
    var home = HomePoseLookup.HomeOrZeros(goo.Value, sourcePath);
    var ctx = RobotContext.FromGoo(goo);
    var geometry = RobotPreviewGeometry.ForViewport(
        ctx.PreviewGeometry ?? ctx.EffectiveModel.CollisionModel,
        goo.Tool);
    if (geometry is not null &&
        KinematicsPreview.PreviewMeshCache.TryCreate(ctx.EffectiveModel, geometry, ctx.Chain, ctx.Base, ctx.Tool) is { } cache)
    {
      meshes = cache.MeshesFor(home);
    }
    else
    {
      meshes = KinematicsPreview
        .LinkMeshes(ctx.EffectiveModel, home, geometry, ctx.Chain, ctx.Base, ctx.Tool)
        .ToList();
    }
    wires = KinematicsPreview
      .LinkLines(ctx.EffectiveModel, home, ctx.Chain, ctx.Base, ctx.Tool)
      .ToList();
  }

  public static BoundingBox ComputeBounds(IReadOnlyList<Mesh> meshes, IReadOnlyList<Line> wires)
  {
    var bb = BoundingBox.Empty;
    foreach (var mesh in meshes)
      bb.Union(mesh.GetBoundingBox(false));
    if (meshes.Count == 0)
    {
      foreach (var line in wires)
        bb.Union(line.BoundingBox);
    }
    return bb.IsValid ? bb : BoundingBox.Unset;
  }

  public static void DrawMeshes(IGH_PreviewArgs args, IReadOnlyList<Mesh> meshes)
  {
    if (meshes.Count == 0) return;
    foreach (var mesh in meshes)
      args.Display.DrawMeshShaded(mesh, MeshMaterial);
  }

  public static void DrawWires(IGH_PreviewArgs args, IReadOnlyList<Line> wires, bool draw)
  {
    if (!draw || wires.Count == 0) return;
    foreach (var line in wires)
      args.Display.DrawLine(line, WireColor, 2);
  }
}
