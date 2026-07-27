using Motus.Geometry;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

namespace Motus.GH.Planning;

/// <summary>
/// Rhino mesh/brep → Motus.NET <see cref="LeggedGait.TerrainHeight"/> (downward +Z ray).
/// </summary>
internal static class TerrainHeightRhino
{
    /// <summary>
    /// Build a height sampler from Mesh/Brep/Box/Surface ScriptVariables. Null if empty (flat gait).
    /// Misses return NaN so Motus.NET rejects with a named Status.
    /// </summary>
    public static LeggedGait.TerrainHeight? TryCreate(IReadOnlyList<object> geometry, out string? warning)
    {
        warning = null;
        if (geometry is null || geometry.Count == 0)
            return null;

        var meshes = new List<Mesh>();
        foreach (var g in geometry)
        {
            switch (g)
            {
                case Mesh m when m.IsValid:
                    meshes.Add(m.DuplicateMesh());
                    break;
                case Brep b when b.IsValid:
                    AppendBrepMeshes(meshes, b);
                    break;
                case Box box when box.IsValid:
                    if (Brep.CreateFromBox(box) is { IsValid: true } bb)
                        AppendBrepMeshes(meshes, bb);
                    break;
                case Extrusion ex when ex.IsValid:
                    if (ex.ToBrep() is { IsValid: true } eb)
                        AppendBrepMeshes(meshes, eb);
                    break;
                case Surface s when s.IsValid:
                    if (Brep.CreateFromSurface(s) is { IsValid: true } sb)
                        AppendBrepMeshes(meshes, sb);
                    break;
            }
        }

        if (meshes.Count == 0)
        {
            warning = "Terrain geometry produced no mesh — gait uses flat Z=0.";
            return null;
        }

        var combined = new Mesh();
        foreach (var m in meshes)
            combined.Append(m);
        combined.Normals.ComputeNormals();
        combined.Compact();
        if (!combined.IsValid || combined.Vertices.Count < 3)
        {
            warning = "Terrain mesh invalid — gait uses flat Z=0.";
            return null;
        }

        var bbox = combined.GetBoundingBox(true);
        var zTop = bbox.Max.Z + 1.0;
        var zSpan = Math.Max(2.0, bbox.Max.Z - bbox.Min.Z + 2.0);

        return (x, y) =>
        {
            var ray = new Ray3d(new Point3d(x, y, zTop), -Vector3d.ZAxis);
            var t = Intersection.MeshRay(combined, ray);
            if (t < 0 || !double.IsFinite(t) || t > zSpan)
                return double.NaN;
            return zTop - t;
        };
    }

    private static void AppendBrepMeshes(List<Mesh> meshes, Brep brep)
    {
        var joined = Mesh.CreateFromBrep(brep, MeshingParameters.Default);
        if (joined is { Length: > 0 })
            meshes.AddRange(joined);
    }
}
