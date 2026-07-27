using Motus.Geometry;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using Grasshopper.Kernel.Types;

namespace Motus.GH.Planning;

/// <summary>
/// Any Rhino Mesh/Brep/Surface/Extrusion/SubD/Box → Motus.NET <see cref="LeggedGait.TerrainHeight"/>.
/// Contract: world ground Z at (x,y) = first hit of a downward ray (top surface if overlaps).
/// </summary>
internal static class TerrainHeightRhino
{
    /// <summary>
    /// Null if empty (flat gait). Misses return NaN so Motus.NET rejects with a named Status.
    /// </summary>
    public static LeggedGait.TerrainHeight? TryCreate(IReadOnlyList<object> geometry, out string? warning)
    {
        warning = null;
        if (geometry is null || geometry.Count == 0)
            return null;

        var meshes = new List<Mesh>();
        foreach (var g in geometry)
            TryAppend(g, meshes);

        if (meshes.Count == 0)
        {
            warning = "Terrain geometry produced no mesh — gait uses flat Z=0.";
            return null;
        }

        var combined = new Mesh();
        foreach (var m in meshes)
            combined.Append(m);
        PrepareTerrainMesh(combined);
        if (!combined.IsValid || combined.Vertices.Count < 3 || combined.Faces.Count < 1)
        {
            warning = "Terrain mesh invalid — gait uses flat Z=0.";
            return null;
        }

        var bbox = combined.GetBoundingBox(true);
        var margin = 0.02;
        var minX = bbox.Min.X - margin;
        var maxX = bbox.Max.X + margin;
        var minY = bbox.Min.Y - margin;
        var maxY = bbox.Max.Y + margin;
        var zTop = bbox.Max.Z + 1.0;
        var zSpan = Math.Max(2.0, bbox.Max.Z - bbox.Min.Z + 2.0);

        // Keep mesh alive for MeshRay; snapshot verts for XY fallback when ray misses.
        var vx = new double[combined.Vertices.Count];
        var vy = new double[combined.Vertices.Count];
        var vz = new double[combined.Vertices.Count];
        for (var i = 0; i < combined.Vertices.Count; i++)
        {
            var p = combined.Vertices[i];
            vx[i] = p.X;
            vy[i] = p.Y;
            vz[i] = p.Z;
        }
        var fa = new int[combined.Faces.Count];
        var fb = new int[combined.Faces.Count];
        var fc = new int[combined.Faces.Count];
        for (var i = 0; i < combined.Faces.Count; i++)
        {
            var f = combined.Faces[i];
            fa[i] = f.A;
            fb[i] = f.B;
            fc[i] = f.C;
        }

        return (x, y) =>
        {
            if (x < minX || x > maxX || y < minY || y > maxY)
                return double.NaN;

            var ray = new Ray3d(new Point3d(x, y, zTop), -Vector3d.ZAxis);
            var t = Intersection.MeshRay(combined, ray);
            if (t >= 0 && double.IsFinite(t) && t <= zSpan)
                return zTop - t;

            // Fallback: highest XY-projected triangle Z (heightfields / MeshRay edge misses).
            var bestZ = double.NegativeInfinity;
            var hit = false;
            for (var i = 0; i < fa.Length; i++)
            {
                if (!TryBarycentricZ(
                        x, y,
                        vx[fa[i]], vy[fa[i]], vz[fa[i]],
                        vx[fb[i]], vy[fb[i]], vz[fb[i]],
                        vx[fc[i]], vy[fc[i]], vz[fc[i]],
                        out var z))
                    continue;
                if (z > bestZ)
                    bestZ = z;
                hit = true;
            }
            return hit ? bestZ : double.NaN;
        };
    }

    private static void PrepareTerrainMesh(Mesh mesh)
    {
        if (mesh.Ngons.Count > 0)
            mesh.Ngons.Clear();
        if (mesh.Faces.QuadCount > 0)
            mesh.Faces.ConvertQuadsToTriangles();
        mesh.Vertices.CombineIdentical(true, true);
        mesh.FaceNormals.ComputeFaceNormals();
        mesh.Normals.ComputeNormals();
        mesh.Compact();
    }

    /// <summary>XY containment + barycentric Z. Overlaps take the highest Z (top surface).</summary>
    private static bool TryBarycentricZ(
        double x, double y,
        double ax, double ay, double az,
        double bx, double by, double bz,
        double cx, double cy, double cz,
        out double z)
    {
        z = 0;
        var v0x = cx - ax;
        var v0y = cy - ay;
        var v1x = bx - ax;
        var v1y = by - ay;
        var v2x = x - ax;
        var v2y = y - ay;
        var dot00 = v0x * v0x + v0y * v0y;
        var dot01 = v0x * v1x + v0y * v1y;
        var dot02 = v0x * v2x + v0y * v2y;
        var dot11 = v1x * v1x + v1y * v1y;
        var dot12 = v1x * v2x + v1y * v2y;
        var denom = dot00 * dot11 - dot01 * dot01;
        if (Math.Abs(denom) < 1e-18)
            return false;
        var inv = 1.0 / denom;
        var u = (dot11 * dot02 - dot01 * dot12) * inv;
        var v = (dot00 * dot12 - dot01 * dot02) * inv;
        if (u < -1e-8 || v < -1e-8 || u + v > 1.0 + 1e-8)
            return false;
        z = az + v * (bz - az) + u * (cz - az);
        return double.IsFinite(z);
    }

    /// <summary>Pull Rhino geometry out of GH goos or ScriptVariables.</summary>
    public static void CollectFromGoos(IEnumerable<IGH_GeometricGoo?> goos, ICollection<object> dest)
    {
        foreach (var goo in goos)
        {
            if (goo is null) continue;
            switch (goo)
            {
                case GH_Mesh { Value: { IsValid: true } mesh }:
                    dest.Add(mesh.DuplicateMesh());
                    break;
                case GH_Brep { Value: { IsValid: true } brep }:
                    dest.Add(brep.DuplicateBrep());
                    break;
                case GH_Box boxGoo:
                    dest.Add(boxGoo.Value);
                    break;
                case GH_Surface { Value: { IsValid: true } srf }:
                    dest.Add(srf.Duplicate());
                    break;
                case GH_Extrusion { Value: { IsValid: true } ex }:
                    dest.Add(ex.Duplicate());
                    break;
                case GH_SubD { Value: { IsValid: true } subd }:
                    dest.Add(subd.Duplicate());
                    break;
                default:
                {
                    var sv = goo.ScriptVariable();
                    if (sv is not null)
                        dest.Add(sv);
                    break;
                }
            }
        }
    }

    private static void TryAppend(object g, List<Mesh> meshes)
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
                if (Brep.CreateFromBox(box) is { IsValid: true } boxFromStruct)
                    AppendBrepMeshes(meshes, boxFromStruct);
                break;
            case BoundingBox bounds when bounds.IsValid:
                if (Brep.CreateFromBox(new Box(bounds)) is { IsValid: true } boxBrep)
                    AppendBrepMeshes(meshes, boxBrep);
                break;
            case Extrusion ex when ex.IsValid:
                if (ex.ToBrep() is { IsValid: true } eb)
                    AppendBrepMeshes(meshes, eb);
                break;
            case Surface s when s.IsValid:
                AppendSurfaceMeshes(meshes, s);
                break;
            case SubD subd when subd.IsValid:
                AppendSubDMeshes(meshes, subd);
                break;
            case GeometryBase gb:
                switch (gb)
                {
                    case Mesh mm when mm.IsValid:
                        meshes.Add(mm.DuplicateMesh());
                        break;
                    case Brep bb2 when bb2.IsValid:
                        AppendBrepMeshes(meshes, bb2);
                        break;
                    case Extrusion ex2 when ex2.IsValid && ex2.ToBrep() is { IsValid: true } eb2:
                        AppendBrepMeshes(meshes, eb2);
                        break;
                    case Surface s2 when s2.IsValid:
                        AppendSurfaceMeshes(meshes, s2);
                        break;
                    case SubD sd2 when sd2.IsValid:
                        AppendSubDMeshes(meshes, sd2);
                        break;
                }
                break;
        }
    }

    private static void AppendSurfaceMeshes(List<Mesh> meshes, Surface s)
    {
        if (Brep.CreateFromSurface(s) is { IsValid: true } sb)
            AppendBrepMeshes(meshes, sb);
    }

    private static void AppendSubDMeshes(List<Mesh> meshes, SubD subd)
    {
        // Density 2 ≈ display mesh; enough for plant height without exploding face count.
        var sm = Mesh.CreateFromSubD(subd, 2);
        if (sm is { IsValid: true })
        {
            meshes.Add(sm);
            return;
        }
        if (subd.ToBrep(SubDToBrepOptions.Default) is { IsValid: true } br)
            AppendBrepMeshes(meshes, br);
    }

    private static void AppendBrepMeshes(List<Mesh> meshes, Brep brep)
    {
        // QualityRenderMesh so freeform Surfaces/Breps aren't under-tessellated for foot plants.
        var joined = Mesh.CreateFromBrep(brep, MeshingParameters.QualityRenderMesh)
                     ?? Mesh.CreateFromBrep(brep, MeshingParameters.Default);
        if (joined is { Length: > 0 })
            meshes.AddRange(joined);
    }
}
