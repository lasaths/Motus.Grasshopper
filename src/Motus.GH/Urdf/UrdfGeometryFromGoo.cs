using Grasshopper.Kernel.Types;
using Motus.Geometry;
using Motus.GH.Rhino;
using Rhino.Geometry;

namespace Motus.GH.Urdf;

/// <summary>
/// Shared Rhino/GH → <see cref="UrdfGeometry"/> conversion (meters). Used by Motus Urdf Link and Motus Robot Attach.
/// </summary>
internal static class UrdfGeometryFromGoo
{
    public static bool TryConvert(IGH_GeometricGoo? goo, out UrdfGeometry? geometry, out string? error)
    {
        geometry = null;
        error = null;
        if (goo is null)
        {
            error = "Geometry item is empty.";
            return false;
        }

        if (goo is GH_Box { Value: { IsValid: true } box })
        {
            var sx = box.X.Length;
            var sy = box.Y.Length;
            var sz = box.Z.Length;
            if (sx <= 0 || sy <= 0 || sz <= 0)
            {
                error = "Box extents must be > 0.";
                return false;
            }

            geometry = UrdfGeometry.Box(sx, sy, sz, FrameConversion.FromPlane(box.Plane));
            return true;
        }

        if (!TryMeshAny(goo, out var mesh) || mesh is not { IsValid: true } || mesh.Faces.Count == 0)
        {
            error = "Could not mesh geometry — wire Box, Mesh, Brep, Surface, Extrusion, SubD, or similar (meters).";
            return false;
        }

        var vertices = new List<double[]>(mesh.Vertices.Count);
        foreach (var v in mesh.Vertices)
            vertices.Add([v.X, v.Y, v.Z]);

        var indices = new List<int>(mesh.Faces.Count * 3);
        foreach (var face in mesh.Faces)
        {
            indices.Add(face.A);
            indices.Add(face.B);
            indices.Add(face.C);
            if (!face.IsTriangle)
            {
                indices.Add(face.A);
                indices.Add(face.C);
                indices.Add(face.D);
            }
        }

        geometry = UrdfGeometry.Mesh(vertices, indices);
        return true;
    }

    public static bool TryConvertAll(
        IReadOnlyList<IGH_GeometricGoo> geos,
        out List<UrdfGeometry> result,
        out string? error)
    {
        result = new List<UrdfGeometry>(geos.Count);
        error = null;
        foreach (var geo in geos)
        {
            if (!TryConvert(geo, out var urdf, out error) || urdf is null)
                return false;
            result.Add(urdf);
        }

        return result.Count > 0;
    }

    private static bool TryMeshAny(IGH_GeometricGoo goo, out Mesh? mesh)
    {
        mesh = null;
        if (goo is GH_Mesh { Value: { IsValid: true } m } && m.Faces.Count > 0)
        {
            mesh = m;
            return true;
        }

        var gb = goo.ScriptVariable() as GeometryBase;
        if (gb is null && goo.CastTo(out GeometryBase cast))
            gb = cast;
        if (gb is null)
            return false;

        switch (gb)
        {
            case Mesh direct when direct.IsValid && direct.Faces.Count > 0:
                mesh = direct;
                return true;
            case Brep brep when brep.IsValid:
                mesh = JoinMeshes(Mesh.CreateFromBrep(brep, MeshingParameters.Default));
                return mesh is not null;
            case Extrusion extrusion: // before Surface — Extrusion : Surface
                mesh = JoinMeshes(Mesh.CreateFromBrep(extrusion.ToBrep(), MeshingParameters.Default));
                return mesh is not null;
            case Surface surface:
                mesh = JoinMeshes(Mesh.CreateFromBrep(Brep.CreateFromSurface(surface), MeshingParameters.Default));
                return mesh is not null;
            case SubD subd:
                mesh = Mesh.CreateFromSubD(subd, 2);
                return mesh is { IsValid: true } && mesh.Faces.Count > 0;
            default:
            {
                var asBrep = Brep.TryConvertBrep(gb);
                if (asBrep is { IsValid: true })
                {
                    mesh = JoinMeshes(Mesh.CreateFromBrep(asBrep, MeshingParameters.Default));
                    return mesh is not null;
                }

                return false;
            }
        }
    }

    private static Mesh? JoinMeshes(Mesh[]? parts)
    {
        if (parts is not { Length: > 0 }) return null;
        var mesh = new Mesh();
        foreach (var part in parts)
            mesh.Append(part);
        return mesh.Faces.Count > 0 ? mesh : null;
    }
}
