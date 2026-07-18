using Motus.Core;
using Rhino.Geometry;
using System.Collections.Generic;

namespace Motus.GH;

internal static class CollisionMeshBuilder
{
    public const int DenseTriangleWarnThreshold = 20_000;

    public static CollisionObject? FromMesh(Mesh mesh, Plane plane, string name) =>
        FromMesh(mesh, plane, name, out _);

    public static CollisionObject? FromMesh(Mesh mesh, Plane plane, string name, out int triangleCount)
    {
        triangleCount = 0;
        var xform = Transform.PlaneToPlane(Plane.WorldXY, plane);
        var vertices = new List<double[]>(mesh.Vertices.Count);
        foreach (var v in mesh.Vertices)
        {
            var pt = new Point3d(v.X, v.Y, v.Z);
            pt.Transform(xform);
            vertices.Add(new[] { pt.X, pt.Y, pt.Z });
        }

        var indices = new List<int>(mesh.Faces.Count * 3);
        foreach (var face in mesh.Faces)
        {
            if (face.IsTriangle)
            {
                indices.Add(face.A);
                indices.Add(face.B);
                indices.Add(face.C);
            }
            else
            {
                indices.Add(face.A);
                indices.Add(face.B);
                indices.Add(face.C);
                indices.Add(face.A);
                indices.Add(face.C);
                indices.Add(face.D);
            }
        }

        if (indices.Count < 3) return null;
        triangleCount = indices.Count / 3;
        return CollisionObject.Mesh(name, Frame.Identity, vertices, indices);
    }

    public static CollisionObject? FromBrep(Brep brep, Plane plane, string name) =>
        FromBrep(brep, plane, name, out _);

    public static CollisionObject? FromBrep(Brep brep, Plane plane, string name, out int triangleCount)
    {
        // Coarser than Default — obstacle meshes are for collision, not CAD fidelity.
        var meshes = Mesh.CreateFromBrep(brep, MeshingParameters.FastRenderMesh);
        if (meshes is null || meshes.Length == 0)
        {
            triangleCount = 0;
            return null;
        }

        var combined = new Mesh();
        foreach (var m in meshes)
            combined.Append(m);
        return FromMesh(combined, plane, name, out triangleCount);
    }
}
