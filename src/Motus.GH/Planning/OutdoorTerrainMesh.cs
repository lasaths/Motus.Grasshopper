using Rhino.Geometry;

namespace Motus.GH.Planning;

/// <summary>
/// Soft outdoor heightfield (gentle hills) for WalkHex Terrain demos — meters, Z-up.
/// </summary>
internal static class OutdoorTerrainMesh
{
    /// <param name="origin">Patch center (m).</param>
    /// <param name="sizeX">Full extent X (m).</param>
    /// <param name="sizeY">Full extent Y (m).</param>
    /// <param name="amplitude">Hill height peak (m) — keep under WalkHex Lift.</param>
    public static Mesh Build(
        Point3d origin,
        double sizeX = 1.2,
        double sizeY = 1.0,
        double amplitude = 0.04,
        int resX = 32,
        int resY = 26)
    {
        sizeX = Math.Max(0.1, sizeX);
        sizeY = Math.Max(0.1, sizeY);
        amplitude = Math.Max(0, amplitude);
        resX = Math.Clamp(resX, 4, 64);
        resY = Math.Clamp(resY, 4, 64);

        var mesh = new Mesh();
        for (var j = 0; j <= resY; j++)
        {
            var v = j / (double)resY;
            var y = origin.Y + (v - 0.5) * sizeY;
            for (var i = 0; i <= resX; i++)
            {
                var u = i / (double)resX;
                var x = origin.X + (u - 0.5) * sizeX;
                // Soft rolling ground + mild diagonal grade (outdoor path feel).
                var z = origin.Z
                    + amplitude * Math.Sin(u * Math.PI * 2.2) * Math.Cos(v * Math.PI * 1.7)
                    + 0.45 * amplitude * Math.Sin((u + v) * Math.PI * 1.4)
                    + 0.55 * amplitude * (u - 0.5); // clear cross-slope so feet climb
                mesh.Vertices.Add(x, y, z);
            }
        }

        var stride = resX + 1;
        for (var j = 0; j < resY; j++)
        {
            for (var i = 0; i < resX; i++)
            {
                var a = j * stride + i;
                var b = a + 1;
                var c = a + stride;
                var d = c + 1;
                // Triangles — MeshRay is unreliable on quads.
                mesh.Faces.AddFace(a, b, d);
                mesh.Faces.AddFace(a, d, c);
            }
        }

        mesh.Normals.ComputeNormals();
        mesh.FaceNormals.ComputeFaceNormals();
        mesh.Compact();
        return mesh;
    }
}
