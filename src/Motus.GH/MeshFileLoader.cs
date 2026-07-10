using Motus.Presets;
using Rhino.Geometry;

namespace Motus.GH;

internal static class MeshFileLoader
{
    public static Mesh? LoadStl(string path)
    {
        if (!File.Exists(path)) return null;
        var (vertices, indices) = StlReader.Read(path);
        if (vertices.Count == 0 || indices.Count < 3) return null;

        var mesh = new Mesh();
        foreach (var v in vertices)
            mesh.Vertices.Add(v[0], v[1], v[2]);
        for (var i = 0; i + 2 < indices.Count; i += 3)
            mesh.Faces.AddFace(indices[i], indices[i + 1], indices[i + 2]);
        return mesh.IsValid ? mesh : null;
    }

    public static (List<double[]> vertices, List<int> indices) ReadStlBytes(string path, double uniformScale = 1.0) =>
        StlReader.Read(path, uniformScale);
}
