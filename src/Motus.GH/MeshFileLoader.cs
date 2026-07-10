using Rhino.Geometry;
using System.Globalization;

namespace Motus.GH;

internal static class MeshFileLoader
{
    public static Mesh? LoadStl(string path)
    {
        if (!File.Exists(path)) return null;
        var (vertices, indices) = ReadStl(path, 1.0);
        if (vertices.Count == 0 || indices.Count < 3) return null;

        var mesh = new Mesh();
        foreach (var v in vertices)
            mesh.Vertices.Add(v[0], v[1], v[2]);
        for (var i = 0; i + 2 < indices.Count; i += 3)
            mesh.Faces.AddFace(indices[i], indices[i + 1], indices[i + 2]);
        return mesh.IsValid ? mesh : null;
    }

    public static (List<double[]> vertices, List<int> indices) ReadStlBytes(string path, double uniformScale = 1.0) =>
        ReadStl(path, uniformScale);

    private static (List<double[]> vertices, List<int> indices) ReadStl(string path, double uniformScale)
    {
        var bytes = File.ReadAllBytes(path);
        if (bytes.Length < 84) return (new List<double[]>(), new List<int>());
        var triCount = BitConverter.ToUInt32(bytes, 80);
        var expected = 84L + triCount * 50L;
        return expected == bytes.LongLength
            ? ReadBinaryStl(bytes, triCount, uniformScale)
            : ReadAsciiStl(File.ReadAllLines(path), uniformScale);
    }

    private static (List<double[]> vertices, List<int> indices) ReadBinaryStl(byte[] bytes, uint triCount, double scale)
    {
        var vertices = new List<double[]>((int)triCount * 3);
        var indices = new List<int>((int)triCount * 3);
        var offset = 84;
        for (var i = 0; i < triCount && offset + 50 <= bytes.Length; i++)
        {
            offset += 12;
            for (var v = 0; v < 3; v++)
            {
                var x = BitConverter.ToSingle(bytes, offset) * scale; offset += 4;
                var y = BitConverter.ToSingle(bytes, offset) * scale; offset += 4;
                var z = BitConverter.ToSingle(bytes, offset) * scale; offset += 4;
                vertices.Add(new[] { (double)x, (double)y, (double)z });
                indices.Add(vertices.Count - 1);
            }
            offset += 2;
        }
        return (vertices, indices);
    }

    private static (List<double[]> vertices, List<int> indices) ReadAsciiStl(string[] lines, double scale)
    {
        var vertices = new List<double[]>();
        var indices = new List<int>();
        foreach (var raw in lines)
        {
            var line = raw.Trim();
            if (!line.StartsWith("vertex ", StringComparison.OrdinalIgnoreCase)) continue;
            var parts = line.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length < 4) continue;
            if (!double.TryParse(parts[1], NumberStyles.Float, CultureInfo.InvariantCulture, out var x)) continue;
            if (!double.TryParse(parts[2], NumberStyles.Float, CultureInfo.InvariantCulture, out var y)) continue;
            if (!double.TryParse(parts[3], NumberStyles.Float, CultureInfo.InvariantCulture, out var z)) continue;
            vertices.Add(new[] { x * scale, y * scale, z * scale });
            indices.Add(vertices.Count - 1);
        }
        return (vertices, indices);
    }
}
