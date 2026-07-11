using Motus.Presets;
using Rhino.Geometry;
using System.Collections.Concurrent;

namespace Motus.GH;

internal static class MeshFileLoader
{
    private sealed record MeshDataEntry(long WriteUtcTicks, List<double[]> Vertices, List<int> Indices);

    private static readonly ConcurrentDictionary<string, MeshDataEntry> MeshDataCache = new(StringComparer.OrdinalIgnoreCase);

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
        ReadCachedMeshData(path, uniformScale, () => StlReader.Read(path, uniformScale));

    public static (List<double[]> vertices, List<int> indices) ReadCachedDae(
        string path,
        double uniformScale,
        Func<(List<double[]> vertices, List<int> indices)> read)
    {
        if (!File.Exists(path))
            return (new List<double[]>(), new List<int>());
        return ReadCachedMeshData(path, uniformScale, read);
    }

    private static (List<double[]> vertices, List<int> indices) ReadCachedMeshData(
        string path,
        double uniformScale,
        Func<(List<double[]> vertices, List<int> indices)> read)
    {
        var full = Path.GetFullPath(path);
        var ticks = File.GetLastWriteTimeUtc(full).Ticks;
        var key = $"{full}|{uniformScale:G17}";
        if (MeshDataCache.TryGetValue(key, out var hit) && hit.WriteUtcTicks == ticks)
            return CloneMeshData(hit.Vertices, hit.Indices);

        var data = read();
        MeshDataCache[key] = new MeshDataEntry(ticks, data.vertices, data.indices);
        return CloneMeshData(data.vertices, data.indices);
    }

    private static (List<double[]> vertices, List<int> indices) CloneMeshData(
        List<double[]> vertices,
        List<int> indices)
    {
        var verts = new List<double[]>(vertices.Count);
        foreach (var v in vertices)
            verts.Add([v[0], v[1], v[2]]);
        return (verts, indices.ToList());
    }
}
