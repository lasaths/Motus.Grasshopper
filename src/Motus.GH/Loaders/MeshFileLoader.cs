using Motus.Presets;
using Rhino.Geometry;
using System.Collections.Concurrent;
using System.Drawing;

namespace Motus.GH.Loaders;

internal static class MeshFileLoader
{
    private sealed record MeshDataEntry(long WriteUtcTicks, List<double[]> Vertices, List<int> Indices);
    private sealed record DaePartsEntry(long WriteUtcTicks, List<(List<double[]> Vertices, List<int> Indices, Color? Color)> Parts);

    private static readonly ConcurrentDictionary<string, MeshDataEntry> MeshDataCache = new(StringComparer.OrdinalIgnoreCase);
    private static readonly ConcurrentDictionary<string, DaePartsEntry> DaePartsCache = new(StringComparer.OrdinalIgnoreCase);

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

    public static List<(List<double[]> vertices, List<int> indices, Color? color)> ReadCachedDaeParts(
        string path,
        double uniformScale,
        Func<List<(List<double[]> vertices, List<int> indices, Color? color)>> read)
    {
        if (!File.Exists(path))
            return [];

        var full = Path.GetFullPath(path);
        var ticks = File.GetLastWriteTimeUtc(full).Ticks;
        var key = $"{full}|{uniformScale:G17}|parts";
        if (DaePartsCache.TryGetValue(key, out var hit) && hit.WriteUtcTicks == ticks)
            return CloneDaeParts(hit.Parts);

        var data = read();
        DaePartsCache[key] = new DaePartsEntry(ticks, data);
        return CloneDaeParts(data);
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

    private static List<(List<double[]> vertices, List<int> indices, Color? color)> CloneDaeParts(
        List<(List<double[]> Vertices, List<int> Indices, Color? Color)> parts)
    {
        var clone = new List<(List<double[]>, List<int>, Color?)>(parts.Count);
        foreach (var part in parts)
        {
            var (verts, indices) = CloneMeshData(part.Vertices, part.Indices);
            clone.Add((verts, indices, part.Color));
        }
        return clone;
    }
}
