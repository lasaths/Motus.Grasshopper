using System.Collections.Concurrent;

namespace Motus.GH.Urdf;

internal static class UrdfPathResolver
{
    private static readonly ConcurrentDictionary<string, string> ResolvedCache =
        new(StringComparer.OrdinalIgnoreCase);

    public static string ResolveUrdfPath(string path)
    {
        if (string.IsNullOrWhiteSpace(path)) return path;
        if (IsRemotePath(path))
            throw new ArgumentException("Remote asset paths are not supported.", nameof(path));

        if (ResolvedCache.TryGetValue(path, out var cached) && File.Exists(cached))
            return cached;

        var resolved = ResolveUncached(path);
        if (File.Exists(resolved))
            ResolvedCache[path] = resolved;
        return resolved;
    }

    private static string ResolveUncached(string path)
    {
        if (File.Exists(path))
            return Path.GetFullPath(path);

        var normalized = path.Replace('/', Path.DirectorySeparatorChar);
        var dir = Directory.GetCurrentDirectory();
        for (var i = 0; i < 10 && dir is not null; i++)
        {
            var candidate = Path.Combine(dir, normalized);
            if (File.Exists(candidate))
                return Path.GetFullPath(candidate);
            dir = Directory.GetParent(dir)?.FullName;
        }

        return path;
    }

    private static bool IsRemotePath(string path)
    {
        if (path.StartsWith(@"\\", StringComparison.Ordinal) ||
            path.StartsWith("//", StringComparison.Ordinal))
            return true;

        return path.Contains("://", StringComparison.Ordinal) &&
               Uri.TryCreate(path, UriKind.Absolute, out var uri) &&
               !uri.IsFile &&
               !string.IsNullOrEmpty(uri.Scheme);
    }
}
