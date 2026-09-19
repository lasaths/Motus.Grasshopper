using System.Collections.Concurrent;
using Motus.GH.Loaders;

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

        var normalized = path.Replace('/', Path.DirectorySeparatorChar).Replace('\\', Path.DirectorySeparatorChar);

        // Stale machine-absolute paths (CI agent / other clones): recover repo-relative suffix.
        var recovered = TryRecoverRepoRelativeSuffix(path);
        if (recovered is not null)
            normalized = recovered;

        if (normalized.StartsWith("resources" + Path.DirectorySeparatorChar, StringComparison.OrdinalIgnoreCase))
        {
            var bundled = BundledToolLoader.ResolveBundledPath(normalized);
            if (File.Exists(bundled))
                return Path.GetFullPath(bundled);
        }

        var fromDoc = TryResolveBesideGrasshopperDocument(normalized);
        if (fromDoc is not null)
            return fromDoc;

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

    /// <summary>
    /// Resolve relative to the active .gh/.ghx directory, then walk parents (repo root when
    /// the definition lives under <c>examples/</c>).
    /// </summary>
    private static string? TryResolveBesideGrasshopperDocument(string normalized)
    {
        try
        {
            var filePath = Grasshopper.Instances.ActiveCanvas?.Document?.FilePath;
            if (string.IsNullOrWhiteSpace(filePath))
                return null;

            var docDir = Path.GetDirectoryName(filePath);
            if (string.IsNullOrWhiteSpace(docDir))
                return null;

            for (var i = 0; i < 8 && docDir is not null; i++)
            {
                var candidate = Path.GetFullPath(Path.Combine(docDir, normalized));
                if (File.Exists(candidate))
                    return candidate;
                docDir = Directory.GetParent(docDir)?.FullName;
            }
        }
        catch
        {
            // Grasshopper not ready / headless.
        }

        return null;
    }

    /// <summary>
    /// If <paramref name="path"/> is an absolute path from another machine that still contains
    /// a <c>resources/</c> or <c>examples/</c> segment, return that suffix for portable resolution.
    /// </summary>
    private static string? TryRecoverRepoRelativeSuffix(string path)
    {
        var unix = path.Replace('\\', '/');
        foreach (var marker in new[] { "/resources/", "/examples/" })
        {
            var idx = unix.IndexOf(marker, StringComparison.OrdinalIgnoreCase);
            if (idx < 0)
                continue;
            var suffix = unix[(idx + 1)..]; // drop leading slash → resources/... or examples/...
            return suffix.Replace('/', Path.DirectorySeparatorChar);
        }

        return null;
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
