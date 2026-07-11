using System.Collections.Concurrent;

namespace Motus.GH.Urdf;

internal static class UrdfWriteTimeCache
{
    private static readonly ConcurrentDictionary<string, (long Ticks, DateTime CheckedAt)> Cache =
        new(StringComparer.OrdinalIgnoreCase);

    private static readonly TimeSpan RefreshInterval = TimeSpan.FromSeconds(1);

    public static long GetTicks(string path)
    {
        var full = Path.GetFullPath(path);
        if (Cache.TryGetValue(full, out var entry) && DateTime.UtcNow - entry.CheckedAt < RefreshInterval)
            return entry.Ticks;

        var ticks = File.Exists(full) ? File.GetLastWriteTimeUtc(full).Ticks : 0;
        Cache[full] = (ticks, DateTime.UtcNow);
        return ticks;
    }
}
