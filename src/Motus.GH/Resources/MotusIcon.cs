using System.Reflection;

namespace Motus.GH.Resources;

internal static class MotusIcon
{
    private static readonly Dictionary<string, System.Drawing.Bitmap> Cache = new(StringComparer.OrdinalIgnoreCase);

    public static System.Drawing.Bitmap Get(string iconName)
    {
        var key = $"{iconName}-bold";
        if (Cache.TryGetValue(key, out var cached)) return cached;

        var asm = Assembly.GetExecutingAssembly();
        var suffix = $"{iconName}-bold.png";
        var resourceName = asm.GetManifestResourceNames()
            .FirstOrDefault(n => n.EndsWith(suffix, StringComparison.OrdinalIgnoreCase));

        if (resourceName is null)
            throw new InvalidOperationException($"Phosphor icon resource not found: {suffix}");

        using var stream = asm.GetManifestResourceStream(resourceName)!;
        var bmp = new System.Drawing.Bitmap(stream);
        Cache[key] = bmp;
        return bmp;
    }
}
