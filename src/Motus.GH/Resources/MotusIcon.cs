using System.Reflection;

namespace Motus.GH.Resources;

internal static class MotusIcon
{
    private static readonly Dictionary<string, System.Drawing.Bitmap> Cache = new(StringComparer.OrdinalIgnoreCase);

    public static System.Drawing.Bitmap Get(string iconName)
    {
        if (Cache.TryGetValue(iconName, out var cached)) return cached;

        var resourceName = $"Motus.GH.Resources.icons.{iconName}-bold.png";
        using var stream = Assembly.GetExecutingAssembly().GetManifestResourceStream(resourceName)
            ?? throw new InvalidOperationException($"Phosphor icon resource not found: {resourceName}");
        var bmp = new System.Drawing.Bitmap(stream);
        Cache[iconName] = bmp;
        return bmp;
    }
}
