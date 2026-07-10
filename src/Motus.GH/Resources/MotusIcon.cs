using System.Drawing;
using System.Drawing.Drawing2D;
using System.Reflection;

namespace Motus.GH.Resources;

internal static class MotusIcon
{
    private static readonly Dictionary<string, Bitmap> Cache = new(StringComparer.OrdinalIgnoreCase);

    /// <summary>Phosphor icon tint per Grasshopper subcategory.</summary>
    public static Color SubcategoryColor(string subcategory) => subcategory switch
    {
        "Model" => Color.FromArgb(0, 196, 154),      // #00c49a
        "Plan" => Color.FromArgb(14, 165, 233),        // #0ea5e9
        "Collision" => Color.FromArgb(249, 115, 22),   // #f97316
        "Preview" => Color.FromArgb(168, 85, 247),     // #a855f7
        "Export" => Color.FromArgb(234, 179, 8),       // #eab308
        _ => Color.FromArgb(0, 196, 154),
    };

    /// <summary>24×24 duotone brand icon for <see cref="GH_AssemblyInfo"/>.</summary>
    public static Bitmap GetAssembly() => Get("robot", SubcategoryColor("Model"));

    /// <summary>16×16 duotone brand icon for the Motus component ribbon tab.</summary>
    public static Bitmap GetCategoryTab()
    {
        const string key = "tab:robot:model";
        if (Cache.TryGetValue(key, out var cached)) return cached;
        var tab = Resize(GetAssembly(), 16);
        Cache[key] = tab;
        return tab;
    }

    public static Bitmap Get(string iconName, Color? tint = null)
    {
        var cacheKey = tint is { } c ? $"{iconName}:{c.ToArgb()}" : iconName;
        if (Cache.TryGetValue(cacheKey, out var cached)) return cached;

        var resourceName = $"Motus.GH.Resources.icons.{iconName}-duotone.png";
        using var stream = Assembly.GetExecutingAssembly().GetManifestResourceStream(resourceName)
            ?? throw new InvalidOperationException($"Phosphor icon resource not found: {resourceName}");
        using var raw = new Bitmap(stream);
        var bmp = tint is { } color ? Recolor(raw, color) : new Bitmap(raw);
        Cache[cacheKey] = bmp;
        return bmp;
    }

    private static Bitmap Resize(Bitmap source, int size)
    {
        var bmp = new Bitmap(size, size, System.Drawing.Imaging.PixelFormat.Format32bppArgb);
        using var g = Graphics.FromImage(bmp);
        g.InterpolationMode = InterpolationMode.HighQualityBicubic;
        g.PixelOffsetMode = PixelOffsetMode.HighQuality;
        g.DrawImage(source, 0, 0, size, size);
        return bmp;
    }

    private static Bitmap Recolor(Bitmap source, Color color)
    {
        var bmp = new Bitmap(source.Width, source.Height, System.Drawing.Imaging.PixelFormat.Format32bppArgb);
        for (var y = 0; y < source.Height; y++)
        for (var x = 0; x < source.Width; x++)
        {
            var px = source.GetPixel(x, y);
            bmp.SetPixel(x, y, px.A == 0 ? Color.Transparent : Color.FromArgb(px.A, color));
        }
        return bmp;
    }
}
