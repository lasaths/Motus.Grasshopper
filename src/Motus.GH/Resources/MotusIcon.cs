using System.Drawing;
using System.Drawing.Drawing2D;
using System.Reflection;

namespace Motus.GH.Resources;

/// <summary>
/// Motus brand palette (lime / emerald / dark teal / periwinkle / lavender / peach).
/// </summary>
internal static class MotusPalette
{
    /// <summary>Brand emerald — Model / primary accent. #00DB87</summary>
    public static readonly Color Model = Color.FromArgb(0x00, 0xDB, 0x87);

    /// <summary>Periwinkle — Plan. #787DFA</summary>
    public static readonly Color Plan = Color.FromArgb(0x78, 0x7D, 0xFA);

    /// <summary>Peach — Collision (board). #F7D3C2</summary>
    public static readonly Color Peach = Color.FromArgb(0xF7, 0xD3, 0xC2);

    /// <summary>Lavender — Preview (board). #CCB6F4</summary>
    public static readonly Color Lavender = Color.FromArgb(0xCC, 0xB6, 0xF4);

    /// <summary>Lime — Export / punch accent. #AFFC41</summary>
    public static readonly Color Export = Color.FromArgb(0xAF, 0xFC, 0x41);

    /// <summary>Dark teal — UI chrome / inactive rails. #0A2E33</summary>
    public static readonly Color Chrome = Color.FromArgb(0x0A, 0x2E, 0x33);

    /// <summary>Peach darkened toward chrome for 24×24 icon legibility on light GH canvas.</summary>
    public static readonly Color Collision = Mix(Peach, Chrome, 0.28f);

    /// <summary>Lavender darkened toward chrome for 24×24 icon legibility.</summary>
    public static readonly Color Preview = Mix(Lavender, Chrome, 0.22f);

    /// <summary>Soft emerald wash for dropdown menus.</summary>
    public static readonly Color MenuFill = Color.FromArgb(0xE6, 0xFB, 0xF2);

    public static Color Mix(Color a, Color b, float t)
    {
        t = Math.Clamp(t, 0f, 1f);
        return Color.FromArgb(
            (int)(a.R + (b.R - a.R) * t),
            (int)(a.G + (b.G - a.G) * t),
            (int)(a.B + (b.B - a.B) * t));
    }
}

internal static class MotusIcon
{
    private static readonly Dictionary<string, Bitmap> Cache = new(StringComparer.OrdinalIgnoreCase);

    /// <summary>Phosphor icon tint per Grasshopper subcategory.</summary>
    public static Color SubcategoryColor(string subcategory) => subcategory switch
    {
        "Model" => MotusPalette.Model,           // #00DB87
        "Plan" => MotusPalette.Plan,             // #787DFA
        "Collision" => MotusPalette.Collision,   // peach → chrome
        "Preview" => MotusPalette.Preview,       // lavender → chrome
        "Export" => MotusPalette.Export,         // #AFFC41
        _ => MotusPalette.Model,
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
