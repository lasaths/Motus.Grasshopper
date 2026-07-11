using System.Drawing;
using System.Globalization;
using System.Xml.Linq;

/// <summary>Mirror of Motus.GH.Urdf.UrdfMaterialParser for headless QA (keep in sync).</summary>
internal static class UrdfMaterialSmoke
{
    public static Dictionary<string, Color> ParseRobotMaterials(XElement robotRoot)
    {
        var materials = new Dictionary<string, Color>(StringComparer.OrdinalIgnoreCase);
        foreach (var material in robotRoot.Elements("material"))
        {
            var name = material.Attribute("name")?.Value;
            if (string.IsNullOrWhiteSpace(name)) continue;
            if (TryParseMaterialColor(material) is { } color)
                materials[name] = color;
        }
        return materials;
    }

    public static Color? ResolveVisualColor(XElement visual, IReadOnlyDictionary<string, Color> materials)
    {
        var material = visual.Element("material");
        if (material is null) return null;
        var name = material.Attribute("name")?.Value;
        if (!string.IsNullOrWhiteSpace(name))
        {
            var key = name.StartsWith('#') ? name[1..] : name;
            if (materials.TryGetValue(key, out var named))
                return named;
        }
        return TryParseMaterialColor(material);
    }

    private static Color? TryParseMaterialColor(XElement materialEl)
    {
        var rgba = materialEl.Element("color")?.Attribute("rgba")?.Value;
        if (string.IsNullOrWhiteSpace(rgba)) return null;
        var parts = rgba.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
        if (parts.Length < 3) return null;
        var r = ParseFloat(parts[0]);
        var g = ParseFloat(parts[1]);
        var b = ParseFloat(parts[2]);
        var a = parts.Length > 3 ? ParseFloat(parts[3]) : 1.0;
        return Color.FromArgb(
            (int)Math.Clamp(a * 255, 0, 255),
            (int)Math.Clamp(r * 255, 0, 255),
            (int)Math.Clamp(g * 255, 0, 255),
            (int)Math.Clamp(b * 255, 0, 255));
    }

    private static double ParseFloat(string s) =>
        double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var v) ? v : 0;
}
