using System.Drawing;
using Motus.Core;
using Motus.Geometry;

namespace Motus.GH.Preview;

public enum PreviewColorMode
{
    Override,
    Urdf,
    Custom
}

public static class PreviewColorResolver
{
    private static readonly Color OverrideCurrent = Color.FromArgb(200, 0, 196, 154);
    private static readonly Color OverrideStart = Color.FromArgb(90, 220, 220, 220);
    private static readonly Color UrdfFallback = Color.FromArgb(200, 200, 200, 200);

    public const float CurrentTransparency = 0.2f;
    public const float StartTransparency = 0.55f;

    public static Color Resolve(
        int meshIndex,
        PreviewColorMode mode,
        IReadOnlyList<Color?>? urdfColors,
        IReadOnlyList<Color>? customColors,
        bool isStartGhost)
    {
        return mode switch
        {
            PreviewColorMode.Override => isStartGhost ? OverrideStart : OverrideCurrent,
            PreviewColorMode.Urdf => ResolveUrdf(meshIndex, urdfColors, isStartGhost),
            PreviewColorMode.Custom when customColors is { Count: > 0 } =>
                ResolveCustom(meshIndex, customColors, isStartGhost),
            PreviewColorMode.Custom => isStartGhost ? OverrideStart : OverrideCurrent,
            _ => isStartGhost ? OverrideStart : OverrideCurrent
        };
    }

    /// <summary>Map geometry-index colours to drawable mesh slots (mesh links + optional tool).</summary>
    public static Color?[] AlignMeshColors(RobotCollisionModel geometry, Color?[]? geometryColors)
    {
        if (geometryColors is null || geometryColors.Length == 0)
            return [];

        var aligned = new List<Color?>();
        for (var gi = 0; gi < geometry.Links.Count; gi++)
        {
            if (geometry.Links[gi].LocalGeometry.Shape != CollisionShape.Mesh) continue;
            aligned.Add(gi < geometryColors.Length ? geometryColors[gi] : null);
        }

        if (geometry.ToolGeometry is { Shape: CollisionShape.Mesh })
            aligned.Add(null);

        return aligned.ToArray();
    }

    private static Color ResolveUrdf(int meshIndex, IReadOnlyList<Color?>? urdfColors, bool isStartGhost)
    {
        Color baseColor;
        if (urdfColors is not null && meshIndex >= 0 && meshIndex < urdfColors.Count && urdfColors[meshIndex] is { } c)
            baseColor = c;
        else
            baseColor = UrdfFallback;

        return isStartGhost ? WithAlpha(baseColor, (int)(StartTransparency * 255)) : baseColor;
    }

    private static Color ResolveCustom(int meshIndex, IReadOnlyList<Color>? customColors, bool isStartGhost)
    {
        if (customColors is null || customColors.Count == 0)
            return isStartGhost ? OverrideStart : OverrideCurrent;

        var idx = meshIndex < customColors.Count
            ? meshIndex
            : customColors.Count - 1;
        var baseColor = customColors[idx];
        return isStartGhost ? WithAlpha(baseColor, (int)(StartTransparency * 255)) : baseColor;
    }

    private static Color WithAlpha(Color color, int alpha) =>
        Color.FromArgb(alpha, color.R, color.G, color.B);
}
