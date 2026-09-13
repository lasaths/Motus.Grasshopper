using System.Drawing;
using System.Drawing.Drawing2D;
using Grasshopper.Kernel.Attributes;

namespace Motus.GH.UI;

/// <summary>Shared canvas-layout helpers for Motus custom attributes.</summary>
internal static class GhLayoutUtils
{
    /// <summary>
    /// Widen <paramref name="attrs"/> to <paramref name="minWidth"/> and keep params centred.
    /// </summary>
    internal static void FixLayout(GH_ComponentAttributes attrs, float minWidth)
    {
        var delta = minWidth - attrs.Bounds.Width;
        if (delta <= 0) return;

        attrs.Bounds = new RectangleF(attrs.Bounds.X - delta / 2f, attrs.Bounds.Y, attrs.Bounds.Width + delta, attrs.Bounds.Height);
        foreach (var p in attrs.Owner.Params.Output)
        {
            p.Attributes.Pivot = new PointF(p.Attributes.Pivot.X + delta / 2f, p.Attributes.Pivot.Y);
            var b = p.Attributes.Bounds;
            p.Attributes.Bounds = new RectangleF(b.X + delta / 2f, b.Y, b.Width, b.Height);
        }
        foreach (var p in attrs.Owner.Params.Input)
        {
            p.Attributes.Pivot = new PointF(p.Attributes.Pivot.X - delta / 2f, p.Attributes.Pivot.Y);
            var b = p.Attributes.Bounds;
            p.Attributes.Bounds = new RectangleF(b.X - delta / 2f, b.Y, b.Width, b.Height);
        }
    }

    /// <summary>Rounded-rectangle <see cref="GraphicsPath"/>. Caller is responsible for disposing.</summary>
    internal static GraphicsPath RoundedRect(RectangleF b, int r)
    {
        var path = new GraphicsPath();
        if (r <= 0) { path.AddRectangle(b); return path; }
        var d = r * 2f;
        path.AddArc(b.X, b.Y, d, d, 180, 90);
        path.AddArc(b.Right - d, b.Y, d, d, 270, 90);
        path.AddArc(b.Right - d, b.Bottom - d, d, d, 0, 90);
        path.AddArc(b.X, b.Bottom - d, d, d, 90, 90);
        path.CloseFigure();
        return path;
    }
}
