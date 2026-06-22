namespace Motus.GH.Resources;

internal static class MotusIcon
{
    private static System.Drawing.Bitmap? _icon;

    public static System.Drawing.Bitmap Get()
    {
        if (_icon is not null) return _icon;
        var bmp = new System.Drawing.Bitmap(24, 24);
        using var g = System.Drawing.Graphics.FromImage(bmp);
        g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
        g.Clear(System.Drawing.Color.FromArgb(28, 32, 38));
        using var accent = new System.Drawing.Pen(System.Drawing.Color.FromArgb(0, 196, 154), 2.2f);
        using var dim = new System.Drawing.Pen(System.Drawing.Color.FromArgb(90, 110, 120), 1.6f);
        g.DrawLine(dim, 4, 19, 10, 12);
        g.DrawLine(dim, 10, 12, 16, 14);
        g.DrawLine(accent, 16, 14, 20, 8);
        g.DrawEllipse(accent, 17, 5, 5, 5);
        _icon = bmp;
        return _icon;
    }
}
