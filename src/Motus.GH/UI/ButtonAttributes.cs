using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;
using Grasshopper;
using Grasshopper.GUI;
using Grasshopper.GUI.Canvas;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Attributes;

namespace Motus.GH.UI;

/// <summary>
/// Canvas button rendered below a component (pattern adapted from Arup Custom-Grasshopper-UI-Components).
/// Label and colour are state-aware via the supplied providers so the button can reflect play/stop.
/// </summary>
public sealed class ButtonAttributes : GH_ComponentAttributes
{
    private readonly Func<string> _label;
    private readonly Func<bool> _isActive;
    private readonly Action _onClick;
    private RectangleF _buttonBounds;
    private bool _mouseDown;
    private bool _mouseOver;

    public ButtonAttributes(GH_Component owner, Func<string> label, Func<bool> isActive, Action onClick) : base(owner)
        => (_label, _isActive, _onClick) = (label, isActive, onClick);

    private float DesiredWidth()
    {
        var idle = GH_FontServer.StringWidth("\u25B6 Play", GH_FontServer.Standard);
        var active = GH_FontServer.StringWidth("\u25A0 Stop", GH_FontServer.Standard);
        return Math.Max(idle, active) + 24;
    }

    protected override void Layout()
    {
        base.Layout();
        FixLayout(DesiredWidth());

        const int pad = 3;
        const int h = 22;
        _buttonBounds = new RectangleF(Bounds.X + 2 * pad, Bounds.Bottom + pad, Bounds.Width - 4 * pad, h);
        Bounds = new RectangleF(Bounds.X, Bounds.Y, Bounds.Width, Bounds.Height + h + 2 * pad);
    }

    protected override void Render(GH_Canvas canvas, Graphics graphics, GH_CanvasChannel channel)
    {
        base.Render(canvas, graphics, channel);
        if (channel != GH_CanvasChannel.Objects) return;

        var active = _isActive();
        // Green = ready to play, amber = currently playing. Avoid red (reads as a GH error state).
        var baseColor = active ? Color.FromArgb(0xF5, 0x9E, 0x0B) : Color.FromArgb(0x2E, 0xA0, 0x43);
        var fill = _mouseDown ? Darken(baseColor, 0.18) : _mouseOver ? Lighten(baseColor, 0.12) : baseColor;

        using var path = RoundedRect(_buttonBounds, 3);
        using var brush = new SolidBrush(fill);
        graphics.FillPath(brush, path);
        using var pen = new Pen(Darken(baseColor, 0.3), _mouseDown ? 1.0f : 0.6f);
        graphics.DrawPath(pen, path);
        graphics.DrawString(_label(), GH_FontServer.Standard, Brushes.White, _buttonBounds, GH_TextRenderingConstants.CenterCenter);
    }

    public override GH_ObjectResponse RespondToMouseDown(GH_Canvas sender, GH_CanvasMouseEvent e)
    {
        if (e.Button == MouseButtons.Left && _buttonBounds.Contains(e.CanvasLocation))
        {
            _mouseDown = true;
            Owner.OnDisplayExpired(false);
            return GH_ObjectResponse.Capture;
        }
        return base.RespondToMouseDown(sender, e);
    }

    public override GH_ObjectResponse RespondToMouseUp(GH_Canvas sender, GH_CanvasMouseEvent e)
    {
        // Only act when we own the press. Always Release here so the Capture taken in
        // RespondToMouseDown is balanced even if the cursor moved off the button.
        if (e.Button == MouseButtons.Left && _mouseDown)
        {
            var clicked = _buttonBounds.Contains(e.CanvasLocation);
            _mouseDown = false;
            _mouseOver = false;
            Owner.OnDisplayExpired(false);
            if (clicked) _onClick();
            return GH_ObjectResponse.Release;
        }
        _mouseDown = false;
        return base.RespondToMouseUp(sender, e);
    }

    public override GH_ObjectResponse RespondToMouseMove(GH_Canvas sender, GH_CanvasMouseEvent e)
    {
        // Balanced Capture/Release: grab the mouse when entering the button, give it back
        // when leaving. Capturing on every move (or never releasing) blocks the canvas.
        var over = _buttonBounds.Contains(e.CanvasLocation);
        if (over && !_mouseOver)
        {
            _mouseOver = true;
            Owner.OnDisplayExpired(false);
            sender.Cursor = Cursors.Hand;
            return GH_ObjectResponse.Capture;
        }
        if (!over && _mouseOver)
        {
            _mouseOver = false;
            Owner.OnDisplayExpired(false);
            Instances.CursorServer.ResetCursor(sender);
            return GH_ObjectResponse.Release;
        }
        return base.RespondToMouseMove(sender, e);
    }

    /// <summary>Widen the component to <paramref name="minWidth"/> and shift the output params to stay right-aligned.</summary>
    private void FixLayout(float minWidth)
    {
        var width = Bounds.Width;
        var newWidth = Math.Max(width, minWidth);
        var delta = newWidth - width;
        if (delta <= 0) return;

        Bounds = new RectangleF(Bounds.X - delta / 2f, Bounds.Y, newWidth, Bounds.Height);
        foreach (var p in Owner.Params.Output)
        {
            p.Attributes.Pivot = new PointF(p.Attributes.Pivot.X + delta / 2f, p.Attributes.Pivot.Y);
            var b = p.Attributes.Bounds;
            p.Attributes.Bounds = new RectangleF(b.X + delta / 2f, b.Y, b.Width, b.Height);
        }
        foreach (var p in Owner.Params.Input)
        {
            p.Attributes.Pivot = new PointF(p.Attributes.Pivot.X - delta / 2f, p.Attributes.Pivot.Y);
            var b = p.Attributes.Bounds;
            p.Attributes.Bounds = new RectangleF(b.X - delta / 2f, b.Y, b.Width, b.Height);
        }
    }

    private static Color Lighten(Color c, double r) =>
        Color.FromArgb(c.A, (int)(c.R + (255 - c.R) * r), (int)(c.G + (255 - c.G) * r), (int)(c.B + (255 - c.B) * r));

    private static Color Darken(Color c, double r) =>
        Color.FromArgb(c.A, (int)(c.R * (1 - r)), (int)(c.G * (1 - r)), (int)(c.B * (1 - r)));

    private static GraphicsPath RoundedRect(RectangleF b, int r)
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
