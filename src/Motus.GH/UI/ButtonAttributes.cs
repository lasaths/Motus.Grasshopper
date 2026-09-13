using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;
using Grasshopper;
using Grasshopper.GUI;
using Grasshopper.GUI.Canvas;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Attributes;
using Motus.GH.Resources;

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
    // Measured once on first Layout; these strings are constant so the width never changes.
    private float _cachedDesiredWidth;

    public ButtonAttributes(GH_Component owner, Func<string> label, Func<bool> isActive, Action onClick) : base(owner)
        => (_label, _isActive, _onClick) = (label, isActive, onClick);

    private float DesiredWidth()
    {
        if (_cachedDesiredWidth > 0) return _cachedDesiredWidth;
        try
        {
            var idle = GH_FontServer.StringWidth("\u25B6 Play", GH_FontServer.Standard);
            var active = GH_FontServer.StringWidth("\u25A0 Stop", GH_FontServer.Standard);
            var replan = GH_FontServer.StringWidth("Replan", GH_FontServer.Standard);
            _cachedDesiredWidth = Math.Max(Math.Max(idle, active), replan) + 24;
        }
        catch
        {
            // ponytail: GH_FontServer can NRE during GHA registration before UI fonts exist
            _cachedDesiredWidth = 96;
        }
        return _cachedDesiredWidth;
    }

    protected override void Layout()
    {
        base.Layout();
        GhLayoutUtils.FixLayout(this, DesiredWidth());

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
        // Emerald = ready; dark teal = active/playing. Avoid red (reads as a GH error state).
        var baseColor = active ? MotusPalette.Chrome : MotusPalette.Model;
        var fill = _mouseDown ? Darken(baseColor, 0.18) : _mouseOver ? Lighten(baseColor, 0.12) : baseColor;

        using var path = GhLayoutUtils.RoundedRect(_buttonBounds, 3);
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

    private static Color Lighten(Color c, double r) =>
        Color.FromArgb(c.A, (int)(c.R + (255 - c.R) * r), (int)(c.G + (255 - c.G) * r), (int)(c.B + (255 - c.B) * r));

    private static Color Darken(Color c, double r) =>
        Color.FromArgb(c.A, (int)(c.R * (1 - r)), (int)(c.G * (1 - r)), (int)(c.B * (1 - r)));


}
