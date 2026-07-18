using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;
using Grasshopper.GUI;
using Grasshopper.GUI.Canvas;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Attributes;

namespace Motus.GH.UI;

/// <summary>
/// On-component dropdown strip (Arup Custom-Grasshopper-UI-Components pattern).
/// Content is supplied each layout via <paramref name="getModel"/> so pin morph can change dropdown count.
/// </summary>
public sealed class DropDownAttributes : GH_ComponentAttributes
{
    public readonly record struct Model(
        IReadOnlyList<string> Spacers,
        IReadOnlyList<IReadOnlyList<string>> Lists,
        IReadOnlyList<string> Selected);

    private readonly Func<Model> _getModel;
    private readonly Action<int, int> _onSelect;

    private readonly List<RectangleF> _spacers = [];
    private readonly List<RectangleF> _borders = [];
    private readonly List<RectangleF> _buttons = [];
    private readonly List<RectangleF> _dropdownAreas = [];
    private readonly List<List<RectangleF>> _itemBounds = [];
    private readonly List<bool> _open = [];

    private int _activeList = -1;

    public DropDownAttributes(GH_Component owner, Func<Model> getModel, Action<int, int> onSelect) : base(owner)
    {
        _getModel = getModel;
        _onSelect = onSelect;
    }

    protected override void Layout()
    {
        base.Layout();
        var model = _getModel();
        EnsureCapacity(model.Lists.Count);
        FixLayout(MinWidth(model));

        const int s = 2;
        const int hSpacer = 10;
        const int hRow = 16;
        var y = Bounds.Bottom;

        for (var i = 0; i < model.Lists.Count; i++)
        {
            var hasSpacer = i < model.Spacers.Count && !string.IsNullOrEmpty(model.Spacers[i]);
            if (hasSpacer)
            {
                _spacers[i] = new RectangleF(Bounds.X, y + s / 2f, Bounds.Width, hSpacer);
                y += hSpacer + s;
            }
            else
                _spacers[i] = RectangleF.Empty;

            _borders[i] = new RectangleF(Bounds.X + 2 * s, y + s, Bounds.Width - 4 * s - 2, hRow);
            _buttons[i] = new RectangleF(_borders[i].Right - hRow, _borders[i].Y, hRow, hRow);
            y += hRow + 3 * s;

            if (_open[i])
            {
                var list = model.Lists[i];
                _itemBounds[i].Clear();
                for (var j = 0; j < list.Count; j++)
                    _itemBounds[i].Add(new RectangleF(_borders[i].X, y + j * hRow, _borders[i].Width, hRow));
                _dropdownAreas[i] = new RectangleF(_borders[i].X, y, _borders[i].Width, list.Count * hRow);
                y += _dropdownAreas[i].Height + s;
            }
            else
            {
                _itemBounds[i].Clear();
                _dropdownAreas[i] = RectangleF.Empty;
            }
        }

        Bounds = new RectangleF(Bounds.X, Bounds.Y, Bounds.Width, y - Bounds.Y + s);
    }

    // Same Motus green as ButtonAttributes Plan chrome.
    private static readonly Color Accent = Color.FromArgb(0x2E, 0xA0, 0x43);
    private static readonly Color AccentDark = Color.FromArgb(0x1E, 0x6B, 0x2C);
    private static readonly Color MenuFill = Color.FromArgb(0xEC, 0xF8, 0xEF);

    protected override void Render(GH_Canvas canvas, Graphics graphics, GH_CanvasChannel channel)
    {
        base.Render(canvas, graphics, channel);
        if (channel != GH_CanvasChannel.Objects) return;

        var model = _getModel();
        for (var i = 0; i < model.Lists.Count; i++)
        {
            if (!_spacers[i].IsEmpty)
            {
                graphics.DrawString(
                    model.Spacers[i],
                    GH_FontServer.Small,
                    Brushes.Gray,
                    _spacers[i],
                    GH_TextRenderingConstants.NearCenter);
            }

            var selected = i < model.Selected.Count ? model.Selected[i] : string.Empty;
            using (var path = RoundedRect(_borders[i], 2))
            using (var fill = new SolidBrush(Accent))
            using (var pen = new Pen(AccentDark, 0.8f))
            {
                graphics.FillPath(fill, path);
                graphics.DrawPath(pen, path);
            }

            var textBounds = new RectangleF(_borders[i].X + 3, _borders[i].Y, _borders[i].Width - _buttons[i].Width - 4, _borders[i].Height);
            graphics.DrawString(selected, GH_FontServer.Standard, Brushes.White, textBounds, GH_TextRenderingConstants.NearCenter);

            var midX = _buttons[i].X + _buttons[i].Width / 2f;
            var midY = _buttons[i].Y + _buttons[i].Height / 2f;
            var dir = _open[i] ? -1f : 1f;
            using (var pen = new Pen(Color.White, 1.2f))
            {
                graphics.DrawLines(pen, new[]
                {
                    new PointF(midX - 3, midY - dir),
                    new PointF(midX, midY + dir * 2),
                    new PointF(midX + 3, midY - dir)
                });
            }

            if (!_open[i] || _dropdownAreas[i].IsEmpty) continue;

            using (var fill = new SolidBrush(MenuFill))
            using (var pen = new Pen(AccentDark, 0.8f))
            {
                graphics.FillRectangle(fill, _dropdownAreas[i]);
                graphics.DrawRectangle(pen, Rectangle.Round(_dropdownAreas[i]));
            }

            var list = model.Lists[i];
            for (var j = 0; j < list.Count && j < _itemBounds[i].Count; j++)
            {
                var item = _itemBounds[i][j];
                var isSel = string.Equals(list[j], selected, StringComparison.OrdinalIgnoreCase);
                if (isSel)
                {
                    using var hi = new SolidBrush(Accent);
                    graphics.FillRectangle(hi, item);
                }
                graphics.DrawString(
                    list[j],
                    GH_FontServer.Standard,
                    isSel ? Brushes.White : Brushes.Black,
                    new RectangleF(item.X + 4, item.Y, item.Width - 6, item.Height),
                    GH_TextRenderingConstants.NearCenter);
            }
        }
    }

    public override GH_ObjectResponse RespondToMouseDown(GH_Canvas sender, GH_CanvasMouseEvent e)
    {
        if (e.Button != MouseButtons.Left)
            return base.RespondToMouseDown(sender, e);

        var model = _getModel();
        for (var i = 0; i < model.Lists.Count; i++)
        {
            if (_open[i])
            {
                for (var j = 0; j < _itemBounds[i].Count; j++)
                {
                    if (!_itemBounds[i][j].Contains(e.CanvasLocation)) continue;
                    CloseAll();
                    _onSelect(i, j);
                    Owner.OnDisplayExpired(true);
                    return GH_ObjectResponse.Handled;
                }
            }

            if (_borders[i].Contains(e.CanvasLocation) || _buttons[i].Contains(e.CanvasLocation))
            {
                var wasOpen = _open[i];
                CloseAll();
                _open[i] = !wasOpen;
                _activeList = _open[i] ? i : -1;
                Owner.ExpireSolution(true);
                return GH_ObjectResponse.Handled;
            }
        }

        if (_activeList >= 0)
        {
            CloseAll();
            Owner.OnDisplayExpired(true);
            return GH_ObjectResponse.Handled;
        }

        return base.RespondToMouseDown(sender, e);
    }

    private void CloseAll()
    {
        for (var i = 0; i < _open.Count; i++)
            _open[i] = false;
        _activeList = -1;
    }

    private void EnsureCapacity(int n)
    {
        while (_spacers.Count < n) _spacers.Add(RectangleF.Empty);
        while (_borders.Count < n) _borders.Add(RectangleF.Empty);
        while (_buttons.Count < n) _buttons.Add(RectangleF.Empty);
        while (_dropdownAreas.Count < n) _dropdownAreas.Add(RectangleF.Empty);
        while (_itemBounds.Count < n) _itemBounds.Add([]);
        while (_open.Count < n) _open.Add(false);
        if (_spacers.Count > n)
        {
            _spacers.RemoveRange(n, _spacers.Count - n);
            _borders.RemoveRange(n, _borders.Count - n);
            _buttons.RemoveRange(n, _buttons.Count - n);
            _dropdownAreas.RemoveRange(n, _dropdownAreas.Count - n);
            _itemBounds.RemoveRange(n, _itemBounds.Count - n);
            _open.RemoveRange(n, _open.Count - n);
        }
    }

    private static float MinWidth(Model model)
    {
        float max = 96;
        foreach (var list in model.Lists)
        {
            foreach (var item in list)
                max = Math.Max(max, GH_FontServer.StringWidth(item, GH_FontServer.Standard) + 28);
        }
        foreach (var spacer in model.Spacers)
        {
            if (!string.IsNullOrEmpty(spacer))
                max = Math.Max(max, GH_FontServer.StringWidth(spacer, GH_FontServer.Small) + 12);
        }
        return max;
    }

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
