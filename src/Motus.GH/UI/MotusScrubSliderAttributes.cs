using System.Drawing;
using System.Drawing.Drawing2D;
using Grasshopper.GUI;
using Grasshopper.GUI.Canvas;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Special;
using Motus.GH.Params;

namespace Motus.GH.UI;

/// <summary>
/// Motus Scrub: trajectory-aware readout, keyframe ticks, magnetic snap.
/// </summary>
public sealed class MotusScrubSliderAttributes : GH_NumberSliderAttributes
{
    private static readonly Color Accent = Color.FromArgb(0, 196, 154);
    private static readonly Color TickColor = Color.FromArgb(78, 120, 120, 128);
    private static readonly Color TickActive = Color.FromArgb(180, Accent);

    public MotusScrubSliderAttributes(MotusScrubSlider owner) : base(owner) { }

    private MotusScrubSlider ScrubOwner => (MotusScrubSlider)Owner;

    protected override void Layout()
    {
        base.Layout();
        const float labelH = 16f;
        var b = Bounds;
        Bounds = new RectangleF(b.X, b.Y - labelH, b.Width, b.Height + labelH);
    }

    protected override void Render(GH_Canvas canvas, Graphics graphics, GH_CanvasChannel channel)
    {
        base.Render(canvas, graphics, channel);
        if (channel != GH_CanvasChannel.Objects) return;

        var timeline = ScrubOwner.ResolveTimeline();
        var t = Math.Clamp((double)ScrubOwner.Slider.Value, 0, 1);
        var track = TrackBounds();
        if (track.Width > 4f)
            DrawKeyframeTicks(graphics, track, timeline, t);

        DrawHeader(graphics, timeline, t);
    }

    public override GH_ObjectResponse RespondToMouseDown(GH_Canvas sender, GH_CanvasMouseEvent e)
    {
        var response = base.RespondToMouseDown(sender, e);
        if (response is GH_ObjectResponse.Handled or GH_ObjectResponse.Capture)
            ScrubOwner.SetDragging(true);
        return response;
    }

    public override GH_ObjectResponse RespondToMouseMove(GH_Canvas sender, GH_CanvasMouseEvent e)
    {
        var response = base.RespondToMouseMove(sender, e);
        if (ScrubOwner.IsDragging && ScrubOwner.SnapToKeyframes)
            ScrubOwner.ApplyKeyframeSnap(TrackBounds().Width, 24f);
        return response;
    }

    public override GH_ObjectResponse RespondToMouseUp(GH_Canvas sender, GH_CanvasMouseEvent e)
    {
        var wasDragging = ScrubOwner.IsDragging;
        var response = base.RespondToMouseUp(sender, e);
        ScrubOwner.SetDragging(false);
        if (wasDragging)
        {
            if (ScrubOwner.SnapToKeyframes)
                ScrubOwner.ApplyKeyframeSnap(TrackBounds().Width, 18f, forceNearest: true);
            ExpireDownstreamPreview(true);
        }
        return response;
    }

    private RectangleF TrackBounds()
    {
        var slider = ScrubOwner.Slider;
        var rail = slider.Rail;
        if (rail.Width > 4f)
        {
            const float h = 8f;
            return new RectangleF(rail.X, rail.Y - h * 0.5f, rail.Width, h);
        }
        var b = Bounds;
        return new RectangleF(b.X + 52f, b.Bottom - 14f, Math.Max(40f, b.Width - 60f), 8f);
    }

    private void DrawHeader(Graphics graphics, ScrubTimeline timeline, double t)
    {
        var labelBounds = new RectangleF(Bounds.X + 4, Bounds.Y + 1, Bounds.Width - 8, 14);
        using var format = new StringFormat { Alignment = StringAlignment.Near, LineAlignment = StringAlignment.Center };
        var text = FormatHeader(timeline, t);
        graphics.DrawString(text, GH_FontServer.Standard, Brushes.DimGray, labelBounds, format);
    }

    private static string FormatHeader(ScrubTimeline timeline, double t)
    {
        if (timeline.IsEmpty)
            return $"{t:0.000} · wire Preview for keyframes";
        var idx = timeline.NearestDisplayIndex(t);
        var time = timeline.IsEmpty ? timeline.TimeAt(t) : timeline.WaypointTimes[idx];
        var dur = timeline.DurationSeconds;
        return $"{t * 100:0}% · {time:0.00} / {dur:0.00} s · keyframe {idx + 1}/{timeline.Count}";
    }

    private static void DrawKeyframeTicks(Graphics graphics, RectangleF track, ScrubTimeline timeline, double t)
    {
        if (timeline.IsEmpty) return;
        var nearest = timeline.NearestDisplayIndex(t);
        using var tickPen = new Pen(TickColor, 1f);
        using var activePen = new Pen(TickActive, 1.5f);
        for (var i = 0; i < timeline.Count; i++)
        {
            var frac = timeline.DisplayFractions[i];
            var x = track.X + (float)(frac * track.Width);
            var h = i == nearest ? 14f : 10f;
            var y0 = track.Y + track.Height * 0.5f - h * 0.5f;
            var pen = i == nearest ? activePen : tickPen;
            graphics.DrawLine(pen, x, y0, x, y0 + h);
        }
    }

    private void ExpireDownstreamPreview(bool recompute) => ScrubOwner.ExpireSolution(recompute);
}
