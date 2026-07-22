using System.Drawing;
using System.Drawing.Drawing2D;
using Grasshopper.GUI;
using Grasshopper.GUI.Canvas;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Special;
using Motus.GH.Components;
using Motus.GH.Params;
using Motus.GH.Preview;
using Motus.GH.Resources;

namespace Motus.GH.UI;

/// <summary>
/// Motus Scrub: trajectory-aware readout, keyframe ticks, magnetic snap.
/// </summary>
public sealed class MotusScrubSliderAttributes : GH_NumberSliderAttributes
{
    private static readonly Color Accent = MotusPalette.Model;
    private static readonly Color TickColor = Color.FromArgb(78, MotusPalette.Chrome);
    private static readonly Color TickActive = Color.FromArgb(180, Accent);

    public MotusScrubSliderAttributes(MotusScrubSlider owner) : base(owner) { }

    private MotusScrubSlider ScrubOwner => (MotusScrubSlider)Owner;

    protected override void Layout()
    {
        // ponytail: lock 0–1 so Value=1 sits at rail end (GH lets users widen Max)
        var slider = ScrubOwner.Slider;
        if (slider.Minimum != 0) slider.Minimum = 0;
        if (slider.Maximum != 1) slider.Maximum = 1;

        base.Layout();
        const float labelH = 14f;
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
        if (ScrubOwner.IsDragging)
        {
            if (ScrubOwner.SnapToKeyframes)
                ScrubOwner.ApplyKeyframeSnap(TrackBounds().Width, 24f);
            ExpireDownstreamPreview(true);
        }
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
        var labelBounds = new RectangleF(Bounds.X + 4, Bounds.Y + 1, Bounds.Width - 8, 12);
        using var format = new StringFormat
        {
            Alignment = StringAlignment.Near,
            LineAlignment = StringAlignment.Center,
            FormatFlags = StringFormatFlags.NoWrap,
            Trimming = StringTrimming.EllipsisCharacter,
        };
        // ponytail: Standard is huge on the 12px strip — Small fits the label row
        var text = FormatHeader(timeline, t);
        graphics.DrawString(text, GH_FontServer.Small, Brushes.DimGray, labelBounds, format);
    }

    private static string FormatHeader(ScrubTimeline timeline, double t)
    {
        if (timeline.IsEmpty)
            return $"{t:0.000} · wire Preview for keyframes";
        var idx = timeline.NearestDisplayIndex(t);
        var time = timeline.TimeAt(t);
        var dur = timeline.DurationSeconds;
        return $"{t:0.000} · {time:0.00}/{dur:0.00}s · kf {idx + 1}/{timeline.Count}";
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

    private void ExpireDownstreamPreview(bool recompute)
    {
        // During drag: preview-only update (no full graph ExpireSolution).
        if (ScrubOwner.IsDragging &&
            MotusPreviewComponent.TryNotifyScrubDrag(ScrubOwner, (double)ScrubOwner.Slider.Value))
        {
            ScrubOwner.OnDisplayExpired(false);
            return;
        }

        ScrubOwner.ExpireSolution(recompute);
    }
}
