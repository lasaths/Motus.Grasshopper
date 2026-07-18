using GH_IO.Serialization;
using Grasshopper.GUI.Base;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Special;
using Motus.Core;
using Motus.GH.Preview;
using Motus.GH.Resources;
using Motus.GH.UI;
using System.Drawing;
using System.Windows.Forms;

namespace Motus.GH.Params;

/// <summary>
/// Resizable 0–1 scrub slider for Motus Preview playback. Subclasses GH_NumberSlider for native resize/capture behaviour.
/// </summary>
public sealed class MotusScrubSlider : GH_NumberSlider
{
    private static readonly Color Accent = Color.FromArgb(0, 196, 154);

    public MotusScrubSlider()
    {
        Name = "Motus Scrub";
        NickName = "Scrub";
        Description = "Normalized playback position (0–1) for Motus Preview; snaps to trajectory keyframes when wired";
        Category = "Motus";
        SubCategory = "Preview";
        ApplyScrubDefaults();
    }

    public override GH_ParamKind Kind => GH_ParamKind.floating;

    public override void CreateAttributes() => m_attributes = new MotusScrubSliderAttributes(this);

    public override Guid ComponentGuid => new("e1f2a3b4-c5d6-4789-a012-3456789abc01");

    protected override Bitmap Icon => MotusIcon.Get("sliders-horizontal", MotusIcon.SubcategoryColor("Preview"));

    internal bool SnapToKeyframes { get; private set; } = true;

    public override bool Write(GH_IWriter writer)
    {
        writer.SetDouble("ScrubValue", (double)Slider.Value);
        writer.SetBoolean("SnapToKeyframes", SnapToKeyframes);
        return base.Write(writer);
    }

    public override bool Read(GH_IReader reader)
    {
        ApplyScrubDefaults();
        if (reader.ItemExists("ScrubValue"))
            Slider.Value = (decimal)Math.Clamp(reader.GetDouble("ScrubValue"), 0, 1);
        if (reader.ItemExists("SnapToKeyframes"))
            SnapToKeyframes = reader.GetBoolean("SnapToKeyframes");
        return base.Read(reader);
    }

    public override void AppendAdditionalMenuItems(ToolStripDropDown menu)
    {
        base.AppendAdditionalMenuItems(menu);
        menu.Items.Add(new ToolStripSeparator());
        menu.Items.Add(new ToolStripMenuItem("Snap to keyframes", null, (_, _) => ToggleSnap())
        {
            Checked = SnapToKeyframes,
        });
        menu.Items.Add(new ToolStripMenuItem("Previous keyframe", null, (_, _) => StepKeyframe(-1)));
        menu.Items.Add(new ToolStripMenuItem("Next keyframe", null, (_, _) => StepKeyframe(1)));
        menu.Items.Add(new ToolStripSeparator());
        menu.Items.Add(new ToolStripMenuItem("Reset to start (0)", null, (_, _) => SetScrubValue(0)));
        menu.Items.Add(new ToolStripMenuItem("Reset to end (1)", null, (_, _) => SetScrubValue(1)));
    }

    internal bool IsDragging { get; private set; }

    internal bool IsSyncingFromPreview => _syncFromPreviewDepth > 0;

    internal void SetDragging(bool dragging) => IsDragging = dragging;

    internal void BeginSyncFromPreview() => _syncFromPreviewDepth++;

    internal void EndSyncFromPreview() => _syncFromPreviewDepth = Math.Max(0, _syncFromPreviewDepth - 1);

    private int _syncFromPreviewDepth;
    private ScrubTimeline _cachedTimeline = ScrubTimeline.Empty;
    private Trajectory? _timelineTrajectory;

    internal ScrubTimeline ResolveTimeline()
    {
        var timeline = ScrubTimelineProbe.TryResolve(this, out var trajectory);
        if (ReferenceEquals(trajectory, _timelineTrajectory) && !_cachedTimeline.IsEmpty)
            return _cachedTimeline;
        _timelineTrajectory = trajectory;
        _cachedTimeline = timeline;
        return timeline;
    }

    internal void InvalidateTimelineCache()
    {
        _timelineTrajectory = null;
        _cachedTimeline = ScrubTimeline.Empty;
    }

    internal void ApplyKeyframeSnap(float trackWidthPx, float snapPx = 12f, bool forceNearest = false)
    {
        if (!SnapToKeyframes) return;
        var timeline = ResolveTimeline();
        if (timeline.IsEmpty) return;
        var current = (double)Slider.Value;
        var snapped = forceNearest ? timeline.SnapToNearest(current) : timeline.Snap(current, trackWidthPx, snapPx);
        SetScrubValue(snapped, expireDownstream: false);
    }

    internal void StepKeyframe(int direction)
    {
        var timeline = ResolveTimeline();
        if (timeline.IsEmpty) return;
        var idx = timeline.NearestDisplayIndex((double)Slider.Value);
        var next = Math.Clamp(idx + direction, 0, timeline.Count - 1);
        SetScrubValue(timeline.DisplayFractionAt(next));
    }

    internal void SetScrubValue(double t, bool expireDownstream = true)
    {
        t = Math.Clamp(t, 0, 1);
        var d = (decimal)t;
        if (Math.Abs(Slider.Value - d) < 0.0000001m) return;
        if (expireDownstream)
            SetSliderValue(d);
        else
            Slider.Value = d;
    }

    private void ToggleSnap()
    {
        SnapToKeyframes = !SnapToKeyframes;
        OnDisplayExpired(false);
    }

    private void ApplyScrubDefaults()
    {
        Slider.Minimum = 0;
        Slider.Maximum = 1;
        Slider.DecimalPlaces = 3;
        Slider.Value = Math.Clamp(Slider.Value, 0, 1);
        Expression = string.Empty;
        Slider.GripDisplay = GH_SliderGripDisplay.Shape;
        Slider.TickDisplay = (GH_SliderTickDisplay)0;
        Slider.RailFullColour = Color.FromArgb(102, Accent);
        Slider.RailEmptyColour = Color.FromArgb(52, 72, 72, 78);
        Slider.GripTopColour = Accent;
    }
}
