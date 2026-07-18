using Grasshopper.Kernel;
using Grasshopper.Kernel.Parameters;
using Grasshopper.Kernel.Types;
using GH_IO.Serialization;
using Motus.Core;
using Motus.Geometry;
using Motus.GH;
using Motus.GH.Data;
using Motus.GH.Params;
using Motus.GH.UI;
using Motus.GH.Preview;
using Motus.GH.Rhino;
using Rhino;
using Rhino.Display;
using Rhino.Geometry;
using System.Drawing;
using System.Windows.Forms;

namespace Motus.GH.Components;

public sealed class MotusPreviewComponent : MotusComponentBase, IGH_VariableParameterComponent
{
    private const int CustomColorsParamIndex = 3;
    private const int CoreOutputCount = 5;

    private static readonly Color PathColor = Color.FromArgb(180, 255, 255, 255);
    private static readonly Color InvalidColor = Color.FromArgb(220, 220, 60, 60);

    private PreviewColorMode _colorMode = PreviewColorMode.Urdf;
    private bool _showCustomColors;
    private bool _showDebugOutputs;
    private List<Color> _customColors = [];
    private IReadOnlyList<Color?>? _drawMeshColors;

    private Trajectory? _trajectory;
    private Trajectory? _previewTrajectory;
    private DateTime _playStartUtc;
    private double _playStartPosition;
    private bool _playing;
    private System.Windows.Forms.Timer? _playTimer;
    private double _position;
    private int _index;
    private bool _showStart;
    private bool _lastBuiltShowStart;
    private bool _suppressScrubInput;
    private Trajectory? _staticsFor;
    private global::Rhino.Geometry.Curve? _tcpCurve;
    private List<global::Rhino.Geometry.Line> _invalidSegments = new();
    private List<Mesh> _currentMeshes = new();
    private List<Mesh> _startMeshes = new();
    private KinematicsPreview.PreviewMeshCache? _meshCache;
    private (int linkCount, string? toolName, int jointCount, int capCount) _cacheSig;
    private List<TrajectoryPoint> _previewPoints = [];
    private readonly Dictionary<(Color Color, float Transparency), DisplayMaterial> _materialCache = new();

    public MotusPreviewComponent()
        : base("Motus Preview", "Preview", "Animated FK preview; wire Motus Scrub or click Play/Stop", "Preview", "eye") { }

    public override void CreateAttributes() =>
        m_attributes = new ButtonAttributes(this, () => _playing ? "\u25A0 Stop" : "\u25B6 Play", () => _playing, TogglePlayback);

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddParameter(new Param_MotusTrajectory(), "Trajectory", "Tr", "Motus trajectory from Motus Plan (list concatenates sequential goals)", GH_ParamAccess.list);
        p.AddBooleanParameter("ShowStart", "SS", "Also preview the trajectory start pose as a ghost", GH_ParamAccess.item, false);
        p.AddNumberParameter("Position", "P", "Optional normalized playback position 0–1 (Motus Scrub); pauses Play when changed", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddMeshParameter("Meshes", "M", "Link meshes at the current frame", GH_ParamAccess.list);
        p.AddLineParameter("Links", "L", "Link lines at the current frame", GH_ParamAccess.list);
        p.AddCurveParameter("TCP Path", "Path", "Full TCP polyline via FK", GH_ParamAccess.item);
        p.AddParameter(new Param_MotusJointState(), "State", "Js", "Joint state at the current frame", GH_ParamAccess.item);
        p.AddNumberParameter("Time", "Tm", "Elapsed trajectory time at current frame (seconds)", GH_ParamAccess.item);
    }

    public override void AddedToDocument(GH_Document doc)
    {
        base.AddedToDocument(doc);
        TrajectoryMerge.EnsureListAccess(this, 0);
        EnsureCustomColorsParam();
        EnsureDebugOutputs();
    }

    public override void AppendAdditionalMenuItems(ToolStripDropDown menu)
    {
        Menu_AppendSeparator(menu);
        Menu_AppendItem(menu, "Override colours", (_, _) => SetColorMode(PreviewColorMode.Override), true, _colorMode == PreviewColorMode.Override);
        Menu_AppendItem(menu, "URDF colours", (_, _) => SetColorMode(PreviewColorMode.Urdf), true, _colorMode == PreviewColorMode.Urdf);
        Menu_AppendItem(menu, "Custom colours", (_, _) => SetColorMode(PreviewColorMode.Custom), true, _colorMode == PreviewColorMode.Custom);
        Menu_AppendSeparator(menu);
        Menu_AppendItem(menu, "Show custom colours input", (_, _) => ToggleCustomColorsInput(), true, _showCustomColors);
        Menu_AppendItem(menu, "Show debug outputs", (_, _) => ToggleDebugOutputs(), true, _showDebugOutputs);
        base.AppendAdditionalMenuItems(menu);
    }

    public bool CanInsertParameter(GH_ParameterSide side, int index)
    {
        if (side == GH_ParameterSide.Input)
            return index == CustomColorsParamIndex && Params.Input.Count == CustomColorsParamIndex;
        if (side == GH_ParameterSide.Output)
            return index >= CoreOutputCount;
        return false;
    }

    public bool CanRemoveParameter(GH_ParameterSide side, int index)
    {
        if (side == GH_ParameterSide.Input)
            return index == CustomColorsParamIndex && Params.Input.Count > CustomColorsParamIndex;
        if (side == GH_ParameterSide.Output)
            return index >= CoreOutputCount;
        return false;
    }

    public IGH_Param CreateParameter(GH_ParameterSide side, int index)
    {
        if (side == GH_ParameterSide.Input)
        {
            return new Param_Colour
            {
                Name = "Custom Colours",
                NickName = "C",
                Description = "One colour per preview mesh slot (same order as Meshes output)",
                Access = GH_ParamAccess.list,
                Optional = true
            };
        }

        return new Param_Integer
        {
            Name = "Index",
            NickName = "I",
            Description = "Current waypoint index (0-based)",
            Access = GH_ParamAccess.item
        };
    }

    public bool DestroyParameter(GH_ParameterSide side, int index) =>
        (side == GH_ParameterSide.Input && index == CustomColorsParamIndex) ||
        (side == GH_ParameterSide.Output && index >= CoreOutputCount);

    public void VariableParameterMaintenance()
    {
        _showCustomColors = Params.Input.Count > CustomColorsParamIndex;
        _showDebugOutputs = Params.Output.Count > CoreOutputCount;
    }

    public override BoundingBox ClippingBox
    {
        get
        {
            var bb = BoundingBox.Empty;
            foreach (var mesh in _currentMeshes)
                bb.Union(mesh.GetBoundingBox(false));
            if (_showStart)
            {
                foreach (var mesh in _startMeshes)
                    bb.Union(mesh.GetBoundingBox(false));
            }
            if (_tcpCurve is not null)
                bb.Union(_tcpCurve.GetBoundingBox(false));
            return bb.IsValid ? bb : BoundingBox.Unset;
        }
    }

    public override void DrawViewportMeshes(IGH_PreviewArgs args)
    {
        if (Locked) return;
        DrawColoredMeshes(args, _currentMeshes, isStartGhost: false);
        if (_showStart)
            DrawColoredMeshes(args, _startMeshes, isStartGhost: true);
    }

    public override void DrawViewportWires(IGH_PreviewArgs args)
    {
        if (Locked) return;
        if (_tcpCurve is not null)
            args.Display.DrawCurve(_tcpCurve, PathColor, 2);
        foreach (var line in _invalidSegments)
            args.Display.DrawLine(line, InvalidColor, 3);
    }

    public override bool Write(GH_IWriter writer)
    {
        writer.SetBoolean("ShowStart", _showStart);
        writer.SetDouble("Position", _position);
        writer.SetInt32("ColorMode", (int)_colorMode);
        writer.SetBoolean("ShowCustomColors", _showCustomColors);
        writer.SetBoolean("ShowDebugOutputs", _showDebugOutputs);
        return base.Write(writer);
    }

    public override bool Read(GH_IReader reader)
    {
        if (reader.ItemExists("ShowStart"))
            _showStart = reader.GetBoolean("ShowStart");
        if (reader.ItemExists("Position"))
            _position = Math.Clamp(reader.GetDouble("Position"), 0, 1);
        if (reader.ItemExists("ColorMode"))
            _colorMode = (PreviewColorMode)reader.GetInt32("ColorMode");
        if (reader.ItemExists("ShowCustomColors"))
            _showCustomColors = reader.GetBoolean("ShowCustomColors");
        if (reader.ItemExists("ShowDebugOutputs"))
            _showDebugOutputs = reader.GetBoolean("ShowDebugOutputs");
        // Migrate older documents that always had debug outputs.
        if (Params.Output.Count > CoreOutputCount)
            _showDebugOutputs = true;
        return base.Read(reader);
    }

    public override void RemovedFromDocument(GH_Document doc)
    {
        StopPlayTimer();
        _playing = false;
        base.RemovedFromDocument(doc);
    }

    private void SetColorMode(PreviewColorMode mode)
    {
        _colorMode = mode;
        ExpireSolution(true);
    }

    private void ToggleCustomColorsInput()
    {
        _showCustomColors = !_showCustomColors;
        if (_showCustomColors)
            EnsureCustomColorsParam();
        else if (Params.Input.Count > CustomColorsParamIndex)
            Params.UnregisterInputParameter(Params.Input[CustomColorsParamIndex]);
        Params.OnParametersChanged();
        ExpireSolution(true);
    }

    private void ToggleDebugOutputs()
    {
        _showDebugOutputs = !_showDebugOutputs;
        EnsureDebugOutputs();
        ExpireSolution(true);
    }

    private void EnsureCustomColorsParam()
    {
        if (!_showCustomColors || Params.Input.Count > CustomColorsParamIndex) return;
        Params.RegisterInputParam(CreateParameter(GH_ParameterSide.Input, CustomColorsParamIndex), CustomColorsParamIndex);
    }

    private void EnsureDebugOutputs()
    {
        void Ensure(string name, Func<IGH_Param> factory)
        {
            var idx = OutputIndexOf(name);
            if (_showDebugOutputs && idx < 0)
                Params.RegisterOutputParam(factory());
            else if (!_showDebugOutputs && idx >= 0)
                Params.UnregisterOutputParameter(Params.Output[idx]);
        }

        Ensure("Index", () => new Param_Integer
        {
            Name = "Index",
            NickName = "I",
            Description = "Current waypoint index (0-based)",
            Access = GH_ParamAccess.item
        });
        Ensure("Invalid", () => new Param_Line
        {
            Name = "Invalid",
            NickName = "X",
            Description = "Invalid TCP segments (joint/velocity/acceleration limits)",
            Access = GH_ParamAccess.list
        });
        Ensure("ToolState", () => new Param_GenericObject
        {
            Name = "ToolState",
            NickName = "Ts",
            Description = "Tool state at the current frame",
            Access = GH_ParamAccess.item,
            Optional = true
        });
        Ensure("Width", () => new Param_Number
        {
            Name = "Width",
            NickName = "W",
            Description = "Gripper width (m) at playhead when present",
            Access = GH_ParamAccess.item,
            Optional = true
        });
        Params.OnParametersChanged();
        VariableParameterMaintenance();
    }

    private int OutputIndexOf(string name)
    {
        for (var i = 0; i < Params.Output.Count; i++)
        {
            if (string.Equals(Params.Output[i].Name, name, StringComparison.Ordinal))
                return i;
        }
        return -1;
    }

    private void DrawColoredMeshes(IGH_PreviewArgs args, IReadOnlyList<Mesh> meshes, bool isStartGhost)
    {
        var urdfColors = _colorMode == PreviewColorMode.Urdf ? _drawMeshColors : null;
        var transparency = isStartGhost ? PreviewColorResolver.StartTransparency : PreviewColorResolver.CurrentTransparency;
        for (var i = 0; i < meshes.Count; i++)
        {
            var color = PreviewColorResolver.Resolve(i, _colorMode, urdfColors, _customColors, isStartGhost);
            args.Display.DrawMeshShaded(meshes[i], MaterialFor(color, transparency));
        }
    }

    private DisplayMaterial MaterialFor(Color color, float transparency)
    {
        var key = (color, transparency);
        if (!_materialCache.TryGetValue(key, out var mat))
        {
            mat = new DisplayMaterial(color) { Transparency = transparency };
            _materialCache[key] = mat;
        }
        return mat;
    }

    private void TogglePlayback()
    {
        if (_playing)
        {
            StopPlayTimer();
            _playing = false;
        }
        else if (_trajectory?.Points.Count > 0)
        {
            _position = 0;
            _playing = true;
            _playStartPosition = 0;
            _playStartUtc = DateTime.UtcNow;
            // Silent scrub move — never expire from inside the Play mouse-up handler.
            SyncScrubSlider(0, expireDownstream: false);
            StartPlayTimer();
        }

        // Defer solve out of the canvas MouseUp stack (re-entrant NewSolution crashes Rhino).
        var doc = OnPingDocument();
        if (doc is not null)
            doc.ScheduleSolution(1, _ => ExpireSolution(false));
        else
            ExpireSolution(false);
    }

    private void StartPlayTimer()
    {
        _playTimer ??= new System.Windows.Forms.Timer { Interval = 33 };
        _playTimer.Tick -= OnPlayTimerTick;
        _playTimer.Tick += OnPlayTimerTick;
        if (!_playTimer.Enabled)
            _playTimer.Start();
    }

    private void StopPlayTimer()
    {
        if (_playTimer is null) return;
        _playTimer.Stop();
        _playTimer.Tick -= OnPlayTimerTick;
    }

    /// <summary>
    /// Play frames on a UI timer — no ScheduleSolution. Document solves freeze the canvas
    /// while wall-clock advances, so the old path jumped straight to the final pose.
    /// </summary>
    private void OnPlayTimerTick(object? sender, EventArgs e)
    {
        if (!_playing || _trajectory is null || _previewPoints.Count == 0)
        {
            StopPlayTimer();
            return;
        }

        ResolveFrame(out var state, out var elapsed, out _, out var toolState);
        var duration = _trajectory.DurationSeconds;
        if (elapsed >= duration)
        {
            StopPlayTimer();
            _playing = false;
            _position = duration > 0 ? 1 : 0;
            SyncScrubSlider(_position, expireDownstream: false);
            OnDisplayExpired(false);
            ExpireSolution(false);
            return;
        }

        if (_meshCache is null)
        {
            SyncScrubSlider(_position, expireDownstream: false);
            var doc = OnPingDocument();
            if (doc is not null)
                doc.ScheduleSolution(1, _ => ExpireSolution(false));
            else
                ExpireSolution(false);
            return;
        }

        _meshCache.UpdateMeshes(state, _currentMeshes, toolState);
        SyncScrubSlider(_position, expireDownstream: false);
        ExpirePreview(true);
        OnDisplayExpired(false);
        RhinoDoc.ActiveDoc?.Views.Redraw();
    }

    private void EnsureMeshCache(RobotContext ctx, RobotCollisionModel? previewGeometry, ToolCapabilities? toolCapabilities)
    {
        if (previewGeometry is null)
        {
            _meshCache = null;
            _drawMeshColors = null;
            return;
        }

        var sig = (previewGeometry.Links.Count, previewGeometry.ToolGeometry?.Name, ctx.Chain?.Joints.Length ?? 0, toolCapabilities?.Parameters.Count ?? 0);
        if (_meshCache is not null && sig == _cacheSig)
        {
            _drawMeshColors = _meshCache.MeshColors ??
                              PreviewColorResolver.AlignMeshColors(previewGeometry, ctx.PreviewMeshColors);
            return;
        }

        _cacheSig = sig;
        _meshCache = KinematicsPreview.PreviewMeshCache.TryCreate(
            ctx.EffectiveModel,
            previewGeometry,
            ctx.Chain,
            ctx.Base,
            ctx.Tool,
            toolCapabilities,
            ctx.PreviewMeshColors);
        _drawMeshColors = _meshCache?.MeshColors ??
                          PreviewColorResolver.AlignMeshColors(previewGeometry, ctx.PreviewMeshColors);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!TrajectoryMerge.TryResolve(da, 0, this, GH_RuntimeMessageLevel.Remark, out var trajGoo))
            return;
        var t = trajGoo.Value!;
        var ctx = trajGoo.Context();
        var previewGeometry = RobotPreviewGeometry.ForViewport(
            ctx.PreviewGeometry ?? ctx.EffectiveModel.CollisionModel,
            trajGoo.ToolSnapshot);
        EnsureMeshCache(ctx, previewGeometry, trajGoo.ToolCapabilitiesSnapshot);
        ReadCustomColors(da);

        if (_colorMode == PreviewColorMode.Custom && _customColors.Count == 0)
        {
            AddRuntimeMessage(
                GH_RuntimeMessageLevel.Remark,
                "Custom colours mode: wire a colour list (Meshes order) or switch to Override.");
        }

        var trajectoryChanged = !SameTrajectoryContent(_trajectory, t);
        if (trajectoryChanged)
        {
            StopPlayTimer();
            _playing = false;
            _position = 0;
            _staticsFor = null;
            _suppressScrubInput = true;
            if (TryGetWiredScrub(out var wiredScrub))
                wiredScrub.InvalidateTimelineCache();
        }
        _trajectory = t;
        if (t.Points.Count == 0)
        {
            _currentMeshes = [];
            _startMeshes = [];
            _tcpCurve = null;
            _invalidSegments = [];
            _previewTrajectory = null;
            ExpirePreview(true);
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Trajectory has no points.");
            return;
        }
        da.GetData(1, ref _showStart);
        if (trajectoryChanged)
            SyncScrubSlider(0);
        else
            ApplyScrubInput(da);

        if (_suppressScrubInput)
        {
            _position = 0;
            _suppressScrubInput = false;
        }
        if (t.Robot.Preset.AxisCount != ctx.Model.Preset.AxisCount)
        {
            AddRuntimeMessage(
                GH_RuntimeMessageLevel.Warning,
                $"Trajectory axis count ({t.Robot.Preset.AxisCount}) differs from preview robot axis count ({ctx.Model.Preset.AxisCount}).");
        }
        var previewPoints = BuildPreviewPoints(t, ctx.Model, out _);
        _previewPoints = previewPoints;
        _previewTrajectory = new Trajectory(ctx.Model, previewPoints);
        ResolveFrame(out var state, out var timeSeconds, out _index, out var toolState);
        // Concatenate() allocates a new Trajectory each solve — compare content, not reference.
        var staticsDirty = trajectoryChanged || _staticsFor is null;
        if (staticsDirty)
        {
            var pl = KinematicsPreview.TcpPath(ctx.EffectiveModel, previewPoints.Select(p => p.JointState), ctx.Chain, ctx.Base, ctx.Tool);
            _tcpCurve = pl.Count >= 2 ? pl.ToNurbsCurve() : null;
            KinematicsPreview.TrajectorySegments(
                ctx.EffectiveModel,
                PreviewTrajectory(),
                new TrajectoryValidationOptions(),
                out _,
                out var invalid,
                ctx.Chain,
                ctx.Base,
                ctx.Tool);
            _invalidSegments = invalid;
            _staticsFor = t;
        }

        if (staticsDirty || _showStart != _lastBuiltShowStart)
        {
            if (_showStart && _meshCache is not null && previewPoints.Count > 0)
                _startMeshes = _meshCache.MeshesFor(previewPoints[0].JointState, previewPoints[0].ToolState);
            else
                _startMeshes = [];
            _lastBuiltShowStart = _showStart;
        }
        if (_meshCache is not null)
        {
            // Always update in place — MeshesFor duplicates every link mesh on idle solves.
            if (_currentMeshes.Count == 0)
                _currentMeshes = _meshCache.MeshesFor(state, toolState);
            else
                _meshCache.UpdateMeshes(state, _currentMeshes, toolState);
        }
        else
        {
            _currentMeshes = KinematicsPreview.LinkMeshes(ctx.EffectiveModel, state, previewGeometry, ctx.Chain, ctx.Base, ctx.Tool).ToList();
            _drawMeshColors = PreviewColorResolver.AlignMeshColors(previewGeometry!, ctx.PreviewMeshColors);
        }

        if (_playing)
        {
            StartPlayTimer();
            SyncScrubSlider(_position);
            ExpirePreview(true);
            return;
        }
        if (IsScrubDragging())
        {
            ExpirePreview(true);
            return;
        }
        da.SetDataList(0, _currentMeshes);
        da.SetDataList(1, KinematicsPreview.LinkLines(ctx.EffectiveModel, state, ctx.Chain, ctx.Base, ctx.Tool).ToList());
        da.SetData(2, _tcpCurve);
        da.SetData(3, new JointStateGoo(state));
        da.SetData(4, timeSeconds);
        if (_showDebugOutputs)
        {
            var indexOut = OutputIndexOf("Index");
            var invalidOut = OutputIndexOf("Invalid");
            var toolOut = OutputIndexOf("ToolState");
            var widthOut = OutputIndexOf("Width");
            if (indexOut >= 0) da.SetData(indexOut, _index);
            if (invalidOut >= 0) da.SetDataList(invalidOut, _invalidSegments);
            if (toolOut >= 0 && toolState is not null)
                da.SetData(toolOut, new EndEffectorStateGoo(toolState));
            if (widthOut >= 0)
                da.SetData(widthOut, toolState?.GetValueOrDefault("width"));
        }
        ExpirePreview(true);
    }

    private void ReadCustomColors(IGH_DataAccess da)
    {
        _customColors = [];
        if (Params.Input.Count <= CustomColorsParamIndex) return;
        var colors = new List<GH_Colour>();
        if (!da.GetDataList(CustomColorsParamIndex, colors)) return;
        _customColors = colors.Select(c => c.Value).ToList();
    }

    private void ResolveFrame(out JointState state, out double timeSeconds, out int index, out EndEffectorState? toolState)
    {
        var duration = _trajectory!.DurationSeconds;
        double elapsed;
        if (_playing)
        {
            elapsed = _playStartPosition * duration + (DateTime.UtcNow - _playStartUtc).TotalSeconds;
            elapsed = Math.Clamp(elapsed, 0, duration);
            _position = duration > 0 ? elapsed / duration : 0;
        }
        else
            elapsed = _position * duration;

        state = TrajectoryInterpolation.AtTime(PreviewTrajectory(), elapsed, out index);
        toolState = TrajectoryInterpolation.AtTimeToolState(_trajectory, elapsed);
        timeSeconds = elapsed;
        _index = index;
    }

    private void ApplyScrubInput(IGH_DataAccess da)
    {
        if (_suppressScrubInput) return;
        if (!TryReadWiredPosition(da, out var scrub, out var positionFromWire)) return;
        if (scrub?.IsSyncingFromPreview == true) return;
        positionFromWire = MapScrubToTimeFraction(scrub, positionFromWire);
        if (scrub?.IsDragging == true)
        {
            _playing = false;
            _position = Math.Clamp(positionFromWire, 0, 1);
            return;
        }
        if (!_playing)
            _position = Math.Clamp(positionFromWire, 0, 1);
    }

    /// <summary>
    /// Scrub slider is display-space (even keyframe ticks). Playback position is time-space.
    /// Snap only affects magnetic pull on the slider — mapping always interpolates.
    /// </summary>
    private static double MapScrubToTimeFraction(MotusScrubSlider? scrub, double scrubFraction)
    {
        if (scrub is null) return scrubFraction;
        var timeline = scrub.ResolveTimeline();
        if (timeline.IsEmpty) return scrubFraction;
        return timeline.DisplayToTimeFraction(scrubFraction);
    }

    private static bool SameTrajectoryContent(Trajectory? prior, Trajectory next)
    {
        if (prior is null) return false;
        if (ReferenceEquals(prior, next)) return true;
        if (prior.Points.Count != next.Points.Count) return false;
        if (Math.Abs(prior.DurationSeconds - next.DurationSeconds) > 1e-6) return false;
        if (prior.Points.Count == 0) return true;
        // Cheap fingerprint: endpoints + midpoint time (Concatenate reallocates each solve).
        var a0 = prior.Points[0];
        var b0 = next.Points[0];
        var a1 = prior.Points[^1];
        var b1 = next.Points[^1];
        if (Math.Abs(a0.TimeSeconds - b0.TimeSeconds) > 1e-9 || Math.Abs(a1.TimeSeconds - b1.TimeSeconds) > 1e-9)
            return false;
        return JointsNearlyEqual(a0.JointState, b0.JointState) && JointsNearlyEqual(a1.JointState, b1.JointState);
    }

    private static bool JointsNearlyEqual(JointState a, JointState b)
    {
        if (a.AxisCount != b.AxisCount) return false;
        for (var i = 0; i < a.AxisCount; i++)
        {
            if (Math.Abs(a.Positions[i] - b.Positions[i]) > 1e-6)
                return false;
        }
        return true;
    }

    private bool TryGetWiredScrub(out MotusScrubSlider scrub)
    {
        if (Params.Input[2].Sources.FirstOrDefault() is MotusScrubSlider s)
        {
            scrub = s;
            return true;
        }
        scrub = null!;
        return false;
    }

    private bool TryReadWiredPosition(IGH_DataAccess da, out MotusScrubSlider? scrub, out double position)
    {
        scrub = null;
        position = 0;
        if (Params.Input[2].SourceCount == 0) return false;
        scrub = Params.Input[2].Sources.FirstOrDefault() as MotusScrubSlider;
        return da.GetData(2, ref position);
    }

    private bool IsScrubDragging() =>
        TryGetWiredScrub(out var scrub) && scrub.IsDragging;

    /// <summary>Trajectory currently driving scrub keyframes (may be null before first solve).</summary>
    internal Trajectory? ScrubTrajectory => _trajectory;

    /// <summary>
    /// Preview-only scrub update during slider drag — skips full graph ExpireSolution.
    /// </summary>
    internal static bool TryNotifyScrubDrag(MotusScrubSlider scrub, double scrubFraction)
    {
        var doc = scrub.OnPingDocument();
        if (doc is null) return false;
        foreach (var obj in doc.Objects)
        {
            if (obj is not MotusPreviewComponent preview) continue;
            if (!preview.Params.Input[2].Sources.Any(s => ReferenceEquals(s, scrub))) continue;
            return preview.ApplyScrubDragPreview(scrub, scrubFraction);
        }
        return false;
    }

    private bool ApplyScrubDragPreview(MotusScrubSlider scrub, double scrubFraction)
    {
        if (_trajectory is null || _previewPoints.Count == 0) return false;
        _playing = false;
        _position = Math.Clamp(MapScrubToTimeFraction(scrub, scrubFraction), 0, 1);
        ResolveFrame(out var state, out _, out _, out var toolState);
        if (_meshCache is not null)
        {
            if (_currentMeshes.Count == 0)
                _currentMeshes = _meshCache.MeshesFor(state, toolState);
            else
                _meshCache.UpdateMeshes(state, _currentMeshes, toolState);
        }
        ExpirePreview(true);
        return true;
    }

    private void SyncScrubSlider(double position, bool expireDownstream = false)
    {
        if (Params.Input[2].Sources.FirstOrDefault() is not MotusScrubSlider scrub) return;
        var timeline = scrub.ResolveTimeline();
        var display = timeline.IsEmpty ? position : timeline.TimeToDisplayFraction(position);
        scrub.BeginSyncFromPreview();
        try
        {
            scrub.SetScrubValue(display, expireDownstream);
            scrub.OnDisplayExpired(false);
        }
        finally
        {
            scrub.EndSyncFromPreview();
        }
    }

    private Trajectory PreviewTrajectory() =>
        _previewTrajectory ?? new Trajectory(_trajectory!.Robot, _previewPoints);

    private static List<TrajectoryPoint> BuildPreviewPoints(
        Trajectory sourceTrajectory,
        RobotModel previewRobot,
        out bool remapApplied)
    {
        remapApplied = false;
        if (sourceTrajectory.Points.Count == 0) return [];
        var mappedStates = TryBuildJointRemap(sourceTrajectory.Robot, previewRobot, out var map)
            ? sourceTrajectory.Points.Select(p => new JointState(RemapPositions(p.JointState.Positions, map))).ToList()
            : sourceTrajectory.Points.Select(p => p.JointState).ToList();
        remapApplied = map.Length > 0;
        var points = new List<TrajectoryPoint>(sourceTrajectory.Points.Count);
        for (var i = 0; i < sourceTrajectory.Points.Count; i++)
        {
            var src = sourceTrajectory.Points[i];
            points.Add(new TrajectoryPoint(
                src.TimeSeconds,
                mappedStates[i],
                src.MotionType,
                src.SegmentIndex,
                src.BlendRadiusMeters,
                src.ToolState));
        }
        return points;
    }

    private static bool TryBuildJointRemap(RobotModel sourceRobot, RobotModel targetRobot, out int[] map)
    {
        map = [];
        if (sourceRobot.Preset.AxisCount != targetRobot.Preset.AxisCount) return false;
        var sourceNames = sourceRobot.JointNames;
        var targetNames = targetRobot.JointNames;
        if (sourceNames is null || targetNames is null) return false;
        if (sourceNames.Count != sourceRobot.Preset.AxisCount || targetNames.Count != targetRobot.Preset.AxisCount) return false;
        var sourceIndex = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
        for (var i = 0; i < sourceNames.Count; i++)
            sourceIndex[sourceNames[i]] = i;
        map = new int[targetNames.Count];
        for (var i = 0; i < targetNames.Count; i++)
        {
            if (!sourceIndex.TryGetValue(targetNames[i], out var idx))
            {
                map = [];
                return false;
            }
            map[i] = idx;
        }
        return true;
    }

    private static double[] RemapPositions(IReadOnlyList<double> sourcePositions, IReadOnlyList<int> map)
    {
        var remapped = new double[map.Count];
        for (var i = 0; i < map.Count; i++)
            remapped[i] = sourcePositions[map[i]];
        return remapped;
    }

    public override Guid ComponentGuid => new Guid("d4a8f1c2-3e5b-4a7d-9c1e-8f2b6d4e0a91");
}
