using System.Collections.Generic;
using System.Drawing;
using System.Windows.Forms;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Parameters;
using GH_IO.Serialization;
using Motus.Core;
using Motus.Geometry;
using Motus.GH.Data;
using Motus.GH.Params;
using Motus.GH.UI;
using Motus.GH.Rhino;
using Rhino.Geometry;

namespace Motus.GH.Components;

/// <summary>
/// Motus Move — one PTP/LIN/CIRC/SET/WAIT line with Arup-style on-component Type (± ToolMode) dropdowns.
/// </summary>
public sealed class MotusMotionSegmentComponent : MotusComponentBase, IGH_VariableParameterComponent
{
    private static readonly string[] MotionTypes = ["PTP", "LIN", "CIRC", "SET", "WAIT"];
    private static readonly string[] ToolModes = ["Hold", "Ramp", "Instant"];

    private string _motionType = "PTP";
    private string _toolMode = "Hold";
    private string _lastSyncedType = "";

    public MotusMotionSegmentComponent()
        : base(
            "Motus Move",
            "Move",
            "Build one PTP/LIN/CIRC/SET/WAIT program line (Type dropdown on component). Wire several into Motus Program.",
            "Plan",
            "line-segments") { }

    protected override IReadOnlyList<string> AiKeywords { get; } =
    [
        "Next: Seg->Motus Program Segments",
        "Note: Type dropdown morphs Goal pins before wiring Goal",
        "Wire: PTP Goal=Joints Js; LIN/CIRC Goal=plane/TCP P",
    ];

    private PointF? _canvasPivot;

    public override void CreateAttributes()
    {
        // Pin morph / dropdown rebuild must not dump the component to (0,0).
        var pivot = _canvasPivot;
        if (pivot is null && Attributes is not null)
        {
            var p = Attributes.Pivot;
            if (p.X != 0 || p.Y != 0)
                pivot = p;
        }

        m_attributes = new DropDownAttributes(this, BuildDropdownModel, OnDropdownSelect);
        if (pivot is { } keep)
            m_attributes.Pivot = keep;
    }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        // Type kept as a pin for document serialization / external wires; UI is the face dropdown.
        p.AddTextParameter("Type", "Ty", "PTP, LIN, CIRC, SET, or WAIT (prefer the on-component dropdown)", GH_ParamAccess.item, "PTP");
        p.AddGenericParameter("Goal", "G", "PTP: Joint State; LIN/CIRC: Plane (TCP pose)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Blend", "B", "Blend radius (m, default 0)", GH_ParamAccess.item, 0);
        p[p.ParamCount - 1].Optional = true;
        p.AddParameter(new Param_MotusToolState(), "ToolState", "Ts", "Tool state (SET required; optional on arm moves)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddParameter(new Param_MotusSegment(), "Segment", "Seg", "Motion segment", GH_ParamAccess.item);

    public override void AddedToDocument(GH_Document doc)
    {
        base.AddedToDocument(doc);
        doc.ScheduleSolution(1, _ =>
        {
            _motionType = PeekType();
            SyncTypePin(_motionType);
            // Keep canvas Bounds/Pivot from Read — recreating attributes here zeros pivots.
            SyncPinsForType(_motionType, force: true, recreateAttributes: false);
            RestoreCanvasPivot();
        });
    }

    public override bool Write(GH_IWriter writer)
    {
        writer.SetString("MotionType", _motionType);
        writer.SetString("ToolMode", _toolMode);
        if (Attributes is not null)
        {
            writer.SetDouble("CanvasPivotX", Attributes.Pivot.X);
            writer.SetDouble("CanvasPivotY", Attributes.Pivot.Y);
        }

        return base.Write(writer);
    }

    public override bool Read(GH_IReader reader)
    {
        if (reader.ItemExists("MotionType"))
            _motionType = NormalizeType(reader.GetString("MotionType"));
        if (reader.ItemExists("ToolMode"))
            _toolMode = NormalizeToolMode(reader.GetString("ToolMode"));
        if (reader.ItemExists("CanvasPivotX") && reader.ItemExists("CanvasPivotY"))
        {
            _canvasPivot = new PointF(
                (float)reader.GetDouble("CanvasPivotX"),
                (float)reader.GetDouble("CanvasPivotY"));
        }

        // Do not SyncPins before base.Read — OnParametersChanged there wipes Attributes.Pivot to (0,0).
        // ParameterData + IGH_VariableParameterComponent hydrate type-specific pins using CreateParameter.
        _lastSyncedType = _motionType;
        var ok = base.Read(reader);
        if (Attributes is not null)
        {
            var p = Attributes.Pivot;
            if (p.X != 0 || p.Y != 0)
                _canvasPivot = p;
        }

        RestoreCanvasPivot();
        return ok;
    }

    private void RestoreCanvasPivot()
    {
        if (_canvasPivot is not { } p || Attributes is null) return;
        Attributes.Pivot = p;
    }

    protected override void BeforeSolveInstance()
    {
        // Wired Type pin still wins (legacy ValueList / panel).
        if (Params.Input[0].SourceCount > 0)
            _motionType = PeekType();
        else
            SyncTypePin(_motionType);

        SyncPinsForType(_motionType);
        base.BeforeSolveInstance();
    }

    public bool CanInsertParameter(GH_ParameterSide side, int index) =>
        side == GH_ParameterSide.Input;

    public bool CanRemoveParameter(GH_ParameterSide side, int index) =>
        side == GH_ParameterSide.Input && index > 0;

    public IGH_Param CreateParameter(GH_ParameterSide side, int index)
    {
        foreach (var (name, factory) in ExtraPinsForType(_motionType))
        {
            if (IndexOf(name) < 0)
                return factory();
        }

        return new Param_Number { Name = "Step", NickName = "St", Optional = true };
    }

    public bool DestroyParameter(GH_ParameterSide side, int index) =>
        side == GH_ParameterSide.Input && index > 0;

    public void VariableParameterMaintenance() { }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var typeText = _motionType;
        if (Params.Input[0].SourceCount > 0)
        {
            typeText = "PTP";
            if (!da.GetData(0, ref typeText) || !TryParseSegmentType(typeText, out _))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Type must be PTP, LIN, CIRC, SET, or WAIT.");
                return;
            }
            _motionType = NormalizeType(typeText);
        }

        if (!TryParseSegmentType(_motionType, out var segmentType))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Type must be PTP, LIN, CIRC, SET, or WAIT.");
            return;
        }

        var blend = 0.0;
        var blendIdx = IndexOf("Blend");
        if (blendIdx >= 0)
            da.GetData(blendIdx, ref blend);
        if (blend < 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Blend must be >= 0.");
            return;
        }

        TryReadToolState(da, IndexOf("ToolState"), out var toolState);
        if (!TryParseToolMode(_toolMode, out var toolMode))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "ToolMode must be Hold, Ramp, or Instant.");
            return;
        }

        var duration = 0.0;
        var durationIdx = IndexOf("Duration");
        if (durationIdx >= 0)
            da.GetData(durationIdx, ref duration);
        if (duration < 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Duration must be >= 0.");
            return;
        }

        Message = _motionType;

        switch (segmentType)
        {
            case MotionPrimitiveType.Ptp:
                if (!TryBuildPtp(da, blend, toolState, toolMode, out var ptp)) return;
                da.SetData(0, new MotionSegmentGoo(ptp));
                break;
            case MotionPrimitiveType.Lin:
                if (!TryBuildLin(da, blend, toolState, toolMode, out var lin)) return;
                da.SetData(0, new MotionSegmentGoo(lin));
                break;
            case MotionPrimitiveType.Circ:
                if (!TryBuildCirc(da, blend, toolState, toolMode, out var circ)) return;
                da.SetData(0, new MotionSegmentGoo(circ));
                break;
            case MotionPrimitiveType.Set:
                if (toolState is null)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "SET requires a ToolState goal.");
                    return;
                }
                da.SetData(0, new MotionSegmentGoo(new SetToolStateSegment(toolState, duration)));
                break;
            case MotionPrimitiveType.Wait:
                if (duration <= 0)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "WAIT requires Duration > 0.");
                    return;
                }
                da.SetData(0, new MotionSegmentGoo(new WaitSegment(duration)));
                break;
        }
    }

    private DropDownAttributes.Model BuildDropdownModel()
    {
        var type = NormalizeType(_motionType);
        var isArm = type is "PTP" or "LIN" or "CIRC";
        if (isArm)
        {
            return new DropDownAttributes.Model(
                ["Type", "ToolMode"],
                [MotionTypes, ToolModes],
                [type, NormalizeToolMode(_toolMode)]);
        }

        return new DropDownAttributes.Model(
            ["Type"],
            [MotionTypes],
            [type]);
    }

    private void OnDropdownSelect(int listIndex, int itemIndex)
    {
        if (listIndex == 0)
        {
            if (itemIndex < 0 || itemIndex >= MotionTypes.Length) return;
            var next = MotionTypes[itemIndex];
            if (next == _motionType) return;
            RecordUndoEvent("Move Type");
            _motionType = next;
            SyncTypePin(_motionType);
            SyncPinsForType(_motionType, force: true);
            ExpireSolution(true);
            return;
        }

        if (listIndex == 1)
        {
            if (itemIndex < 0 || itemIndex >= ToolModes.Length) return;
            var next = ToolModes[itemIndex];
            if (next == _toolMode) return;
            RecordUndoEvent("Move ToolMode");
            _toolMode = next;
            ExpireSolution(true);
        }
    }

    private string PeekType()
    {
        var param = Params.Input[0];
        foreach (var branch in param.VolatileData.Paths)
        {
            var list = param.VolatileData.get_Branch(branch);
            if (list is null || list.Count == 0) continue;
            if (list[0] is Grasshopper.Kernel.Types.GH_String gs && !string.IsNullOrWhiteSpace(gs.Value))
                return NormalizeType(gs.Value);
            if (list[0] is Grasshopper.Kernel.Types.IGH_Goo goo && goo.CastTo<string>(out var s) && !string.IsNullOrWhiteSpace(s))
                return NormalizeType(s);
        }

        if (param is Param_String ps && ps.PersistentDataCount > 0)
        {
            var v = ps.PersistentData.get_FirstItem(false);
            if (v is not null && !string.IsNullOrWhiteSpace(v.Value))
                return NormalizeType(v.Value);
        }

        return NormalizeType(_motionType);
    }

    private void SyncTypePin(string typeUpper)
    {
        if (Params.Input[0] is not Param_String ps) return;
        if (ps.SourceCount > 0) return;
        var current = ps.PersistentDataCount > 0 ? ps.PersistentData.get_FirstItem(false)?.Value : null;
        if (string.Equals(current, typeUpper, StringComparison.OrdinalIgnoreCase)) return;
        ps.PersistentData.Clear();
        ps.SetPersistentData(typeUpper);
    }

    private IEnumerable<(string Name, Func<IGH_Param> Factory)> ExtraPinsForType(string typeUpper)
    {
        typeUpper = NormalizeType(typeUpper);
        if (typeUpper == "LIN")
        {
            yield return ("Step", () => new Param_Number
            {
                Name = "Step",
                NickName = "St",
                Description = "LIN only: TCP step size (m)",
                Access = GH_ParamAccess.item,
                Optional = true
            });
        }

        if (typeUpper == "CIRC")
        {
            yield return ("Via", () => new Param_Plane
            {
                Name = "Via",
                NickName = "V",
                Description = "CIRC only: arc via point (TCP plane)",
                Access = GH_ParamAccess.item,
                Optional = true
            });
            yield return ("Samples", () => new Param_Integer
            {
                Name = "Samples",
                NickName = "N",
                Description = "CIRC only: arc samples (>= 4)",
                Access = GH_ParamAccess.item,
                Optional = true
            });
        }

        if (typeUpper is "SET" or "WAIT")
        {
            yield return ("Duration", () => new Param_Number
            {
                Name = "Duration",
                NickName = "D",
                Description = "SET/WAIT timing hint (s); SET ramp time when > 0",
                Access = GH_ParamAccess.item,
                Optional = true
            });
        }
    }

    private void SyncPinsForType(string typeUpper, bool force = false, bool recreateAttributes = true)
    {
        typeUpper = NormalizeType(typeUpper);
        if (!force && typeUpper == _lastSyncedType) return;

        var prevArm = _lastSyncedType is "PTP" or "LIN" or "CIRC";
        var isArm = typeUpper is "PTP" or "LIN" or "CIRC";
        _lastSyncedType = typeUpper;

        var wantGoal = isArm;
        var wantBlend = isArm;
        var wantToolState = isArm || typeUpper == "SET";
        var wantStep = typeUpper == "LIN";
        var wantVia = typeUpper == "CIRC";
        var wantSamples = typeUpper == "CIRC";
        var wantDuration = typeUpper is "SET" or "WAIT";

        var changed = false;
        changed |= SetPinPresent("Goal", wantGoal, () => new Param_GenericObject
        {
            Name = "Goal",
            NickName = "G",
            Description = "PTP: Joint State; LIN/CIRC: Plane (TCP pose)",
            Access = GH_ParamAccess.item,
            Optional = true
        });

        changed |= SetPinPresent("Blend", wantBlend, () => new Param_Number
        {
            Name = "Blend",
            NickName = "B",
            Description = "Blend radius (m, default 0)",
            Access = GH_ParamAccess.item,
            Optional = true
        }, setDefault: p =>
        {
            if (p is Param_Number pn)
                pn.SetPersistentData(0.0);
        });

        changed |= SetPinPresent("ToolState", wantToolState, () => new Param_MotusToolState
        {
            Name = "ToolState",
            NickName = "Ts",
            Description = typeUpper == "SET"
                ? "SET: tool state goal (required)"
                : "Optional tool state on arm move",
            Access = GH_ParamAccess.item,
            Optional = true
        });

        // Remove legacy ToolMode pin if an older document still has it.
        changed |= SetPinPresent("ToolMode", false, () => new Param_String());

        changed |= SetPinPresent("Step", wantStep, () => new Param_Number
        {
            Name = "Step",
            NickName = "St",
            Description = "LIN only: TCP step size (m)",
            Access = GH_ParamAccess.item,
            Optional = true
        }, setDefault: p =>
        {
            if (p is Param_Number pn)
                pn.SetPersistentData(0.005);
        });

        changed |= SetPinPresent("Via", wantVia, () => new Param_Plane
        {
            Name = "Via",
            NickName = "V",
            Description = "CIRC only: arc via point (TCP plane)",
            Access = GH_ParamAccess.item,
            Optional = true
        });

        changed |= SetPinPresent("Samples", wantSamples, () => new Param_Integer
        {
            Name = "Samples",
            NickName = "N",
            Description = "CIRC only: arc samples (>= 4)",
            Access = GH_ParamAccess.item,
            Optional = true
        }, setDefault: p =>
        {
            if (p is Param_Integer pi)
                pi.SetPersistentData(16);
        });

        changed |= SetPinPresent("Duration", wantDuration, () => new Param_Number
        {
            Name = "Duration",
            NickName = "D",
            Description = "SET/WAIT timing hint (s); SET ramp time when > 0",
            Access = GH_ParamAccess.item,
            Optional = true
        }, setDefault: p =>
        {
            if (p is Param_Number pn)
                pn.SetPersistentData(0.0);
        });

        if (!changed) return;

        var pivot = Attributes?.Pivot;
        if (pivot is { } p && (p.X != 0 || p.Y != 0))
            _canvasPivot = p;

        Params.OnParametersChanged();
        if (recreateAttributes && (force || prevArm != isArm))
            CreateAttributes();
        else
            RestoreCanvasPivot();
    }

    private bool SetPinPresent(string name, bool show, Func<IGH_Param> factory, Action<IGH_Param>? setDefault = null)
    {
        var existing = IndexOf(name);
        if (show && existing < 0)
        {
            var param = factory();
            setDefault?.Invoke(param);
            Params.RegisterInputParam(param);
            return true;
        }

        if (!show && existing >= 0)
        {
            Params.UnregisterInputParameter(Params.Input[existing]);
            return true;
        }

        return false;
    }

    private int IndexOf(string name)
    {
        for (var i = 0; i < Params.Input.Count; i++)
        {
            if (string.Equals(Params.Input[i].Name, name, StringComparison.Ordinal))
                return i;
        }
        return -1;
    }

    private static bool TryReadToolState(IGH_DataAccess da, int index, out EndEffectorState? state)
    {
        state = null;
        if (index < 0) return true;
        EndEffectorStateGoo? goo = null;
        if (!da.GetData(index, ref goo) || goo?.Value is null) return true;
        state = goo.Value;
        return true;
    }

    private bool TryBuildPtp(
        IGH_DataAccess da,
        double blend,
        EndEffectorState? toolState,
        ToolStateMode toolMode,
        out PtpSegment segment)
    {
        segment = null!;
        if (!GhExtract.TryGoal(da, IndexOf("Goal"), out var joints, out _) || joints is null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "PTP requires a Joint State goal.");
            return false;
        }

        segment = new PtpSegment(joints, blend)
        {
            TargetState = toolState,
            ToolStateMode = toolMode
        };
        return true;
    }

    private bool TryBuildLin(
        IGH_DataAccess da,
        double blend,
        EndEffectorState? toolState,
        ToolStateMode toolMode,
        out LinSegment segment)
    {
        segment = null!;
        if (!GhExtract.TryGoal(da, IndexOf("Goal"), out _, out var plane) || plane is null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "LIN requires a Plane goal (TCP pose).");
            return false;
        }

        var step = 0.005;
        var stepIdx = IndexOf("Step");
        if (stepIdx >= 0)
            da.GetData(stepIdx, ref step);
        if (step <= 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Step must be > 0.");
            return false;
        }

        segment = new LinSegment(new CartesianPose(FrameConversion.FromPlane(plane.Value)), step, blend)
        {
            TargetState = toolState,
            ToolStateMode = toolMode
        };
        return true;
    }

    private bool TryBuildCirc(
        IGH_DataAccess da,
        double blend,
        EndEffectorState? toolState,
        ToolStateMode toolMode,
        out CircSegment segment)
    {
        segment = null!;
        var via = Plane.Unset;
        var viaIdx = IndexOf("Via");
        if (viaIdx < 0 || !da.GetData(viaIdx, ref via) || !via.IsValid)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "CIRC requires Via and Goal planes.");
            return false;
        }

        if (!GhExtract.TryGoal(da, IndexOf("Goal"), out _, out var goalPlane) || goalPlane is null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "CIRC requires Via and Goal planes.");
            return false;
        }

        var samples = 16;
        var samplesIdx = IndexOf("Samples");
        if (samplesIdx >= 0)
            da.GetData(samplesIdx, ref samples);
        if (samples < 4)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Samples must be >= 4.");
            return false;
        }

        segment = new CircSegment(
            new CartesianPose(FrameConversion.FromPlane(via)),
            new CartesianPose(FrameConversion.FromPlane(goalPlane.Value)),
            samples,
            blend)
        {
            TargetState = toolState,
            ToolStateMode = toolMode
        };
        return true;
    }

    private static string NormalizeType(string? raw)
    {
        var t = (raw ?? "PTP").Trim().ToUpperInvariant();
        return t is "PTP" or "LIN" or "CIRC" or "SET" or "WAIT" ? t : "PTP";
    }

    private static string NormalizeToolMode(string? raw)
    {
        var t = (raw ?? "Hold").Trim();
        return t.ToUpperInvariant() switch
        {
            "RAMP" => "Ramp",
            "INSTANT" => "Instant",
            _ => "Hold"
        };
    }

    private static bool TryParseToolMode(string raw, out ToolStateMode mode)
    {
        switch (raw.Trim().ToUpperInvariant())
        {
            case "HOLD":
                mode = ToolStateMode.Hold;
                return true;
            case "RAMP":
                mode = ToolStateMode.Ramp;
                return true;
            case "INSTANT":
                mode = ToolStateMode.Instant;
                return true;
            default:
                mode = ToolStateMode.Hold;
                return false;
        }
    }

    private static bool TryParseSegmentType(string raw, out MotionPrimitiveType type)
    {
        switch (raw.Trim().ToUpperInvariant())
        {
            case "PTP":
                type = MotionPrimitiveType.Ptp;
                return true;
            case "LIN":
                type = MotionPrimitiveType.Lin;
                return true;
            case "CIRC":
                type = MotionPrimitiveType.Circ;
                return true;
            case "SET":
                type = MotionPrimitiveType.Set;
                return true;
            case "WAIT":
                type = MotionPrimitiveType.Wait;
                return true;
            default:
                type = default;
                return false;
        }
    }

    public override Guid ComponentGuid => new("7c4e9a2f-1b3d-4e8a-9f6c-2d8b5a7e9c31");
}

public sealed class MotusProgramPlanComponent : MotusComponentBase
{
    private const double MaxJointStep = 0.05;
    private const int AutoPlanDebounceMs = 400;

    private PlanningResult? _cached;
    private TrajectoryGoo? _cachedGoo;
    private bool _run;
    private bool _autoPlan;
    private bool _autoPlanAttempted;
    private int _debounceGen;

    public MotusProgramPlanComponent()
        : base(
            "Motus Program",
            "Program",
            "Plan a mixed Motus Move sequence (PTP/LIN/CIRC/SET/WAIT). Click Plan or enable Auto Plan. Unlike Motus Plan plane goals, LIN failures do not fall back to joint-space paths.",
            "Plan",
            "stack") { }

    protected override IReadOnlyList<string> AiKeywords { get; } =
    [
        "Wire: Motus Robot Rb; Motus Move Seg list in program order",
        "Next: Tr->Motus Preview / Motus Waypoints",
        "Note: click Plan or enable Auto Plan",
    ];

    public override void CreateAttributes() =>
        m_attributes = new ButtonAttributes(
            this,
            () => _autoPlan ? "Replan" : "Plan",
            () => _autoPlan,
            RequestRun);

    public override void AppendAdditionalMenuItems(ToolStripDropDown menu)
    {
        Menu_AppendItem(menu, "Auto Plan", AutoPlanMenuClick, true, _autoPlan);
        base.AppendAdditionalMenuItems(menu);
    }

    public override bool Write(GH_IWriter writer)
    {
        writer.SetBoolean("AutoPlan", _autoPlan);
        return base.Write(writer);
    }

    public override bool Read(GH_IReader reader)
    {
        if (reader.ItemExists("AutoPlan"))
            _autoPlan = reader.GetBoolean("AutoPlan");
        return base.Read(reader);
    }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddParameter(new Param_MotusRobot(), "Robot", "Rb", "Robot model", GH_ParamAccess.item);
        p.AddParameter(new Param_MotusSegment(), "Segments", "Seg", "List of Motus Move segments (wire order = program order)", GH_ParamAccess.list);
        p.AddParameter(new Param_MotusJointState(), "Start", "St0", "Start joint state (defaults to home)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddParameter(new Param_MotusCollisionScene(), "Collision", "C", "Collision scene", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddGenericParameter("Group", "Gr", "Optional planning group (locks non-group joints)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddGenericParameter("Attach", "A", "Optional attached bodies list", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddParameter(new Param_MotusTrajectory(), "Trajectory", "Tr", "Planned trajectory", GH_ParamAccess.item);
        p.AddTextParameter("Status", "Msg", "Status message", GH_ParamAccess.item);
        p.AddTextParameter("Warnings", "W", "Warnings", GH_ParamAccess.list);
    }

    private void RequestRun()
    {
        _run = true;
        ExpireSolution(true);
    }

    private void AutoPlanMenuClick(object? sender, EventArgs e)
    {
        RecordUndoEvent("Auto Plan");
        _autoPlan = !_autoPlan;
        _debounceGen++;
        if (_autoPlan)
        {
            _cached = null;
            _cachedGoo = null;
            _autoPlanAttempted = false;
        }

        ExpireSolution(true);
    }

    private void ScheduleDebouncedPlan()
    {
        var gen = ++_debounceGen;
        if (OnPingDocument() is not GH_Document doc) return;
        doc.ScheduleSolution(AutoPlanDebounceMs, _ =>
        {
            if (gen != _debounceGen || Locked || !_autoPlan) return;
            _run = true;
            ExpireSolution(false);
        });
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!GhExtract.TryRobotGoo(da, 0, out var robotGoo)) return;
        var ctx = RobotContext.FromGoo(robotGoo);

        if (!_run)
        {
            // First open / empty cache: schedule once (segments are new objects each solve).
            if (_autoPlan && !Locked && !_autoPlanAttempted && _cached is null &&
                GhExtract.TryMotionSegments(da, 1, out var pending, out _) &&
                pending.Count > 0)
            {
                _autoPlanAttempted = true;
                ScheduleDebouncedPlan();
            }

            if (_cachedGoo is not null) da.SetData(0, _cachedGoo);
            da.SetData(1, _cached is null
                ? (_autoPlan ? "Auto Plan pending…" : "Press Plan to compute.")
                : GhExtract.BuildProgramStatusMessage(_cached, true));
            if (_cached is not null) da.SetDataList(2, GhExtract.BuildProgramWarnings(_cached));
            return;
        }

        _run = false;

        if (!GhExtract.TryMotionSegments(da, 1, out var segments, out var segmentErrors))
        {
            foreach (var error in segmentErrors)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, error);
            da.SetData(1, "Fix segment input errors.");
            da.SetDataList(2, segmentErrors);
            return;
        }

        var toolCaps = robotGoo.Tool?.Capabilities;
        var toolStateErrors = MotionProgramValidation.ValidateToolStates(segments, toolCaps).ToList();
        if (toolStateErrors.Count > 0)
        {
            _cached = null;
            _cachedGoo = null;
            foreach (var err in toolStateErrors)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, err);
            da.SetData(1, "Fix tool state errors.");
            da.SetDataList(2, toolStateErrors);
            return;
        }

        var start = GhExtract.StartOrHome(da, 2, ctx.EffectiveModel, out var usedDefaultStart);
        GhExtract.RemarkIfDefaultStart(this, usedDefaultStart);

        var collision = GhExtract.ParseCollisionInput(da, 3);
        if (collision.Error is not null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, collision.Error);
            _cached = null;
            _cachedGoo = null;
            da.SetData(1, "Fix Collision input errors.");
            da.SetDataList(2, new[] { collision.Error });
            return;
        }
        else if (collision.Warning is not null)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, collision.Warning);

        var planningContext = GhExtract.BuildPlanningContext(ctx.EffectiveModel, da, 3, 4, 5, collision.Scene);
        var checker = GhExtract.TryCollisionChecker(ctx.EffectiveModel, ctx.Chain, planningContext.Scene, planningContext.Attached);
        var opts = planningContext.ToPlanningOptions(new PlanningOptions
        {
            MaxJointStepRadians = MaxJointStep,
            CollisionChecker = checker
        });

        var request = new MotionProgramRequest(ctx.EffectiveModel, start, segments, opts)
        {
            InitialToolState = toolCaps?.DefaultState(),
            ToolCapabilities = toolCaps,
            SessionTool = robotGoo.Tool
        };
        _cached = new IndustrialMotionPlanner(ctx.EffectiveModel.Preset, ctx.Chain).Plan(request);

        if (_cached.Success && _cached.Trajectory is not null)
        {
            _cachedGoo = new TrajectoryGoo(_cached.Trajectory)
            {
                Chain = robotGoo.Chain,
                Tree = robotGoo.Tree,
                PreviewGeometry = robotGoo.EffectivePreviewGeometry(),
                PreviewMeshColors = robotGoo.PreviewMeshColors,
                BaseFrameOverride = robotGoo.BaseFrameOverride,
                ToolSnapshot = robotGoo.Tool,
                ToolCapabilitiesSnapshot = toolCaps,
                DiagnosticsSnapshot = _cached.Messages,
                ProvenanceSnapshot = new PlannerProvenance
                {
                    PlannerId = "industrial-motion-program"
                }
            };
            da.SetData(0, _cachedGoo);
        }

        da.SetData(1, GhExtract.BuildProgramStatusMessage(_cached, false));
        da.SetDataList(2, GhExtract.BuildProgramWarnings(_cached));
    }

    public override Guid ComponentGuid => new("8d5f0b3e-2c4e-4f9b-0a7d-3e9c6b8f0d42");
}
