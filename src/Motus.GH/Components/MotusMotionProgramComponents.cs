using Grasshopper.Kernel;
using Grasshopper.Kernel.Parameters;
using Motus.Core;
using Motus.Geometry;
using Motus.GH.Data;
using Motus.GH.Params;
using Motus.GH.UI;
using Motus.GH.Rhino;
using Rhino.Geometry;
using System.Windows.Forms;

namespace Motus.GH.Components;

public sealed class MotusMotionSegmentComponent : MotusComponentBase, IGH_VariableParameterComponent
{
    private const int CoreInputCount = 4;
    private string _lastSyncedType = "PTP";

    public MotusMotionSegmentComponent()
        : base(
            "Motus Motion Segment",
            "Segment",
            "Build declarative PTP/LIN/CIRC/SET/WAIT planner segments (execution hints only; no robot commands). Optional ToolState on arm segments.",
            "Plan",
            "line-segments") { }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddTextParameter("Type", "Ty", "PTP, LIN, CIRC, SET, or WAIT", GH_ParamAccess.item, "PTP");
        p.AddGenericParameter("Goal", "G", "PTP: Joint State; LIN/CIRC: Plane (TCP pose)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Blend", "B", "Blend radius (m, default 0)", GH_ParamAccess.item, 0);
        p[p.ParamCount - 1].Optional = true;
        p.AddGenericParameter("ToolState", "Ts", "Optional tool state goal", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddParameter(new Param_MotusSegment(), "Segment", "Seg", "Motion segment", GH_ParamAccess.item);

    public override void AddedToDocument(GH_Document doc)
    {
        base.AddedToDocument(doc);
        doc.ScheduleSolution(1, _ =>
        {
            if (Params.Input[0].SourceCount == 0)
                GhValueList.AttachDropdown(this, 0, new[] { "PTP", "LIN", "CIRC", "SET", "WAIT" }, "Type");
            SyncPinsForType(PeekType(), force: true);
        });
    }

    protected override void BeforeSolveInstance()
    {
        SyncPinsForType(PeekType());
        base.BeforeSolveInstance();
    }

    public bool CanInsertParameter(GH_ParameterSide side, int index) =>
        side == GH_ParameterSide.Input && index >= CoreInputCount;

    public bool CanRemoveParameter(GH_ParameterSide side, int index) =>
        side == GH_ParameterSide.Input && index >= CoreInputCount;

    public IGH_Param CreateParameter(GH_ParameterSide side, int index) =>
        new Param_Number { Name = "Step", NickName = "St", Optional = true };

    public bool DestroyParameter(GH_ParameterSide side, int index) =>
        side == GH_ParameterSide.Input && index >= CoreInputCount;

    public void VariableParameterMaintenance() { }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var typeText = "PTP";
        if (!da.GetData(0, ref typeText) || !TryParseSegmentType(typeText, out var segmentType))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Type must be PTP, LIN, CIRC, SET, or WAIT.");
            return;
        }

        var blend = 0.0;
        da.GetData(IndexOf("Blend"), ref blend);
        if (blend < 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Blend must be >= 0.");
            return;
        }

        TryReadToolState(da, IndexOf("ToolState"), out var toolState);
        var toolMode = ToolStateMode.Hold;
        var toolModeIdx = IndexOf("ToolMode");
        if (toolModeIdx >= 0 && !TryReadToolMode(da, toolModeIdx, ref toolMode)) return;

        var duration = 0.0;
        var durationIdx = IndexOf("Duration");
        if (durationIdx >= 0)
            da.GetData(durationIdx, ref duration);
        if (duration < 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Duration must be >= 0.");
            return;
        }

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

    private string PeekType()
    {
        var param = Params.Input[0];
        foreach (var branch in param.VolatileData.Paths)
        {
            var list = param.VolatileData.get_Branch(branch);
            if (list is null || list.Count == 0) continue;
            if (list[0] is Grasshopper.Kernel.Types.GH_String gs && !string.IsNullOrWhiteSpace(gs.Value))
                return gs.Value.Trim().ToUpperInvariant();
            if (list[0] is Grasshopper.Kernel.Types.IGH_Goo goo && goo.CastTo<string>(out var s) && !string.IsNullOrWhiteSpace(s))
                return s.Trim().ToUpperInvariant();
        }

        if (param is Param_String ps && ps.PersistentDataCount > 0)
        {
            var v = ps.PersistentData.get_FirstItem(false);
            if (v is not null && !string.IsNullOrWhiteSpace(v.Value))
                return v.Value.Trim().ToUpperInvariant();
        }

        return "PTP";
    }

    private void SyncPinsForType(string typeUpper, bool force = false)
    {
        if (!force && typeUpper == _lastSyncedType) return;
        _lastSyncedType = typeUpper;

        var wantStep = typeUpper == "LIN";
        var wantVia = typeUpper == "CIRC";
        var wantSamples = typeUpper == "CIRC";
        var wantToolMode = typeUpper is "PTP" or "LIN" or "CIRC";
        var wantDuration = typeUpper is "SET" or "WAIT";

        SetPinPresent("Step", wantStep, () => new Param_Number
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

        SetPinPresent("Via", wantVia, () => new Param_Plane
        {
            Name = "Via",
            NickName = "V",
            Description = "CIRC only: arc via point (TCP plane)",
            Access = GH_ParamAccess.item,
            Optional = true
        });

        SetPinPresent("Samples", wantSamples, () => new Param_Integer
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

        SetPinPresent("ToolMode", wantToolMode, () => new Param_String
        {
            Name = "ToolMode",
            NickName = "Tm",
            Description = "Hold, Ramp, or Instant interpolation hint (arm segments)",
            Access = GH_ParamAccess.item,
            Optional = true
        }, setDefault: p =>
        {
            if (p is Param_String ps)
                ps.SetPersistentData("Hold");
        });

        SetPinPresent("Duration", wantDuration, () => new Param_Number
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

        Params.OnParametersChanged();

        var toolModeIdx = IndexOf("ToolMode");
        if (toolModeIdx >= 0 && Params.Input[toolModeIdx].SourceCount == 0 && OnPingDocument() is GH_Document doc)
        {
            doc.ScheduleSolution(1, _ =>
                GhValueList.AttachDropdown(this, toolModeIdx, new[] { "Hold", "Ramp", "Instant" }, "ToolMode"));
        }
    }

    private void SetPinPresent(string name, bool show, Func<IGH_Param> factory, Action<IGH_Param>? setDefault = null)
    {
        var existing = IndexOf(name);
        if (show && existing < 0)
        {
            var param = factory();
            setDefault?.Invoke(param);
            Params.RegisterInputParam(param);
        }
        else if (!show && existing >= 0)
            Params.UnregisterInputParameter(Params.Input[existing]);
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

    private bool TryReadToolMode(IGH_DataAccess da, int index, ref ToolStateMode mode)
    {
        var text = "Hold";
        if (!da.GetData(index, ref text)) return true;
        switch (text.Trim().ToUpperInvariant())
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
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "ToolMode must be Hold, Ramp, or Instant.");
                return false;
        }
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

    private PlanningResult? _cached;
    private TrajectoryGoo? _cachedGoo;
    private bool _run;

    public MotusProgramPlanComponent()
        : base(
            "Motus Program Plan",
            "ProgPlan",
            "Plan a mixed PTP/LIN/CIRC motion program (click Plan). Unlike Motus Plan plane goals, LIN failures do not fall back to joint-space paths.",
            "Plan",
            "stack") { }

    public override void CreateAttributes() =>
        m_attributes = new ButtonAttributes(this, () => "Plan", () => false, RequestRun);

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddParameter(new Param_MotusRobot(), "Robot", "Rb", "Robot model", GH_ParamAccess.item);
        p.AddParameter(new Param_MotusSegment(), "Segments", "Seg", "List of motion segments", GH_ParamAccess.list);
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

    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!GhExtract.TryRobotGoo(da, 0, out var robotGoo)) return;
        var ctx = RobotContext.FromGoo(robotGoo);

        if (!_run)
        {
            if (_cachedGoo is not null) da.SetData(0, _cachedGoo);
            da.SetData(1, _cached is null ? "Press Plan to compute." : GhExtract.BuildProgramStatusMessage(_cached, true));
            if (_cached is not null) da.SetDataList(2, GhExtract.BuildProgramWarnings(_cached));
            return;
        }

        _run = false;

        if (!GhExtract.TryMotionSegments(da, 1, out var segments, out var segmentErrors))
        {
            foreach (var error in segmentErrors)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, error);
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
