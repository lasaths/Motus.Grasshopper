using Grasshopper.Kernel;
using Motus.Core;
using Motus.Geometry;
using Motus.GH.Data;
using Motus.GH.UI;
using Motus.GH.Rhino;
using Rhino.Geometry;

namespace Motus.GH.Components;

public sealed class MotusMotionSegmentComponent : MotusComponentBase
{
    public MotusMotionSegmentComponent()
        : base(
            "Motus Motion Segment",
            "Segment",
            "Build declarative PTP/LIN/CIRC/SET/WAIT planner segments (execution hints only; no robot commands). Optional ToolState on arm segments.",
            "Plan",
            "line-segments") { }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddTextParameter("Type", "T", "PTP, LIN, CIRC, SET, or WAIT", GH_ParamAccess.item, "PTP");
        p.AddGenericParameter("Goal", "G", "PTP: Joint State; LIN/CIRC: Plane (TCP pose)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddPlaneParameter("Via", "V", "CIRC only: arc via point (TCP plane)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Step", "St", "LIN only: TCP step size (m)", GH_ParamAccess.item, 0.005);
        p[p.ParamCount - 1].Optional = true;
        p.AddIntegerParameter("Samples", "N", "CIRC only: arc samples (>= 4)", GH_ParamAccess.item, 16);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Blend", "B", "Blend radius (m, default 0)", GH_ParamAccess.item, 0);
        p[p.ParamCount - 1].Optional = true;
        p.AddGenericParameter("ToolState", "Ts", "Optional tool state goal", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("ToolMode", "Tm", "Hold, Ramp, or Instant interpolation hint (arm segments)", GH_ParamAccess.item, "Hold");
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Duration", "D", "SET/WAIT timing hint (s); SET ramp time when > 0", GH_ParamAccess.item, 0);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddGenericParameter("Segment", "S", "Motion segment", GH_ParamAccess.item);

    public override void AddedToDocument(GH_Document doc)
    {
        base.AddedToDocument(doc);
        if (Params.Input[0].SourceCount > 0) return;
        doc.ScheduleSolution(1, _ =>
        {
            GhValueList.AttachDropdown(this, 0, new[] { "PTP", "LIN", "CIRC", "SET", "WAIT" }, "Type");
            GhValueList.AttachDropdown(this, 7, new[] { "Hold", "Ramp", "Instant" }, "ToolMode");
        });
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var typeText = "PTP";
        if (!da.GetData(0, ref typeText) || !TryParseSegmentType(typeText, out var segmentType))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Type must be PTP, LIN, CIRC, SET, or WAIT.");
            return;
        }

        var blend = 0.0;
        da.GetData(5, ref blend);
        if (blend < 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Blend must be >= 0.");
            return;
        }

        TryReadToolState(da, 6, out var toolState);
        var toolMode = ToolStateMode.Hold;
        if (!TryReadToolMode(da, 7, ref toolMode)) return;

        var duration = 0.0;
        da.GetData(8, ref duration);
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

    private static bool TryReadToolState(IGH_DataAccess da, int index, out EndEffectorState? state)
    {
        state = null;
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
        if (!GhExtract.TryGoal(da, 1, out var joints, out _) || joints is null)
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
        if (!GhExtract.TryGoal(da, 1, out _, out var plane) || plane is null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "LIN requires a Plane goal (TCP pose).");
            return false;
        }

        var step = 0.005;
        da.GetData(3, ref step);
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
        if (!da.GetData(2, ref via) || !via.IsValid)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "CIRC requires Via and Goal planes.");
            return false;
        }

        if (!GhExtract.TryGoal(da, 1, out _, out var goalPlane) || goalPlane is null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "CIRC requires Via and Goal planes.");
            return false;
        }

        var samples = 16;
        da.GetData(4, ref samples);
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
        p.AddGenericParameter("Robot", "Rb", "Robot model", GH_ParamAccess.item);
        p.AddGenericParameter("Segments", "Seg", "List of motion segments", GH_ParamAccess.list);
        p.AddGenericParameter("Start", "S", "Start joint state (defaults to home)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddGenericParameter("Collision", "C", "Collision scene", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddGenericParameter("Group", "Gr", "Optional planning group (locks non-group joints)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddGenericParameter("Attach", "A", "Optional attached bodies list", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddGenericParameter("Trajectory", "T", "Planned trajectory", GH_ParamAccess.item);
        p.AddTextParameter("Status", "St", "Status message", GH_ParamAccess.item);
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

        var start = GhExtract.StartOrHome(da, 2, ctx.EffectiveModel);
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
