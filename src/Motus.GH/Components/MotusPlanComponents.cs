using Grasshopper.Kernel;
using GH_IO.Serialization;
using Motus.Core;
using Motus.GH.Data;
using Motus.GH.Planning;
using Motus.GH.UI;
using Motus.GH.Rhino;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Motus.GH.Components;

public sealed class MotusPlanComponent : MotusAsyncComponentBase
{
    internal const double DefaultLinStepMeters = 0.005;
    private const int AutoPlanDebounceMs = 400;

    private readonly PlanWorker _worker;

    private List<PlanningResult>? _cached;
    private List<TrajectoryGoo>? _cachedGoos;
    private bool _run;
    private bool _autoPlan;
    private string? _lastPlannedFingerprint;
    private int _debounceGen;
    private bool _planningPending;
    private string? _activeWorkerFingerprint;

    public MotusPlanComponent()
        : base("Motus Plan", "Plan", "Plan motion to a plane (TCP LIN) or joint goal; click Plan or enable Auto Plan", "Plan", "flow-arrow")
    {
        _worker = new PlanWorker(this);
        BaseWorker = _worker;
        TaskCreationOptions = System.Threading.Tasks.TaskCreationOptions.LongRunning;
    }

    internal bool AutoPlanEnabled => _autoPlan;

    protected override bool ShouldAbortRunningWorkers() => false;

    protected override void BeforeSolveInstance()
    {
        // Keep in-flight workers alive across idle re-solves; cancel explicitly below.
        if (IsReadyToSetData)
            base.BeforeSolveInstance();
    }

    public override void CreateAttributes() =>
        m_attributes = new ButtonAttributes(this, PlanButtonLabel, () => _autoPlan || IsOperationInProgress, RequestRun);

    private string PlanButtonLabel() =>
        IsOperationInProgress ? "Planning…" : _autoPlan ? "Replan" : "Plan";

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Robot", "Rb", "Robot model", GH_ParamAccess.item);
        p.AddGenericParameter("Goal", "G", "Targets as Planes (TCP LIN) or Joint States", GH_ParamAccess.list);
        p.AddGenericParameter("Start", "S", "Start joint state (defaults to UR10e home or zeros)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Step", "St", "Plane goals only: TCP LIN step size (m)", GH_ParamAccess.item, DefaultLinStepMeters);
        p[p.ParamCount - 1].Optional = true;
        p.AddGenericParameter("Collision", "C", "Obstacle-aware planning: ColScene → Collision. Joint goals use RRT (tune via RrtSettings); plane goals use LIN validate", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddGenericParameter("Group", "Gr", "Optional planning group (locks non-group joints)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddGenericParameter("Attach", "A", "Optional attached bodies list", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
        p.AddGenericParameter("RrtSettings", "Rrt", "Optional RRT tuning from Motus RRT Settings (joint goals + collision only)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddGenericParameter("Trajectory", "T", "Planned trajectories", GH_ParamAccess.list);
        p.AddTextParameter("Status", "St", "Status message", GH_ParamAccess.item);
        p.AddTextParameter("Warnings", "W", "Warnings", GH_ParamAccess.list);
    }

    public override void AppendAdditionalMenuItems(ToolStripDropDown menu)
    {
        Menu_AppendItem(menu, "Auto Plan", AutoPlanMenuClick, true, _autoPlan);
        if (IsOperationInProgress)
            Menu_AppendItem(menu, "Cancel planning", (_, _) => RequestCancellation());
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

    internal bool HasCollisionInputsWired() =>
        Params.Input[MotusPlanInputs.Collision].SourceCount > 0 ||
        Params.Input[MotusPlanInputs.Attach].SourceCount > 0;

    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (IsReadyToSetData)
        {
            base.SolveInstance(da);
            _activeWorkerFingerprint = null;
            return;
        }

        if (!GhExtract.TryRobotGoo(da, 0, out var robotGoo))
            return;

        var ctx = RobotContext.FromGoo(robotGoo);
        if (!GhExtract.TryGoals(da, 1, out var goals, out var goalErrors))
        {
            foreach (var error in goalErrors)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, error);
            if (goals.Count == 0)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Provide at least one valid Plane or Joint State goal.");
            EmitOutputs(da, GhExtract.PlanStatusKind.Manual);
            return;
        }

        var start = GhExtract.StartOrHome(da, 2, ctx.Model);
        var linStep = DefaultLinStepMeters;
        var stepInput = DefaultLinStepMeters;
        if (da.GetData(3, ref stepInput))
        {
            if (stepInput <= 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Step must be positive for plane goals.");
                EmitOutputs(da, GhExtract.PlanStatusKind.Manual);
                return;
            }

            linStep = stepInput;
        }

        var collision = GhExtract.ParseCollisionInput(da, MotusPlanInputs.Collision);
        if (collision.Error is not null)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, collision.Error);
        else if (collision.Warning is not null)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, collision.Warning);

        var rrtSettings = GhExtract.ResolveRrtSettings(da, MotusPlanInputs.RrtSettings, this);

        var planningContext = GhExtract.BuildPlanningContext(
            ctx.EffectiveModel,
            da,
            MotusPlanInputs.Collision,
            MotusPlanInputs.Group,
            MotusPlanInputs.Attach,
            collision.Scene);
        var fingerprint = PlanInputFingerprint.Compute(
            ctx.Model,
            robotGoo.BaseFrameOverride,
            robotGoo.Tool,
            goals,
            start,
            planningContext,
            linStep,
            rrtSettings);

        if (IsOperationInProgress && _activeWorkerFingerprint is not null && _activeWorkerFingerprint != fingerprint)
            RequestCancellation();

        var collisionInputWired = HasCollisionInputsWired();
        var activity = GhExtract.DescribePlanningActivity(goals, planningContext, collisionInputWired, rrtSettings);

        var planNow = _run;
        if (planNow)
            _run = false;

        if (_autoPlan && !Locked && !planNow && !IsOperationInProgress && fingerprint != _lastPlannedFingerprint)
        {
            _debounceGen++;
            ScheduleDebouncedPlan(fingerprint);
            _planningPending = true;
        }

        if (!ShouldStartPlanning(fingerprint, planNow))
        {
            var idleKind = ResolveIdleStatusKind(fingerprint);
            var idleActivity = idleKind == GhExtract.PlanStatusKind.Planning ? activity : null;
            EmitOutputs(da, idleKind, idleActivity);
            return;
        }

        _planningPending = false;
        _activeWorkerFingerprint = fingerprint;
        Message = "Planning…";
        OnDisplayExpired(true);
        // Collect inputs before writing outputs — GH data access may not allow reads after SetData.
        LaunchWorker(da);
        EmitOutputs(da, GhExtract.PlanStatusKind.Planning, activity);
    }

    internal void ApplyWorkerResult(
        string fingerprint,
        IReadOnlyList<PlanningResult> results,
        IReadOnlyList<TrajectoryGoo> goos,
        bool isAutoPlan,
        IReadOnlyList<string> remarks)
    {
        _cached = results.ToList();
        _cachedGoos = goos.ToList();
        _lastPlannedFingerprint = fingerprint;

        ReportPlanningFailures(_cached);
        foreach (var remark in remarks)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, remark);
    }

    private bool ShouldStartPlanning(string fingerprint, bool planNow)
    {
        if (Locked)
            return false;
        if (planNow)
            return true;
        if (_autoPlan && fingerprint != _lastPlannedFingerprint && !IsOperationInProgress)
            return _run;
        return false;
    }

    private void AutoPlanMenuClick(object? sender, EventArgs e)
    {
        RecordUndoEvent("Auto Plan");
        _autoPlan = !_autoPlan;
        _lastPlannedFingerprint = null;
        _debounceGen++;
        _planningPending = false;
        if (_autoPlan && HasCollisionInputsWired())
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                "Auto Plan enabled with collision — planning is debounced and runs in the background.");
        }

        RequestSolutionRefresh();
    }

    private void RequestRun()
    {
        if (IsOperationInProgress)
            RequestCancellation();
        _run = true;
        _planningPending = false;
        RequestSolutionRefresh();
    }

    private GhExtract.PlanStatusKind ResolveIdleStatusKind(string fingerprint)
    {
        if (IsOperationInProgress || _planningPending || (_autoPlan && fingerprint != _lastPlannedFingerprint))
            return GhExtract.PlanStatusKind.Planning;
        if (_autoPlan)
            return GhExtract.PlanStatusKind.AutoCached;
        return _cached is null ? GhExtract.PlanStatusKind.Manual : GhExtract.PlanStatusKind.ManualCached;
    }

    private void EmitOutputs(IGH_DataAccess da, GhExtract.PlanStatusKind statusKind, string? activity = null)
    {
        if (statusKind != GhExtract.PlanStatusKind.Planning && _cached is not null)
            ReportPlanningFailures(_cached);

        if (_cachedGoos is { Count: > 0 })
            da.SetDataList(0, _cachedGoos);

        if (statusKind == GhExtract.PlanStatusKind.Planning && _cachedGoos is { Count: > 0 })
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                "Trajectory is from previous inputs; replanning in background…");
        }

        da.SetData(1, GhExtract.BuildStatusMessage(_cached, statusKind, activity));
        if (_cached is not null)
            da.SetDataList(2, GhExtract.BuildWarnings(_cached));
    }

    private void ScheduleDebouncedPlan(string fingerprint)
    {
        var gen = ++_debounceGen;
        if (OnPingDocument() is not Grasshopper.Kernel.GH_Document doc) return;
        doc.ScheduleSolution(AutoPlanDebounceMs, _ =>
        {
            if (gen != _debounceGen || Locked || !_autoPlan) return;
            _run = true;
            ExpireSolution(false);
        });
    }

    private void RequestSolutionRefresh()
    {
        if (OnPingDocument() is GH_Document doc)
            doc.ScheduleSolution(1, _ => ExpireSolution(false));
        else
            ExpireSolution(true);
    }

    private void ReportPlanningFailures(IReadOnlyList<PlanningResult> results)
    {
        foreach (var pair in results.Select((result, index) => (result, index)))
        {
            if (pair.result.Success) continue;
            var detail = pair.result.Errors.Count > 0
                ? string.Join("; ", pair.result.Errors)
                : "Planning failed.";
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $"Goal[{pair.index}]: {detail}");
            if (pair.result.Errors.Any(e =>
                    e.Contains("Start configuration", StringComparison.OrdinalIgnoreCase) &&
                    e.Contains("collision", StringComparison.OrdinalIgnoreCase)))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                    "Preview draws URDF visual meshes; Plan checks URDF collision meshes (often larger) plus any tool collision hull. " +
                    "ShowStart ghost is the trajectory start — confirm Plan Start matches that pose.");
            }
        }
    }

    public override Guid ComponentGuid => new Guid("8bb0bae3-527f-4e80-a8a4-c8a88b7276de");
}
