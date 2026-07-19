using Grasshopper.Kernel;
using Grasshopper.Kernel.Parameters;
using GH_IO.Serialization;
using Motus.Core;
using Motus.GH.Data;
using Motus.GH.Params;
using Motus.GH.Planning;
using Motus.GH.UI;
using Motus.GH.Rhino;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;

namespace Motus.GH.Components;

public sealed class MotusPlanComponent : MotusAsyncComponentBase, IGH_VariableParameterComponent
{
    internal const double DefaultLinStepMeters = 0.005;
    private const int AutoPlanDebounceMs = 400;
    private const int CoreInputCount = 4;

    private readonly PlanWorker _worker;

    private List<PlanningResult>? _cached;
    private List<TrajectoryGoo>? _cachedGoos;
    private List<string> _pendingRemarks = [];
    private bool _run;
    private bool _autoPlan;
    private string? _lastPlannedFingerprint;
    private int _debounceGen;
    private bool _planningPending;
    private string? _activeWorkerFingerprint;

    private bool _showCollision;
    private bool _showGroup;
    private bool _showAttach;
    private bool _showRrtSettings;

    public MotusPlanComponent()
        : base("Motus Plan", "Quick", "Quick planner: plane goals = TCP LIN, joint goals = joint-linear or RRT with collision. Click Plan or enable Auto Plan. For PTP/CIRC/SET/WAIT use Motus Move → Motus Program.", "Plan", "flow-arrow")
    {
        _worker = new PlanWorker(this);
        BaseWorker = _worker;
        TaskCreationOptions = System.Threading.Tasks.TaskCreationOptions.LongRunning;
    }

    protected override IReadOnlyList<string> AiKeywords { get; } =
    [
        "Wire: Motus UR10e/Robot Rb; Goal G from Joints Js and/or planes/TCP P",
        "Next: Tr->Motus Preview Tr; Motus Waypoints Tr",
        "Note: click Plan or enable Auto Plan",
        "Note: show Collision pin for obstacle-aware RRT; unwired obstacles are display-only",
    ];

    internal bool AutoPlanEnabled => _autoPlan;

    protected override bool ShouldAbortRunningWorkers() => false;

    protected override void BeforeSolveInstance()
    {
        if (IsReadyToSetData)
            base.BeforeSolveInstance();
    }

    public override void CreateAttributes() =>
        m_attributes = new ButtonAttributes(this, PlanButtonLabel, () => _autoPlan || IsOperationInProgress, RequestRun);

    private string PlanButtonLabel() =>
        IsOperationInProgress ? "Planning…" : _autoPlan ? "Replan" : "Plan";

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddParameter(new Param_MotusRobot(), "Robot", "Rb", "Robot model from Motus UR10e or Motus Robot", GH_ParamAccess.item);
        p.AddGenericParameter("Goal", "G", "Planes (TCP LIN) or Joint States; list = visit order", GH_ParamAccess.list);
        p.AddGenericParameter("Start", "St0", "Start as Plane (IK) or Joint State (defaults to home/zeros)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Step", "St", "Plane goals only: TCP LIN step size (m)", GH_ParamAccess.item, DefaultLinStepMeters);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddParameter(new Param_MotusTrajectory(), "Trajectory", "Tr", "Planned trajectories → Motus Preview / Motus Waypoints (one per goal)", GH_ParamAccess.list);
        p.AddTextParameter("Status", "Msg", "Status message (read before controller handoff)", GH_ParamAccess.item);
        p.AddTextParameter("Warnings", "W", "Warnings", GH_ParamAccess.list);
    }

    public override void AddedToDocument(GH_Document doc)
    {
        base.AddedToDocument(doc);
        EnsureAdvancedParams();
    }

    public override void AppendAdditionalMenuItems(ToolStripDropDown menu)
    {
        Menu_AppendItem(menu, "Auto Plan", AutoPlanMenuClick, true, _autoPlan);
        if (IsOperationInProgress)
            Menu_AppendItem(menu, "Cancel planning", (_, _) => RequestCancellation());
        Menu_AppendSeparator(menu);
        Menu_AppendItem(menu, "Show Collision input", (_, _) => ToggleAdvanced(MotusPlanInputs.Collision, ref _showCollision), true, _showCollision);
        Menu_AppendItem(menu, "Show Group input", (_, _) => ToggleAdvanced(MotusPlanInputs.Group, ref _showGroup), true, _showGroup);
        Menu_AppendItem(menu, "Show Attach input", (_, _) => ToggleAdvanced(MotusPlanInputs.Attach, ref _showAttach), true, _showAttach);
        Menu_AppendItem(menu, "Show RRT Settings input", (_, _) => ToggleAdvanced(MotusPlanInputs.RrtSettings, ref _showRrtSettings), true, _showRrtSettings);
        base.AppendAdditionalMenuItems(menu);
    }

    public override bool Write(GH_IWriter writer)
    {
        writer.SetBoolean("AutoPlan", _autoPlan);
        writer.SetBoolean("ShowCollision", _showCollision);
        writer.SetBoolean("ShowGroup", _showGroup);
        writer.SetBoolean("ShowAttach", _showAttach);
        writer.SetBoolean("ShowRrtSettings", _showRrtSettings);
        return base.Write(writer);
    }

    public override bool Read(GH_IReader reader)
    {
        if (reader.ItemExists("AutoPlan"))
            _autoPlan = reader.GetBoolean("AutoPlan");
        if (reader.ItemExists("ShowCollision"))
            _showCollision = reader.GetBoolean("ShowCollision");
        if (reader.ItemExists("ShowGroup"))
            _showGroup = reader.GetBoolean("ShowGroup");
        if (reader.ItemExists("ShowAttach"))
            _showAttach = reader.GetBoolean("ShowAttach");
        if (reader.ItemExists("ShowRrtSettings"))
            _showRrtSettings = reader.GetBoolean("ShowRrtSettings");

        // Keep Show* flags for CreateParameter during ParameterData hydrate.
        // Sync from actual pins only after base.Read (sparse advanced sets omit Group, etc.).
        var ok = base.Read(reader);
        SyncFlagsFromExistingParams();
        return ok;
    }

    public bool CanInsertParameter(GH_ParameterSide side, int index) =>
        side == GH_ParameterSide.Input && index >= CoreInputCount;

    public bool CanRemoveParameter(GH_ParameterSide side, int index) =>
        side == GH_ParameterSide.Input && index >= CoreInputCount;

    public IGH_Param CreateParameter(GH_ParameterSide side, int index)
    {
        // Honor Show* flags so archives that skip Group (Collision+Attach+Rrt) deserialize cleanly.
        foreach (var (name, show) in new (string, bool)[]
                 {
                     (MotusPlanInputs.Collision, _showCollision),
                     (MotusPlanInputs.Group, _showGroup),
                     (MotusPlanInputs.Attach, _showAttach),
                     (MotusPlanInputs.RrtSettings, _showRrtSettings)
                 })
        {
            if (show && !MotusPlanInputs.Has(this, name))
                return CreateAdvancedParam(name);
        }

        // Legacy docs without Show* flags: next missing pin in canonical order.
        foreach (var name in new[]
                 {
                     MotusPlanInputs.Collision,
                     MotusPlanInputs.Group,
                     MotusPlanInputs.Attach,
                     MotusPlanInputs.RrtSettings
                 })
        {
            if (!MotusPlanInputs.Has(this, name))
                return CreateAdvancedParam(name);
        }
        return CreateAdvancedParam(MotusPlanInputs.Collision);
    }

    public bool DestroyParameter(GH_ParameterSide side, int index) =>
        side == GH_ParameterSide.Input && index >= CoreInputCount;

    public void VariableParameterMaintenance() => SyncFlagsFromExistingParams();

    /// <summary>Enable advanced pins for example generators / document restore.</summary>
    public void EnsureAdvancedInput(string name)
    {
        switch (name)
        {
            case MotusPlanInputs.Collision: _showCollision = true; break;
            case MotusPlanInputs.Group: _showGroup = true; break;
            case MotusPlanInputs.Attach: _showAttach = true; break;
            case MotusPlanInputs.RrtSettings: _showRrtSettings = true; break;
            default: return;
        }
        EnsureAdvancedParams();
    }

    internal bool IsCollisionPortWired() => MotusPlanInputs.IsWired(this, MotusPlanInputs.Collision);

    internal bool HasObstacleAwareInputsWired() =>
        IsCollisionPortWired() || MotusPlanInputs.IsWired(this, MotusPlanInputs.Attach);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (IsReadyToSetData)
        {
            // Runtime messages must be added during SolveInstance — GH clears them at solution start,
            // so reporting from CommitWorkerCachedResults (pre-ExpireSolution) never sticks.
            base.SolveInstance(da);
            _activeWorkerFingerprint = null;
            ReportCachedFailuresIfNeeded();
            foreach (var remark in _pendingRemarks)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, remark);
            return;
        }

        var robotIdx = MotusPlanInputs.IndexOf(this, MotusPlanInputs.Robot);
        var goalIdx = MotusPlanInputs.IndexOf(this, MotusPlanInputs.Goal);
        var stepIdx = MotusPlanInputs.IndexOf(this, MotusPlanInputs.Step);
        var collisionIdx = MotusPlanInputs.IndexOf(this, MotusPlanInputs.Collision);

        if (robotIdx < 0 || !GhExtract.TryRobotGoo(da, robotIdx, out _))
        {
            InvalidateCachedPlan();
            return;
        }

        if (goalIdx < 0)
        {
            InvalidateCachedPlan();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Provide at least one valid Plane or Joint State goal.");
            EmitOutputs(da, GhExtract.PlanStatusKind.Manual, emitCache: false, statusOverride: "Fix goal input errors.");
            return;
        }

        if (!GhExtract.TryGoals(da, goalIdx, out var goals, out var goalErrors))
        {
            InvalidateCachedPlan();
            foreach (var error in goalErrors)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, error);
            if (goals.Count == 0)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Provide at least one valid Plane or Joint State goal.");
            EmitOutputs(da, GhExtract.PlanStatusKind.Manual, emitCache: false, statusOverride: "Fix goal input errors.");
            return;
        }

        var stepInput = DefaultLinStepMeters;
        if (stepIdx >= 0 && da.GetData(stepIdx, ref stepInput))
        {
            if (stepInput <= 0)
            {
                InvalidateCachedPlan();
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Step must be positive for plane goals.");
                EmitOutputs(da, GhExtract.PlanStatusKind.Manual, emitCache: false, statusOverride: "Fix Step input (must be positive).");
                return;
            }
        }

        var collision = GhExtract.ParseCollisionInput(da, collisionIdx);
        if (collision.Error is not null)
        {
            InvalidateCachedPlan();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, collision.Error);
            EmitOutputs(da, GhExtract.PlanStatusKind.Manual, emitCache: false, statusOverride: "Fix Collision input errors.");
            return;
        }
        else if (collision.Warning is not null)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, collision.Warning);

        if (!PlanInputSnapshot.TryCollect(da, this, out var snapshot, out var collectError) || snapshot is null)
        {
            InvalidateCachedPlan();
            if (collectError is not null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, collectError);
                EmitOutputs(da, GhExtract.PlanStatusKind.Manual, emitCache: false, statusOverride: collectError);
            }
            return;
        }

        GhExtract.RemarkIfDefaultStart(this, snapshot.UsedDefaultStart);
        if (goals.Any(g => g.plane is not null))
            AddRuntimeMessage(
                GH_RuntimeMessageLevel.Remark,
                "Plane goals use TCP LIN. For PTP/CIRC/SET/WAIT use Motus Move → Motus Program.");

        var fingerprint = snapshot.Fingerprint;
        var planningContext = snapshot.PlanningContext;
        var rrtSettings = snapshot.RrtSettings;

        // Immediate reachability for plane goals — do not wait for Plan.
        var reachErrors = GhExtract.CollectPlaneGoalReachErrors(snapshot.Context, snapshot.Start, goals);
        if (reachErrors.Count > 0)
        {
            _run = false;
            _debounceGen++;
            _planningPending = false;
            InvalidateCachedPlan();
            foreach (var error in reachErrors)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, error);
            EmitOutputs(
                da,
                GhExtract.PlanStatusKind.Manual,
                emitCache: false,
                statusOverride: string.Join(" | ", reachErrors));
            return;
        }

        if (IsOperationInProgress && _activeWorkerFingerprint is not null && _activeWorkerFingerprint != fingerprint)
            RequestCancellation();

        var collisionInputWired = snapshot.CollisionInputWired;
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
            EmitOutputs(da, idleKind, idleActivity, fingerprint: fingerprint);
            return;
        }

        _planningPending = false;
        _activeWorkerFingerprint = fingerprint;
        Message = "Planning…";
        OnDisplayExpired(true);
        LaunchWorker(da, snapshot);
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
        // Defer runtime messages until the SetData SolveInstance pass (see SolveInstance).
        _pendingRemarks = remarks.ToList();
    }

    internal void ReportCachedFailuresIfNeeded()
    {
        if (_cached is not null)
            ReportPlanningFailures(_cached);
    }

    private void ToggleAdvanced(string name, ref bool flag)
    {
        RecordUndoEvent($"Toggle {name}");
        flag = !flag;
        EnsureAdvancedParams();
        ExpireSolution(true);
    }

    private void EnsureAdvancedParams()
    {
        SetAdvancedPresent(MotusPlanInputs.Collision, _showCollision, CreateAdvancedParam);
        SetAdvancedPresent(MotusPlanInputs.Group, _showGroup, CreateAdvancedParam);
        SetAdvancedPresent(MotusPlanInputs.Attach, _showAttach, CreateAdvancedParam);
        SetAdvancedPresent(MotusPlanInputs.RrtSettings, _showRrtSettings, CreateAdvancedParam);
        Params.OnParametersChanged();
        VariableParameterMaintenance();
    }

    private void SetAdvancedPresent(string name, bool show, Func<string, IGH_Param> factory)
    {
        var existing = MotusPlanInputs.IndexOf(this, name);
        if (show && existing < 0)
            Params.RegisterInputParam(factory(name));
        else if (!show && existing >= 0)
            Params.UnregisterInputParameter(Params.Input[existing]);
    }

    private static IGH_Param CreateAdvancedParam(string name) => name switch
    {
        MotusPlanInputs.Collision => new Param_MotusCollisionScene
        {
            Name = MotusPlanInputs.Collision,
            NickName = "C",
            Description = "Obstacle-aware planning: ColScene → Collision. Joint goals use RRT; plane goals use LIN validate",
            Access = GH_ParamAccess.item,
            Optional = true
        },
        MotusPlanInputs.Group => new Param_GenericObject
        {
            Name = MotusPlanInputs.Group,
            NickName = "Gr",
            Description = "Optional planning group (locks non-group joints)",
            Access = GH_ParamAccess.item,
            Optional = true
        },
        MotusPlanInputs.Attach => new Param_GenericObject
        {
            Name = MotusPlanInputs.Attach,
            NickName = "A",
            Description = "Optional attached bodies list",
            Access = GH_ParamAccess.list,
            Optional = true
        },
        MotusPlanInputs.RrtSettings => new Param_GenericObject
        {
            Name = MotusPlanInputs.RrtSettings,
            NickName = "Rrt",
            Description = "Optional RRT tuning from Motus RRT Settings (joint goals + collision only)",
            Access = GH_ParamAccess.item,
            Optional = true
        },
        _ => new Param_GenericObject { Name = name, NickName = name, Optional = true }
    };

    private void SyncFlagsFromExistingParams()
    {
        _showCollision = MotusPlanInputs.Has(this, MotusPlanInputs.Collision);
        _showGroup = MotusPlanInputs.Has(this, MotusPlanInputs.Group);
        _showAttach = MotusPlanInputs.Has(this, MotusPlanInputs.Attach);
        _showRrtSettings = MotusPlanInputs.Has(this, MotusPlanInputs.RrtSettings);
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
        if (_autoPlan && HasObstacleAwareInputsWired())
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

    private void InvalidateCachedPlan()
    {
        if (IsOperationInProgress)
            RequestCancellation();
        _cached = null;
        _cachedGoos = null;
        _pendingRemarks = [];
        _lastPlannedFingerprint = null;
        _activeWorkerFingerprint = null;
        _planningPending = false;
    }

    private GhExtract.PlanStatusKind ResolveIdleStatusKind(string fingerprint)
    {
        if (IsOperationInProgress || _planningPending || (_autoPlan && fingerprint != _lastPlannedFingerprint))
            return GhExtract.PlanStatusKind.Planning;
        if (_autoPlan)
            return GhExtract.PlanStatusKind.AutoCached;
        return _cached is null ? GhExtract.PlanStatusKind.Manual : GhExtract.PlanStatusKind.ManualCached;
    }

    private void EmitOutputs(
        IGH_DataAccess da,
        GhExtract.PlanStatusKind statusKind,
        string? activity = null,
        bool emitCache = true,
        string? statusOverride = null,
        string? fingerprint = null)
    {
        if (emitCache)
        {
            if (statusKind != GhExtract.PlanStatusKind.Planning && _cached is not null)
                ReportCachedFailuresIfNeeded();

            if (_cachedGoos is { Count: > 0 })
                da.SetDataList(0, _cachedGoos);

            if (statusKind == GhExtract.PlanStatusKind.Planning && _cachedGoos is { Count: > 0 })
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                    "Trajectory is from previous inputs; replanning in background…");
            }
            else if (statusKind == GhExtract.PlanStatusKind.ManualCached &&
                     fingerprint is not null &&
                     _lastPlannedFingerprint is not null &&
                     fingerprint != _lastPlannedFingerprint &&
                     _cachedGoos is { Count: > 0 })
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                    "Inputs changed — click Plan (or enable Auto Plan) to update Trajectory.");
            }

            da.SetData(1, statusOverride ?? GhExtract.BuildStatusMessage(_cached, statusKind, activity));
            if (_cached is not null)
                da.SetDataList(2, GhExtract.BuildWarnings(_cached));
            return;
        }

        da.SetDataList(0, Array.Empty<TrajectoryGoo>());
        da.SetData(1, statusOverride ?? "Fix input errors.");
        da.SetDataList(2, Array.Empty<string>());
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
                    "Plan checks URDF collision meshes (often larger than visuals) plus tool collision hull. " +
                    "Right-click the robot component → Preview collision meshes to compare. " +
                    "Confirm Plan Start matches the ShowStart ghost pose.");
            }
        }
    }

    public override Guid ComponentGuid => new Guid("8bb0bae3-527f-4e80-a8a4-c8a88b7276de");

    protected override string FormatProgressMessage(double fraction) =>
        fraction switch
        {
            >= 0.999 => "Done",
            <= 0.001 => "Planning…",
            >= 0.90 => "Planning… RRT (busy)",
            _ => $"Planning… {(fraction * 100):0}%"
        };
}
