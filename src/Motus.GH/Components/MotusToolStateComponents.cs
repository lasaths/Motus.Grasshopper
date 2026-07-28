using System.Drawing;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Parameters;
using Grasshopper.Kernel.Types;
using GH_IO.Serialization;
using Motus.Core;
using Motus.GH;
using Motus.GH.Data;
using Motus.GH.Params;
using Motus.GH.UI;

namespace Motus.GH.Components;

/// <summary>
/// Motus Tool State — EndEffectorState from Cap schema. Preset is on-component.
/// Pins stay stable (Width always optional) so GHX wires survive Preset changes.
/// </summary>
public sealed class MotusToolStateComponent : MotusComponentBase
{
    private static readonly string[] Presets = ["Open", "Closed", "Custom"];

    private string _preset = "Open";
    private PointF? _canvasPivot;

    public MotusToolStateComponent()
        : base(
            "Motus Tool State",
            "ToolState",
            "Build end-effector parameter values from Motus Tool Cap schema (e.g. Robotiq jaw width). Preset dropdown on component.",
            "Model",
            "sliders-horizontal") { }

    public override void CreateAttributes()
    {
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
        p.AddGenericParameter("Tool", "Tl", "Motus Tool or Robot (uses Robot.Tool / bundled Cap schema)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Width", "W", "Jaw width (m) when Preset=Custom", GH_ParamAccess.item, 0.085);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Speed", "Sp", "Grip speed ratio 0–1 (export hint)", GH_ParamAccess.item, 0.5);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Force", "F", "Grip force ratio 0–1 (export hint)", GH_ParamAccess.item, 0.5);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddParameter(new Param_MotusToolState(), "State", "Ts", "End-effector state", GH_ParamAccess.item);

    public override bool Write(GH_IWriter writer)
    {
        writer.SetString("ToolStatePreset", _preset);
        if (Attributes is not null)
        {
            writer.SetDouble("CanvasPivotX", Attributes.Pivot.X);
            writer.SetDouble("CanvasPivotY", Attributes.Pivot.Y);
        }

        return base.Write(writer);
    }

    public override bool Read(GH_IReader reader)
    {
        if (reader.ItemExists("ToolStatePreset"))
            _preset = NormalizePreset(reader.GetString("ToolStatePreset"));
        if (reader.ItemExists("CanvasPivotX") && reader.ItemExists("CanvasPivotY"))
        {
            _canvasPivot = new PointF(
                (float)reader.GetDouble("CanvasPivotX"),
                (float)reader.GetDouble("CanvasPivotY"));
        }

        var ok = base.Read(reader);
        if (Attributes is not null)
        {
            var p = Attributes.Pivot;
            if (p.X != 0 || p.Y != 0)
                _canvasPivot = p;
        }

        MigrateLegacyPresetPin();
        var presetIdx = IndexOf("Preset");
        if (presetIdx >= 0)
            Params.UnregisterInputParameter(Params.Input[presetIdx]);

        RestoreCanvasPivot();
        return ok;
    }

    private void MigrateLegacyPresetPin()
    {
        var idx = IndexOf("Preset");
        if (idx < 0) return;
        var param = Params.Input[idx];
        if (param.SourceCount == 0 && param is Param_String ps && ps.PersistentDataCount > 0)
        {
            var v = ps.PersistentData.get_FirstItem(false)?.Value;
            if (!string.IsNullOrWhiteSpace(v))
                _preset = NormalizePreset(v);
        }
    }

    private void RestoreCanvasPivot()
    {
        if (_canvasPivot is not { } p || Attributes is null) return;
        Attributes.Pivot = p;
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!TryResolveTool(da, 0, out var tool, out var sourceKind))
            return;

        if (!ToolCapContract.TryResolveForToolState(
                tool,
                toolOrRobotWired: sourceKind != ToolSource.None,
                out var caps,
                out var error,
                out var warning))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, error!);
            return;
        }

        if (warning is not null)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, warning);

        var width = 0.085;
        var speed = 0.5;
        var force = 0.5;
        da.GetData(1, ref width);
        da.GetData(2, ref speed);
        da.GetData(3, ref force);

        var openWidth = caps.Parameters.FirstOrDefault(p =>
            string.Equals(p.Name, "width", StringComparison.Ordinal))?.Max ?? 0.085;

        var preset = NormalizePreset(_preset);
        Message = preset;
        width = preset switch
        {
            "Closed" => 0,
            "Open" => openWidth,
            _ => width
        };

        var values = new Dictionary<string, double>(StringComparer.Ordinal)
        {
            ["width"] = width,
            ["speed"] = speed,
            ["force"] = force
        };

        var state = caps.Clamp(new EndEffectorState(values));
        foreach (var err in caps.Validate(state))
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, err);

        da.SetData(0, new EndEffectorStateGoo(state));
    }

    private DropDownAttributes.Model BuildDropdownModel() =>
        new(["Preset"], [Presets], [NormalizePreset(_preset)]);

    private void OnDropdownSelect(int listIndex, int itemIndex)
    {
        if (listIndex != 0) return;
        if (itemIndex < 0 || itemIndex >= Presets.Length) return;
        var next = Presets[itemIndex];
        if (next == NormalizePreset(_preset)) return;
        RecordUndoEvent("Tool State Preset");
        _preset = next;
        ExpireSolution(true);
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

    private static string NormalizePreset(string? raw)
    {
        var t = (raw ?? "Open").Trim();
        if (t.Equals("Closed", StringComparison.OrdinalIgnoreCase)) return "Closed";
        if (t.Equals("Custom", StringComparison.OrdinalIgnoreCase)) return "Custom";
        return "Open";
    }

    private enum ToolSource { None, Tool, Robot }

    private bool TryResolveTool(IGH_DataAccess da, int index, out ToolDefinition? tool, out ToolSource source)
    {
        tool = null;
        source = ToolSource.None;
        IGH_Goo? goo = null;
        if (!da.GetData(index, ref goo) || goo is null)
            return true;

        if (goo is ToolGoo toolGoo)
        {
            tool = toolGoo.Value;
            source = ToolSource.Tool;
            if (tool is null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Tool input is empty.");
                return false;
            }

            return true;
        }

        if (goo is RobotModelGoo robotGoo)
        {
            robotGoo.EnsureBundledTool();
            tool = robotGoo.Tool;
            source = ToolSource.Robot;
            return true;
        }

        AddRuntimeMessage(
            GH_RuntimeMessageLevel.Error,
            "Tool input must be a Motus Tool or Motus Robot.");
        return false;
    }

    public override Guid ComponentGuid => new("a1b2c3d4-e5f6-7890-abcd-ef1234567890");
}
