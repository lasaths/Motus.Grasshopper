using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Motus.Core;
using Motus.GH.Data;
using Motus.GH.Params;
using Motus.GH.UI;

namespace Motus.GH.Components;

public sealed class MotusToolStateComponent : MotusComponentBase
{
    public MotusToolStateComponent()
        : base(
            "Motus Tool State",
            "ToolState",
            "Build end-effector parameter values (e.g. Robotiq jaw width). Wire Motus Tool or Robot for capabilities.",
            "Model",
            "sliders-horizontal") { }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        // Generic: accepts ToolGoo or RobotModelGoo (session tool / bundled Robotiq).
        p.AddGenericParameter("Tool", "Tl", "Motus Tool or Robot (uses Robot.Tool / bundled capabilities)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("Preset", "P", "Open, Closed, or Custom", GH_ParamAccess.item, "Open");
        p.AddNumberParameter("Width", "W", "Jaw width (m) when Preset=Custom", GH_ParamAccess.item, 0.085);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Speed", "Sp", "Grip speed ratio 0–1", GH_ParamAccess.item, 0.5);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Force", "F", "Grip force ratio 0–1", GH_ParamAccess.item, 0.5);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddParameter(new Param_MotusToolState(), "State", "Ts", "End-effector state", GH_ParamAccess.item);

    public override void AddedToDocument(GH_Document doc)
    {
        base.AddedToDocument(doc);
        if (Params.Input[1].SourceCount > 0) return;
        doc.ScheduleSolution(1, _ => GhValueList.AttachDropdown(this, 1, new[] { "Open", "Closed", "Custom" }, "Preset"));
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!TryResolveTool(da, 0, out var tool, out var sourceKind))
            return;

        ToolCapabilities caps;
        if (tool?.Capabilities is { } toolCaps)
        {
            caps = toolCaps;
        }
        else if (sourceKind == ToolSource.None)
        {
            AddRuntimeMessage(
                GH_RuntimeMessageLevel.Warning,
                "No Tool/Robot wired — assuming Robotiq 2F-85. Wire Motus Tool or Motus UR10e/Robot for real capabilities.");
            caps = ToolCapabilities.Robotiq2F85;
        }
        else
        {
            AddRuntimeMessage(
                GH_RuntimeMessageLevel.Warning,
                "Wired tool has no Capabilities — assuming Robotiq 2F-85. Set Motus Tool Cap to Robotiq2F85.");
            caps = ToolCapabilities.Robotiq2F85;
        }

        var preset = "Open";
        var width = 0.085;
        var speed = 0.5;
        var force = 0.5;
        da.GetData(1, ref preset);
        da.GetData(2, ref width);
        da.GetData(3, ref speed);
        da.GetData(4, ref force);

        var openWidth = caps.Parameters.FirstOrDefault(p =>
            string.Equals(p.Name, "width", StringComparison.Ordinal))?.Max ?? 0.085;

        width = preset.Trim().Equals("Closed", StringComparison.OrdinalIgnoreCase)
            ? 0
            : preset.Trim().Equals("Open", StringComparison.OrdinalIgnoreCase)
                ? openWidth
                : width;

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
            if (tool is null)
            {
                AddRuntimeMessage(
                    GH_RuntimeMessageLevel.Warning,
                    "Robot has no Tool attached — assuming Robotiq 2F-85 presets.");
            }
            return true;
        }

        AddRuntimeMessage(
            GH_RuntimeMessageLevel.Error,
            "Tool input must be a Motus Tool or Motus Robot.");
        return false;
    }

    public override Guid ComponentGuid => new("a1b2c3d4-e5f6-7890-abcd-ef1234567890");
}
