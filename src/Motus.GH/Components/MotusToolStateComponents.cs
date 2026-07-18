using Grasshopper.Kernel;
using Motus.Core;
using Motus.GH.Data;
using Motus.GH.UI;

namespace Motus.GH.Components;

public sealed class MotusToolStateComponent : MotusComponentBase
{
    public MotusToolStateComponent()
        : base(
            "Motus Tool State",
            "ToolState",
            "Build end-effector parameter values (e.g. Robotiq jaw width)",
            "Model",
            "sliders-horizontal") { }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Tool", "Tl", "Optional tool for validation and presets", GH_ParamAccess.item);
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
        p.AddGenericParameter("State", "Ts", "End-effector state", GH_ParamAccess.item);

    public override void AddedToDocument(GH_Document doc)
    {
        base.AddedToDocument(doc);
        if (Params.Input[1].SourceCount > 0) return;
        doc.ScheduleSolution(1, _ => GhValueList.AttachDropdown(this, 1, new[] { "Open", "Closed", "Custom" }, "Preset"));
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        ToolDefinition? tool = null;
        ToolGoo? toolGoo = null;
        if (da.GetData(0, ref toolGoo) && toolGoo?.Value is not null)
            tool = toolGoo.Value;
        else
        {
            AddRuntimeMessage(
                GH_RuntimeMessageLevel.Remark,
                "No Tool wired — assuming Robotiq 2F-85.");
        }

        var preset = "Open";
        var width = 0.085;
        var speed = 0.5;
        var force = 0.5;
        da.GetData(1, ref preset);
        da.GetData(2, ref width);
        da.GetData(3, ref speed);
        da.GetData(4, ref force);

        var caps = tool?.Capabilities ?? ToolCapabilities.Robotiq2F85;
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

    public override Guid ComponentGuid => new("a1b2c3d4-e5f6-7890-abcd-ef1234567890");
}
