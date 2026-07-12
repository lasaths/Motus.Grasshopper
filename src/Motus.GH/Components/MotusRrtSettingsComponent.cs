using Grasshopper.Kernel;
using Motus.GH.Data;
using Motus.GH.Planning;
using Motus.OMPL.NET;
using System.Linq;

namespace Motus.GH.Components;

public sealed class MotusRrtSettingsComponent : MotusComponentBase
{
    private const int PlannerInputIndex = 2;

    public MotusRrtSettingsComponent()
        : base(
            "Motus RRT Settings",
            "RrtSet",
            "Tune sampling planners for joint goals with collision; wire Settings into Motus Plan",
            "Plan",
            "faders") { }

    public override void AddedToDocument(GH_Document doc)
    {
        base.AddedToDocument(doc);
        var available = SamplingPlannerRegistry.ListAvailable()
            .Select(d => d.ShortName)
            .ToArray();
        if (available.Length == 0)
            available = ["RrtConnect"];
        GhValueList.AttachDropdown(this, PlannerInputIndex, available, "Planner");
    }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddIntegerParameter("MaxIter", "Mi", "Max sampling iterations", GH_ParamAccess.item, 4000);
        p.AddNumberParameter("TimeLimit", "T", "Max plan time in seconds (0 = no limit)", GH_ParamAccess.item, 0);
        p.AddTextParameter("Planner", "P", "Sampling planner from registry", GH_ParamAccess.item, "RrtConnect");
        p.AddNumberParameter("GoalBias", "Gb", "Goal bias 0–1", GH_ParamAccess.item, 0.08);
        p.AddNumberParameter("Step", "St", "Tree step size (rad)", GH_ParamAccess.item, 0.12);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddGenericParameter("Settings", "S", "Sampling planner settings for Motus Plan", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!RrtPlanSettings.TryBuild(da, out var settings, out var error))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, error!);
            return;
        }

        if (settings.MaxPlanTimeSeconds <= 0 && settings.MaxIterations >= 8000)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                "No time limit — large MaxIter can feel slow. Set TimeLimit (s) for interactive tuning.");
        }

        da.SetData(0, new RrtPlanSettingsGoo(settings));
    }

    public override Guid ComponentGuid => new("11d59b15-ffe2-488e-83b8-52eddf772025");
}
