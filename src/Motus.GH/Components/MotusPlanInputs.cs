using Grasshopper.Kernel;

namespace Motus.GH.Components;

/// <summary>Name-based Plan input indices (advanced Collision/Group/Attach/Rrt are variable).</summary>
internal static class MotusPlanInputs
{
    internal const string Robot = "Robot";
    internal const string Goal = "Goal";
    internal const string Start = "Start";
    internal const string Step = "Step";
    internal const string Collision = "Collision";
    internal const string Group = "Group";
    internal const string Attach = "Attach";
    internal const string RrtSettings = "RrtSettings";

    internal static int IndexOf(IGH_Component component, string name)
    {
        for (var i = 0; i < component.Params.Input.Count; i++)
        {
            if (string.Equals(component.Params.Input[i].Name, name, StringComparison.Ordinal))
                return i;
        }
        return -1;
    }

    internal static bool Has(IGH_Component component, string name) => IndexOf(component, name) >= 0;

    internal static bool IsWired(IGH_Component component, string name)
    {
        var i = IndexOf(component, name);
        return i >= 0 && component.Params.Input[i].SourceCount > 0;
    }
}
