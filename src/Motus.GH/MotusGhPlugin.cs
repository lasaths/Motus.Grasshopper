using Grasshopper;
using Grasshopper.Kernel;
using Motus.GH.Resources;

namespace Motus.GH;

public sealed class MotusGhPlugin : GH_AssemblyInfo
{
    public override string Name => "Motus";
    public override string Version => "0.3.3";
    public override string AuthorName => "Motus";
    public override GH_LibraryLicense License => GH_LibraryLicense.opensource;
    public override string Description => "Motion planning for robotics (visualization only, no robot control).";
}

/// <summary>Registers the ribbon tab icon and shortcut letter for the Motus category.</summary>
public sealed class MotusCategoryIcon : GH_AssemblyPriority
{
    public override GH_LoadingInstruction PriorityLoad()
    {
        Instances.ComponentServer.AddCategoryIcon("Motus", MotusIcon.Get("robot"));
        Instances.ComponentServer.AddCategorySymbolName("Motus", 'M');
        return GH_LoadingInstruction.Proceed;
    }
}
