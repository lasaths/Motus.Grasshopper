using Grasshopper;
using Grasshopper.Kernel;
using Motus.GH.Resources;
using System.Drawing;

namespace Motus.GH;

public sealed class MotusGhPlugin : GH_AssemblyInfo
{
    /// <summary>Stable GHA library id — must match MOTUS_LIB in scripts/generate-examples.mjs / validate-ghx.mjs.</summary>
    public override Guid Id => new("dc547e55-81a8-c313-e25d-e1468ddecddb");
    public override string Name => "Motus";
    public override string Version => "0.7.2";
    public override string AuthorName => "Motus";
    public override GH_LibraryLicense License => GH_LibraryLicense.opensource;
    public override string Description => "Motion planning for robotics (visualization only, no robot control).";
    // ponytail: never throw from GH_AssemblyInfo getters — GH wraps that as GHA load failure
    public override Bitmap? Icon
    {
        get
        {
            try { return MotusIcon.GetAssembly(); }
            catch { return null; }
        }
    }
}

/// <summary>Registers the ribbon tab icon and shortcut letter for the Motus category.</summary>
public sealed class MotusCategoryIcon : GH_AssemblyPriority
{
    public override GH_LoadingInstruction PriorityLoad()
    {
        try
        {
            var server = Instances.ComponentServer;
            if (server is null) return GH_LoadingInstruction.Proceed;
            server.AddCategoryIcon("Motus", MotusIcon.GetCategoryTab());
            server.AddCategorySymbolName("Motus", 'M');
        }
        catch
        {
            // Tab chrome is optional; components still register.
        }

        return GH_LoadingInstruction.Proceed;
    }
}
