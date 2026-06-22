using Grasshopper.Kernel;
using Motus.GH.Components;

namespace Motus.GH;

public sealed class MotusGhPlugin : GH_AssemblyInfo
{
    public override string Name => "Motus";
    public override string Version => "0.1.0";
    public override string AuthorName => "Motus";
    public override GH_LibraryLicense License => GH_LibraryLicense.opensource;
    public override string Description => "Motion planning for robotics (visualization only, no robot control).";
}
