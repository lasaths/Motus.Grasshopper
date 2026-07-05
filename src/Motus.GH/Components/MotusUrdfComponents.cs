using Grasshopper.Kernel;
using Motus.Core;
using Motus.GH.Data;
using Motus.Presets;

namespace Motus.GH.Components;

public sealed class MotusLoadUrdfComponent : MotusComponentBase
{
    public MotusLoadUrdfComponent() : base("Motus Load URDF", "URDF", "Load a serial-chain URDF into a robot model", "Model", "file") { }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddTextParameter("Path", "P", "Path to .urdf file", GH_ParamAccess.item);
        p.AddTextParameter("BaseLink", "B", "Base link name", GH_ParamAccess.item, "base_link");
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("TipLink", "T", "Tip link name", GH_ParamAccess.item, "tool0");
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddGenericParameter("Robot", "Rb", "Robot model with URDF kinematics chain", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var path = "";
        var baseLink = "base_link";
        var tipLink = "tool0";
        if (!da.GetData(0, ref path) || string.IsNullOrWhiteSpace(path)) return;
        da.GetData(1, ref baseLink);
        da.GetData(2, ref tipLink);

        try
        {
            var urdf = UrdfRobotLoader.Load(path, new UrdfLoadOptions
            {
                BaseLink = baseLink,
                TipLink = tipLink,
                ModelName = Path.GetFileNameWithoutExtension(path)
            });
            da.SetData(0, RobotModelGoo.FromUrdf(urdf));
        }
        catch (Exception ex)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
        }
    }

    public override Guid ComponentGuid => new Guid("c8e4a1b2-3f5d-4e6a-9b7c-1d2e3f4a5b6c");
}
