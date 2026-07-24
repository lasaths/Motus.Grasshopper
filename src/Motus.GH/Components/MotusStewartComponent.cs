using Motus.Core;
using Motus.Geometry;
using Motus.GH.Data;
using Motus.GH.Rhino;
using Rhino.Geometry;
using Grasshopper.Kernel;

namespace Motus.GH.Components;

/// <summary>
/// Stewart/Gough platform robot (<c>Family=stewart</c>). Classic hex geometry or JSON description path.
/// </summary>
public sealed class MotusStewartComponent : RobotSourceComponentBase
{
    public MotusStewartComponent()
        : base(
            "Motus Stewart",
            "Stewart",
            "Stewart/Gough hexapod (6 prismatic legs). Family=stewart; JointState = leg lengths in meters. Wire to Motus Plan with TCP plane goals.",
            "stack")
    {
    }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddTextParameter(
            "Path",
            "P",
            "Optional Stewart JSON (schemaVersion=1). Leave empty for classic hex geometry.",
            GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("BaseRadius", "Br", "Classic base anchor radius (m)", GH_ParamAccess.item, 0.5);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("PlatformRadius", "Pr", "Classic platform anchor radius (m)", GH_ParamAccess.item, 0.3);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("MinStroke", "Lmin", "Min leg length (m)", GH_ParamAccess.item, 0.35);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("MaxStroke", "Lmax", "Max leg length (m)", GH_ParamAccess.item, 0.90);
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("Name", "N", "Model name", GH_ParamAccess.item, "stewart_classic");
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddGenericParameter("Robot", "Rb", "Stewart robot (Family=stewart)", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var path = "";
        da.GetData(0, ref path);
        var br = 0.5;
        var pr = 0.3;
        var lmin = 0.35;
        var lmax = 0.90;
        var name = "stewart_classic";
        da.GetData(1, ref br);
        da.GetData(2, ref pr);
        da.GetData(3, ref lmin);
        da.GetData(4, ref lmax);
        da.GetData(5, ref name);

        try
        {
            StewartRobot stewart;
            if (!string.IsNullOrWhiteSpace(path))
            {
                if (!File.Exists(path))
                {
                    ClearPreview();
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $"Stewart JSON not found: {path}");
                    return;
                }
                stewart = StewartRobot.LoadFile(path);
            }
            else
            {
                stewart = StewartRobot.CreateClassic(name, br, pr, lmin, lmax);
            }

            var mid = 0.5 * (stewart.Platform.StrokeLimits[0].Min + stewart.Platform.StrokeLimits[0].Max);
            var homePose = new CartesianPose(new Frame(0, 0, mid));
            var homeIk = stewart.InverseKinematics.TrySolveDetailed(homePose);
            var home = homeIk.Success && homeIk.JointState is not null
                ? homeIk.JointState
                : stewart.Platform.HomeLengths();

            var goo = new RobotModelGoo(stewart.Model)
            {
                Stewart = stewart.Platform,
                PreviewHome = home
            };

            // Wireframe preview: base + platform + legs at home.
            _previewWires = KinematicsPreview.StewartLegLines(stewart.Platform, home, homePose).ToList();
            _previewMeshes = [];
            ExpirePreview(true);

            da.SetData(0, goo);
            if (!homeIk.Success)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, $"Home IK: {homeIk}");
        }
        catch (Exception ex)
        {
            ClearPreview();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
        }
    }

    public override Guid ComponentGuid => new("a9e1c3f0-7b2d-4e8a-9c1f-6d4b2a0e8f73");
}
