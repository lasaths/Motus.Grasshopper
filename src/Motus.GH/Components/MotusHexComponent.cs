using Grasshopper.Kernel;
using Motus.Core;
using Motus.Geometry;
using Motus.GH.Data;
using Motus.GH.Params;
using Motus.GH.Preview;
using Rhino.Display;
using Rhino.Geometry;
using System.Drawing;
using System.Linq;

namespace Motus.GH.Components;

/// <summary>
/// Walking hex size &amp; stance (Family=legged). Wire <c>Hx</c> into Motus Walk Hex, or use Robot for tip-path Plan.
/// </summary>
public sealed class MotusHexComponent : RobotSourceComponentBase
{
    public static readonly Guid Id = new("c7a02fcb-2562-4540-9f44-5cc9e99293ec");

    private List<Color> _previewColors = [];
    private readonly Dictionary<Color, DisplayMaterial> _matCache = new();

    public MotusHexComponent()
        : base(
            "Motus Hex",
            "Hex",
            "Walking hex size & stance (Family=legged). Wire Hx → WalkHex; Rb → Plan tip-path (one leg).",
            "polygon")
    {
    }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddNumberParameter("BodyR", "Br", "Body hex radius to hip (m)", GH_ParamAccess.item, 0.06);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Coxa", "Cx", "Coxa length (m)", GH_ParamAccess.item, 0.035);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Femur", "Fm", "Femur length (m)", GH_ParamAccess.item, 0.08);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Tibia", "Tb", "Tibia length (m)", GH_ParamAccess.item, 0.10);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("BodyZ", "Bz", "Body height above ground (m)", GH_ParamAccess.item, 0.07);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("HipStance", "Hs", "Coxa stance angle (rad, signed by leg side)", GH_ParamAccess.item, 7.5 * Math.PI / 180.0);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("FemurStance", "Fs", "Fallback femur angle (rad) if ground-plant IK fails", GH_ParamAccess.item, 30.0 * Math.PI / 180.0);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("TibiaStance", "Ts", "Fallback tibia angle (rad) if ground-plant IK fails", GH_ParamAccess.item, -30.0 * Math.PI / 180.0);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Q", "Q", "Optional full driver q (18): leg-major coxa,femur,tibia × 6", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddParameter(new Param_MotusHex(), "Hex", "Hx", "Size & stance → Motus Walk Hex", GH_ParamAccess.item);
        p.AddGenericParameter("Robot", "Rb", "Robot (Plan tip-path = right-middle 3-DOF; TreeFK = 18)", GH_ParamAccess.item);
        p.AddParameter(new Param_MotusJointState(), "State", "Js", "Full 18-DOF stance", GH_ParamAccess.item);
        p.AddMeshParameter("Meshes", "M", "Stance preview meshes", GH_ParamAccess.list);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var br = 0.06;
        var cx = 0.035;
        var fm = 0.08;
        var tb = 0.10;
        var bz = 0.07;
        var hs = 7.5 * Math.PI / 180.0;
        var fs = 30.0 * Math.PI / 180.0;
        var ts = -30.0 * Math.PI / 180.0;
        var qIn = new List<double>();
        da.GetData(0, ref br);
        da.GetData(1, ref cx);
        da.GetData(2, ref fm);
        da.GetData(3, ref tb);
        da.GetData(4, ref bz);
        da.GetData(5, ref hs);
        da.GetData(6, ref fs);
        da.GetData(7, ref ts);
        da.GetDataList(8, qIn);

        if (!double.IsFinite(br) || !double.IsFinite(cx) || !double.IsFinite(fm) || !double.IsFinite(tb)
            || !double.IsFinite(bz) || !double.IsFinite(hs) || !double.IsFinite(fs) || !double.IsFinite(ts))
        {
            ClearPreview();
            _previewColors = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "BodyR / Coxa / Femur / Tibia / BodyZ / stance must be finite.");
            return;
        }

        if (qIn.Count is > 0 and < 18)
        {
            ClearPreview();
            _previewColors = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Q must be empty or exactly 18 driver values.");
            return;
        }

        if (qIn.Any(v => !double.IsFinite(v)))
        {
            ClearPreview();
            _previewColors = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Q contains non-finite values.");
            return;
        }

        if (br <= 0 || cx <= 0 || fm <= 0 || tb <= 0)
        {
            ClearPreview();
            _previewColors = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "BodyR / Coxa / Femur / Tibia must be > 0 (m).");
            return;
        }

        try
        {
            var layout = LeggedLayout.HexMithi(br, cx, fm, tb, bz);
            var q = WalkingHexShared.BuildStanceQ(layout, hs, fs, ts, qIn);
            var desc = WalkingHexShared.BuildDescription(layout);
            var tree = desc.ToKinematicTree();
            var tip = tree.ExtractSerialTip("body", layout.TipLinkName);
            var tipLimits = WalkingHexShared.LimitsAlongTip(tree, tip.JointNames);
            var driverNames = WalkingHexShared.DriverNames(tree);

            var tipHome = new double[tip.Chain.Joints.Length];
            for (var i = 0; i < tipHome.Length && i < 3; i++)
                tipHome[i] = q[i];

            var preset = new RobotPreset
            {
                Manufacturer = RobotManufacturer.Unknown,
                ModelName = "walking_hexapod",
                Family = Units.LeggedFamily,
                AxisCount = tip.Chain.Joints.Length,
                JointLimits = tipLimits,
                BaseFrame = BaseFrame.Identity,
                ToolFrame = tip.TipToolOffset is { } off
                    ? new ToolFrame(off, "foot")
                    : ToolFrame.Identity,
                Notes = "Walking hexapod Family=legged. Motus Hex → Plan tip-path (one leg) or WalkHex Hx for gait.",
                SourceNote = "Motus Hex",
            };

            var model = new RobotModel(preset, jointNames: driverNames);
            var robotGoo = new RobotModelGoo(model)
            {
                Chain = tip.Chain,
                Tree = tree,
                PreviewHome = new JointState(tipHome),
                TreeDriverHome = new JointState(q),
                PreviewGeometry = MechanismPreviewGeometry.Build(desc),
            };

            var hexGoo = new HexLayoutGoo(layout)
            {
                HipStance = hs,
                FemurStance = fs,
                TibiaStance = ts,
                DriverQ = qIn.Count >= 18 ? qIn.ToArray() : null,
            };

            var preview = WalkingHexPreview.Build(layout, q);
            _previewMeshes = preview.Meshes.ToList();
            _previewWires = preview.Wires.ToList();
            _previewColors = preview.Colors.ToList();
            ExpirePreview(true);

            da.SetData(0, hexGoo);
            da.SetData(1, robotGoo);
            da.SetData(2, new JointStateGoo(new JointState(q)));
            da.SetDataList(3, _previewMeshes);
        }
        catch (Exception ex)
        {
            ClearPreview();
            _previewColors = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
        }
    }

    public override void DrawViewportMeshes(IGH_PreviewArgs args)
    {
        if (Locked || _previewMeshes.Count == 0) return;
        for (var i = 0; i < _previewMeshes.Count; i++)
        {
            var c = i < _previewColors.Count ? _previewColors[i] : Color.White;
            if (!_matCache.TryGetValue(c, out var mat))
            {
                mat = new DisplayMaterial(c) { Transparency = 0.2 };
                _matCache[c] = mat;
            }
            args.Display.DrawMeshShaded(_previewMeshes[i], mat);
        }
    }

    public override void DrawViewportWires(IGH_PreviewArgs args)
    {
        if (Locked || _previewWires.Count == 0) return;
        foreach (var line in _previewWires)
            args.Display.DrawLine(line, Color.FromArgb(200, 255, 140, 40), 2);
    }

    public override Guid ComponentGuid => Id;
}
