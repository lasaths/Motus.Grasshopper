using Motus.Core;
using Motus.Geometry;
using Motus.GH.Data;
using Motus.GH.Rhino;
using Rhino.Display;
using Rhino.Geometry;
using Grasshopper.Kernel;
using System.Drawing;

namespace Motus.GH.Components;

/// <summary>
/// Stewart/Gough platform robot (<c>Family=stewart</c>).
/// Priority: JSON Path → Base+Plat point lists → classic Br/Pr factory.
/// </summary>
public sealed class MotusStewartComponent : RobotSourceComponentBase
{
    private List<Color> _previewColors = [];
    private readonly Dictionary<Color, DisplayMaterial> _matCache = new();

    public MotusStewartComponent()
        : base(
            "Motus Stewart",
            "Stewart",
            "Stewart/Gough hexapod (6 prismatic legs). Family=stewart; JointState = leg lengths in meters. Wire Base+Plat points or classic Br/Pr; Plan with TCP plane goals.",
            "stack")
    {
    }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddTextParameter(
            "Path",
            "P",
            "Optional Stewart JSON (schemaVersion=1). Leave empty for Base/Plat or classic hex.",
            GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddPointParameter(
            "Base",
            "Base",
            "Optional 6 base anchor points (m). With Plat → custom geometry (skips classic Br/Pr).",
            GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
        p.AddPointParameter(
            "Plat",
            "Plat",
            "Optional 6 platform anchor points in plate frame (m). With Base → custom geometry.",
            GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("BaseRadius", "Br", "Classic base anchor radius (m) when Base/Plat unwired", GH_ParamAccess.item, 0.5);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("PlatformRadius", "Pr", "Classic platform anchor radius (m) when Base/Plat unwired", GH_ParamAccess.item, 0.3);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("MinStroke", "Lmin", "Min leg length (m)", GH_ParamAccess.item, 0.35);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("MaxStroke", "Lmax", "Max leg length (m)", GH_ParamAccess.item, 0.90);
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("Name", "N", "Model name", GH_ParamAccess.item, "stewart_classic");
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter(
            "PairSep",
            "Sep",
            "Classic pair angular separation (rad) when Base/Plat unwired",
            GH_ParamAccess.item,
            0.15);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddGenericParameter("Robot", "Rb", "Stewart robot (Family=stewart)", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var path = "";
        da.GetData(0, ref path);
        var basePts = new List<Point3d>();
        var platPts = new List<Point3d>();
        da.GetDataList(1, basePts);
        da.GetDataList(2, platPts);
        var br = 0.5;
        var pr = 0.3;
        var lmin = 0.35;
        var lmax = 0.90;
        var name = "stewart_classic";
        var sep = 0.15;
        da.GetData(3, ref br);
        da.GetData(4, ref pr);
        da.GetData(5, ref lmin);
        da.GetData(6, ref lmax);
        da.GetData(7, ref name);
        da.GetData(8, ref sep);

        try
        {
            StewartRobot stewart;
            if (!string.IsNullOrWhiteSpace(path))
            {
                if (!File.Exists(path))
                {
                    ClearPreview();
                    _previewColors = [];
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $"Stewart JSON not found: {path}");
                    return;
                }
                stewart = StewartRobot.LoadFile(path);
            }
            else if (basePts.Count > 0 || platPts.Count > 0)
            {
                if (!TryAnchors(basePts, platPts, lmin, lmax, name, out stewart, out var err))
                {
                    ClearPreview();
                    _previewColors = [];
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, err!);
                    return;
                }
            }
            else
            {
                if (!double.IsFinite(sep) || sep <= 0 || sep >= Math.PI)
                {
                    ClearPreview();
                    _previewColors = [];
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "PairSep must be finite and in (0, π).");
                    return;
                }
                stewart = new StewartRobot(StewartPlatform.CreateClassic(
                    name, br, pr, lmin, lmax, sep, sep));
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

            var preview = KinematicsPreview.StewartPreview(stewart.Platform, home, homePose);
            _previewMeshes = preview.Meshes.ToList();
            _previewWires = preview.Wires.ToList();
            _previewColors = preview.Colors.ToList();
            ExpirePreview(true);

            da.SetData(0, goo);
            if (!homeIk.Success)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, $"Home IK: {homeIk}");
        }
        catch (Exception ex)
        {
            ClearPreview();
            _previewColors = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
        }
    }

    private static bool TryAnchors(
        List<Point3d> basePts,
        List<Point3d> platPts,
        double lmin,
        double lmax,
        string name,
        out StewartRobot stewart,
        out string? error)
    {
        stewart = null!;
        error = null;
        if (basePts.Count != StewartPlatform.LegCount || platPts.Count != StewartPlatform.LegCount)
        {
            error =
                $"Base and Plat need exactly {StewartPlatform.LegCount} points each (got Base={basePts.Count}, Plat={platPts.Count}).";
            return false;
        }

        if (!double.IsFinite(lmin) || !double.IsFinite(lmax) || lmin <= 0 || lmax <= lmin)
        {
            error = "Lmin/Lmax must satisfy 0 < Lmin < Lmax (m).";
            return false;
        }

        var baseAnchors = new Vec3[StewartPlatform.LegCount];
        var platAnchors = new Vec3[StewartPlatform.LegCount];
        for (var i = 0; i < StewartPlatform.LegCount; i++)
        {
            if (!TryVec3(basePts[i], out baseAnchors[i]))
            {
                error = $"Base[{i}] is non-finite.";
                return false;
            }
            if (!TryVec3(platPts[i], out platAnchors[i]))
            {
                error = $"Plat[{i}] is non-finite.";
                return false;
            }
        }

        var limits = Enumerable.Range(0, StewartPlatform.LegCount)
            .Select(_ => JointLimit.Meters(lmin, lmax))
            .ToArray();
        var modelName = string.IsNullOrWhiteSpace(name) ? "stewart_custom" : name.Trim();
        stewart = new StewartRobot(new StewartPlatform(modelName, baseAnchors, platAnchors, limits));
        return true;
    }

    private static bool TryVec3(Point3d p, out Vec3 v)
    {
        if (!p.IsValid || !double.IsFinite(p.X) || !double.IsFinite(p.Y) || !double.IsFinite(p.Z))
        {
            v = default;
            return false;
        }
        v = new Vec3(p.X, p.Y, p.Z);
        return true;
    }

    public override void DrawViewportMeshes(IGH_PreviewArgs args)
    {
        if (Locked || _previewMeshes.Count == 0) return;
        for (var i = 0; i < _previewMeshes.Count; i++)
        {
            var c = i < _previewColors.Count ? _previewColors[i] : Color.White;
            if (!_matCache.TryGetValue(c, out var mat))
            {
                mat = new DisplayMaterial(c) { Transparency = 0.25 };
                _matCache[c] = mat;
            }
            args.Display.DrawMeshShaded(_previewMeshes[i], mat);
        }
    }

    public override void DrawViewportWires(IGH_PreviewArgs args)
    {
        if (Locked || _previewMeshes.Count > 0) return;
        base.DrawViewportWires(args);
    }

    public override Guid ComponentGuid => new("a9e1c3f0-7b2d-4e8a-9c1f-6d4b2a0e8f73");
}
