using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Motus.Core;
using Motus.Geometry;
using Motus.GH.Data;
using Motus.GH.Params;
using Motus.GH.Planning;
using Motus.GH.Preview;
using Rhino.Display;
using Rhino.Geometry;
using System.Drawing;

namespace Motus.GH.Components;

/// <summary>
/// Foot-target gait for a walking hex (Family=legged). Optional Motus Hex size; Path/Planes + Terrain → Tr.
/// </summary>
public sealed class MotusWalkingHexapodComponent : RobotSourceComponentBase
{
    public const string LeggedFamily = Units.LeggedFamily;

    public static readonly Guid Id = new("236f9a53-c07b-4663-bf27-950e20fb59ab");

    public static readonly string[] LegNames =
    [
        "right-middle", "right-front", "left-front",
        "left-middle", "left-back", "right-back",
    ];

    private List<Color> _previewColors = [];
    private List<Circle> _previewContactCircles = [];
    private readonly Dictionary<Color, DisplayMaterial> _matCache = new();

    public MotusWalkingHexapodComponent()
        : base(
            "Motus Walk Hex",
            "WalkHex",
            "Walk a hex along Path/Planes (optional Terrain). Optional Hx from Motus Hex; omit = compact defaults. Family=legged — not Stewart.",
            "polygon")
    {
    }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddParameter(new Param_MotusHex(), "Hex", "Hx", "Optional size & stance from Motus Hex (omit = compact defaults)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddCurveParameter("Path", "P", "Walk path (Curve or Planes list, m)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddPlaneParameter("Planes", "Pl", "Optional path as plane list (origins, m)", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Speed", "Spd", "Walk speed along path (m/s)", GH_ParamAccess.item, 0.06);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Step", "St", "Nominal step length along path (m)", GH_ParamAccess.item, 0.04);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Lift", "Lf", "Swing foot lift (m) — keep ≥ terrain Amp for hills", GH_ParamAccess.item, 0.02);
        p[p.ParamCount - 1].Optional = true;
        p.AddGeometryParameter("Terrain", "Tn", "Optional ground Mesh/Brep/Surface/Extrusion/SubD/Box (m); omit = flat Z=0", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddGenericParameter("Robot", "Rb", "Gait robot (18-DOF) or tip-path robot when no Path", GH_ParamAccess.item);
        p.AddParameter(new Param_MotusJointState(), "State", "Js", "Full 18-DOF stance", GH_ParamAccess.item);
        p.AddParameter(new Param_MotusTrajectory(), "Trajectory", "Tr", "Gait trajectory when Path/Planes wired", GH_ParamAccess.item);
        p.AddCurveParameter("PathCurve", "Pc", "Resolved walk path curve (m)", GH_ParamAccess.item);
        p.AddPlaneParameter("PathPlanes", "Pp", "Body planes sampled along path", GH_ParamAccess.list);
        p.AddMeshParameter("Meshes", "M", "Preview meshes", GH_ParamAccess.list);
        p.AddCurveParameter("Support", "Sp", "Support polygon (foot tips)", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        HexLayoutGoo? hexGoo = null;
        Curve? pathCurve = null;
        var pathPlanes = new List<Plane>();
        var speed = 0.06;
        var stepLen = 0.04;
        var lift = 0.02;
        var terrainGoos = new List<IGH_GeometricGoo>();
        da.GetData(0, ref hexGoo);
        da.GetData(1, ref pathCurve);
        da.GetDataList(2, pathPlanes);
        da.GetData(3, ref speed);
        da.GetData(4, ref stepLen);
        da.GetData(5, ref lift);
        da.GetDataList(6, terrainGoos);

        var layout = hexGoo?.Value ?? LeggedLayout.HexMithi(0.06, 0.035, 0.08, 0.10, 0.07);
        var hs = hexGoo?.HipStance ?? 7.5 * Math.PI / 180.0;
        var fs = hexGoo?.FemurStance ?? 30.0 * Math.PI / 180.0;
        var ts = hexGoo?.TibiaStance ?? -30.0 * Math.PI / 180.0;
        var qIn = hexGoo?.DriverQ is { Count: >= 18 } dq ? dq.ToList() : new List<double>();

        var terrainGeom = new List<object>(terrainGoos.Count);
        TerrainHeightRhino.CollectFromGoos(terrainGoos, terrainGeom);

        var hasPath = (pathCurve is not null && pathCurve.IsValid) || pathPlanes.Count >= 2;

        if (!double.IsFinite(speed) || !double.IsFinite(stepLen) || !double.IsFinite(lift)
            || !double.IsFinite(hs) || !double.IsFinite(fs) || !double.IsFinite(ts)
            || !double.IsFinite(layout.BodyR) || !double.IsFinite(layout.Coxa)
            || !double.IsFinite(layout.Femur) || !double.IsFinite(layout.Tibia)
            || !double.IsFinite(layout.BodyZ))
        {
            ClearPreview();
            _previewColors = [];
            _previewContactCircles = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Speed / Step / Lift / Hex size must be finite.");
            return;
        }

        if (layout.BodyR <= 0 || layout.Coxa <= 0 || layout.Femur <= 0 || layout.Tibia <= 0)
        {
            ClearPreview();
            _previewColors = [];
            _previewContactCircles = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Hex BodyR / Coxa / Femur / Tibia must be > 0 (m).");
            return;
        }

        try
        {
            var q = WalkingHexShared.BuildStanceQ(layout, hs, fs, ts, qIn);
            var desc = WalkingHexShared.BuildDescription(layout);
            var tree = desc.ToKinematicTree();
            var tipLink = layout.TipLinkName;
            var tip = tree.ExtractSerialTip("body", tipLink);
            var driverNames = WalkingHexShared.DriverNames(tree);
            var allLimits = WalkingHexShared.LimitsAllDrivers(tree);

            RobotPreset preset;
            SerialJointChain? chain;
            JointState previewHome;
            JointState? treeDriverHome;
            if (hasPath)
            {
                preset = new RobotPreset
                {
                    Manufacturer = RobotManufacturer.Unknown,
                    ModelName = "walking_hexapod_gait",
                    Family = Units.LeggedFamily,
                    AxisCount = tree.DriverCount,
                    JointLimits = allLimits,
                    BaseFrame = BaseFrame.Identity,
                    ToolFrame = ToolFrame.Identity,
                    Notes = "Walking hex gait (18-DOF, Family=legged). Trajectory → Preview — not UR MoveJ.",
                    SourceNote = "Motus Walk Hex",
                };
                chain = null;
                previewHome = new JointState(q);
                treeDriverHome = null;
            }
            else
            {
                var tipLimits = WalkingHexShared.LimitsAlongTip(tree, tip.JointNames);
                preset = new RobotPreset
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
                    Notes = "Walking hex tip-path (one leg). Wire Path for gait, or use Motus Hex → Plan.",
                    SourceNote = "Motus Walk Hex",
                };
                chain = tip.Chain;
                var tipHome = new double[preset.AxisCount];
                for (var i = 0; i < tipHome.Length && i < 3; i++)
                    tipHome[i] = q[i];
                previewHome = new JointState(tipHome);
                treeDriverHome = new JointState(q);
            }

            var model = new RobotModel(preset, jointNames: driverNames);
            var goo = new RobotModelGoo(model)
            {
                Chain = chain,
                Tree = tree,
                PreviewHome = previewHome,
                TreeDriverHome = treeDriverHome,
                PreviewGeometry = MechanismPreviewGeometry.Build(desc),
            };

            TrajectoryGoo? trajGoo = null;
            Curve? pathCurveOut = null;
            List<Plane> pathPlanesOut = [];
            LeggedGait.TerrainHeight? terrain = null;
            if (hasPath)
            {
                terrain = TerrainHeightRhino.TryCreate(terrainGeom, out var terrainWarn);
                if (terrainWarn is not null)
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, terrainWarn);
                else if (terrain is not null)
                {
                    var probeX = pathPlanes.Count > 0 ? pathPlanes[0].OriginX
                        : pathCurve!.PointAtStart.X;
                    var probeY = pathPlanes.Count > 0 ? pathPlanes[0].OriginY
                        : pathCurve!.PointAtStart.Y;
                    var z0 = terrain(probeX, probeY);
                    if (double.IsFinite(z0))
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                            $"Terrain active — path projected onto support plane (probe Z={z0:F3} m).");
                    else
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                            $"Terrain miss at path start ({probeX:F3},{probeY:F3}) — enlarge Ground or move path.");

                    // Swing must clear local height delta or feet clip / IK struggles mid-swing.
                    double zMin = double.PositiveInfinity, zMax = double.NegativeInfinity;
                    void Acc(double x, double y)
                    {
                        var z = terrain(x, y);
                        if (!double.IsFinite(z)) return;
                        if (z < zMin) zMin = z;
                        if (z > zMax) zMax = z;
                    }
                    if (pathPlanes.Count >= 2)
                    {
                        foreach (var pl in pathPlanes)
                            Acc(pl.OriginX, pl.OriginY);
                    }
                    else if (pathCurve is not null)
                    {
                        for (var i = 0; i <= 8; i++)
                        {
                            var t = pathCurve.Domain.ParameterAt(i / 8.0);
                            var p = pathCurve.PointAt(t);
                            Acc(p.X, p.Y);
                        }
                    }
                    if (double.IsFinite(zMin) && double.IsFinite(zMax))
                    {
                        var span = zMax - zMin;
                        var need = Math.Max(0.02, 0.55 * span);
                        if (lift < need)
                        {
                            lift = need;
                            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                                $"Lift raised to {lift:F3} m for terrain span {span:F3} m.");
                        }
                    }
                }

                if (!LeggedGaitRhino.TryBuild(
                        layout, pathCurve, pathPlanes, speed, stepLen, lift,
                        hs, fs, ts, model,
                        out var gait, out var gaitErr, terrain))
                {
                    ClearPreview();
                    _previewColors = [];
                    _previewContactCircles = [];
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, gaitErr);
                    return;
                }

                var validation = LeggedGait.ValidateForPlan(gait!.GaitResult);
                if (!validation.Success)
                {
                    ClearPreview();
                    _previewColors = [];
                    _previewContactCircles = [];
                    AddRuntimeMessage(
                        GH_RuntimeMessageLevel.Error,
                        validation.Errors.Count > 0
                            ? string.Join("; ", validation.Errors)
                            : "LeggedGait.ValidateForPlan failed.");
                    return;
                }
                foreach (var warning in validation.Warnings)
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, warning);

                trajGoo = new TrajectoryGoo(gait.Trajectory)
                {
                    Tree = tree,
                    PreviewGeometry = goo.PreviewGeometry,
                    PreviewMeshColors = goo.PreviewMeshColors,
                    TreeDriverHome = new JointState(q),
                    BasePath = gait.BasePath,
                    TerrainSampler = terrain ?? FlatTerrain,
                };
                pathCurveOut = gait.PathCurve;
                pathPlanesOut = gait.PathPlanes.ToList();
                if (gait.Warning is { } warn)
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, warn);
                if (double.IsFinite(gait.MinStaticStabilityMarginMeters))
                {
                    AddRuntimeMessage(
                        GH_RuntimeMessageLevel.Remark,
                        $"McGhee–Frank SSM min={gait.MinStaticStabilityMarginMeters:F4} m (DOI {LeggedMethodRefs.McGheeFrank1968Doi}; CoM≈body XY heuristic).");
                }
            }
            else
            {
                AddRuntimeMessage(
                    GH_RuntimeMessageLevel.Remark,
                    "Wire Path or Planes (≥2) for gait Tr → Preview. Or Motus Hex → Plan for tip-path only.");
            }

            var previewQ = q;
            Frame? previewBase = null;
            LeggedGait.TerrainHeight? previewTerrain = null;
            if (hasPath && trajGoo?.BasePath is { Count: > 0 } bp
                && trajGoo.Value is { Points.Count: > 0 } traj)
            {
                var mid = traj.Points.Count / 2;
                previewQ = traj.Points[mid].JointState.Positions.ToArray();
                previewBase = bp[Math.Min(mid, bp.Count - 1)];
                previewTerrain = trajGoo.TerrainSampler;
            }

            var preview = WalkingHexPreview.Build(layout, previewQ, previewBase, previewTerrain);
            _previewMeshes = preview.Meshes.ToList();
            _previewWires = preview.Wires.ToList();
            _previewColors = preview.Colors.ToList();
            _previewContactCircles = preview.ContactCircles.ToList();
            ExpirePreview(true);

            da.SetData(0, goo);
            da.SetData(1, new JointStateGoo(new JointState(q)));
            da.SetData(2, trajGoo);
            da.SetData(3, pathCurveOut);
            da.SetDataList(4, pathPlanesOut);
            da.SetDataList(5, _previewMeshes);
            da.SetData(6, preview.SupportPolygon);
        }
        catch (Exception ex)
        {
            ClearPreview();
            _previewColors = [];
            _previewContactCircles = [];
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
        if (Locked) return;
        var contactColor = Color.FromArgb(220, 80, 200, 120);
        foreach (var c in _previewContactCircles)
            args.Display.DrawCircle(c, contactColor, 2);
        if (_previewWires.Count > 0)
        {
            foreach (var line in _previewWires)
                args.Display.DrawLine(line, Color.FromArgb(200, 255, 140, 40), 2);
            return;
        }
        base.DrawViewportWires(args);
    }

    public override Guid ComponentGuid => Id;

    private static double FlatTerrain(double x, double y) => 0;
}
