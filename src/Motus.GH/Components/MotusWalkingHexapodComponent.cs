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
/// Foot-target gait for an N-leg walker (Family=legged). Mech required; optional Pose; Path/Planes + Terrain → Tr.
/// GUID kept from Motus Walk Hex (0.8).
/// </summary>
public sealed class MotusWalkingHexapodComponent : RobotSourceComponentBase
{
    public const string LeggedFamily = Units.LeggedFamily;

    public static readonly Guid Id = new("236f9a53-c07b-4663-bf27-950e20fb59ab");

    private List<Color> _previewColors = [];
    private List<Circle> _previewContactCircles = [];
    private readonly Dictionary<Color, DisplayMaterial> _matCache = new();

    public MotusWalkingHexapodComponent()
        : base(
            "Motus Walk",
            "Walk",
            "Walk Mech along Path/Planes (optional Terrain + Pose). Family=legged — not Stewart / not UR MoveJ.",
            "polygon")
    {
    }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddParameter(new Param_MotusMechanism(), "Mechanism", "Mech", "From Motus Mechanism (Body+Leg)", GH_ParamAccess.item);
        p.AddParameter(new Param_MotusBodyPose(), "Pose", "Pose", "Optional body-pose policy (omit = Auto)", GH_ParamAccess.item);
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
        p.AddGenericParameter("Robot", "Rb", "Gait robot (full drivers) or tip-path robot when no Path", GH_ParamAccess.item);
        p.AddParameter(new Param_MotusJointState(), "State", "Js", "Full-driver stance", GH_ParamAccess.item);
        p.AddParameter(new Param_MotusTrajectory(), "Trajectory", "Tr", "Gait trajectory when Path/Planes wired", GH_ParamAccess.item);
        p.AddCurveParameter("PathCurve", "Pc", "Resolved walk path curve (m)", GH_ParamAccess.item);
        p.AddPlaneParameter("PathPlanes", "Pp", "Body planes sampled along path", GH_ParamAccess.list);
        p.AddMeshParameter("Meshes", "M", "Preview meshes", GH_ParamAccess.list);
        p.AddCurveParameter("Support", "Sp", "Support polygon (foot tips)", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        LeggedMechanismGoo? mechGoo = null;
        BodyPoseSolverGoo? poseGoo = null;
        Curve? pathCurve = null;
        var pathPlanes = new List<Plane>();
        var speed = 0.06;
        var stepLen = 0.04;
        var lift = 0.02;
        var terrainGoos = new List<IGH_GeometricGoo>();
        da.GetData(0, ref mechGoo);
        da.GetData(1, ref poseGoo);
        da.GetData(2, ref pathCurve);
        da.GetDataList(3, pathPlanes);
        da.GetData(4, ref speed);
        da.GetData(5, ref stepLen);
        da.GetData(6, ref lift);
        da.GetDataList(7, terrainGoos);

        if (mechGoo?.Value is null)
        {
            ClearPreview();
            _previewColors = [];
            _previewContactCircles = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Mech required — wire Motus Mechanism (Body + Leg).");
            return;
        }

        var mechanism = mechGoo.Value;
        var hs = mechGoo.HipStance;
        var fs = mechGoo.FemurStance;
        var ts = mechGoo.TibiaStance;

        var terrainGeom = new List<object>(terrainGoos.Count);
        TerrainHeightRhino.CollectFromGoos(terrainGoos, terrainGeom);

        var hasPath = (pathCurve is not null && pathCurve.IsValid) || pathPlanes.Count >= 2;
        var hasTerrainGeom = terrainGeom.Count > 0;

        if (!double.IsFinite(speed) || !double.IsFinite(stepLen) || !double.IsFinite(lift)
            || !double.IsFinite(hs) || !double.IsFinite(fs) || !double.IsFinite(ts))
        {
            ClearPreview();
            _previewColors = [];
            _previewContactCircles = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Speed / Step / Lift / stance must be finite.");
            return;
        }

        try
        {
            var q = WalkingHexShared.BuildStanceQ(mechanism, hs, fs, ts, null);
            var tree = mechanism.Assemble();
            var tipLink = mechanism.TipLinkName;
            var tip = tree.ExtractSerialTip(mechanism.BodyLinkName, tipLink);
            var driverNames = WalkingHexShared.DriverNames(tree);
            var allLimits = WalkingHexShared.LimitsAllDrivers(tree);
            // Preview visuals: namespaced URDF matching Assemble (3R sticks).
            RobotDescription? desc = null;
            try { desc = WalkingHexShared.BuildDescription(mechanism); }
            catch { /* numerical-only legs may lack Lengths3R visuals */ }

            RobotPreset preset;
            SerialJointChain? chain;
            JointState previewHome;
            JointState? treeDriverHome;
            if (hasPath)
            {
                preset = mechanism.ToPreset(limits: allLimits);
                preset = new RobotPreset
                {
                    Manufacturer = preset.Manufacturer,
                    ModelName = mechanism.ModelName + "_gait",
                    Family = Units.LeggedFamily,
                    AxisCount = tree.DriverCount,
                    JointLimits = allLimits,
                    BaseFrame = BaseFrame.Identity,
                    ToolFrame = ToolFrame.Identity,
                    Notes = $"Legged gait ({tree.DriverCount}-DOF, Family=legged). Trajectory → Preview — not UR MoveJ.",
                    SourceNote = "Motus Walk",
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
                    ModelName = mechanism.ModelName,
                    Family = Units.LeggedFamily,
                    AxisCount = tip.Chain.Joints.Length,
                    JointLimits = tipLimits,
                    BaseFrame = BaseFrame.Identity,
                    ToolFrame = tip.TipToolOffset is { } off
                        ? new ToolFrame(off, "foot")
                        : ToolFrame.Identity,
                    Notes = "Legged tip-path (one leg). Wire Path for gait, or Mechanism → Walk without Path.",
                    SourceNote = "Motus Walk",
                };
                chain = tip.Chain;
                var tipHome = new double[preset.AxisCount];
                var tipOff = 0;
                for (var i = 0; i < mechanism.LegCount; i++)
                {
                    if (string.Equals(mechanism.Legs[i].Name, mechanism.TipLegName, StringComparison.Ordinal))
                    {
                        tipOff = mechanism.DriverOffsets[i];
                        break;
                    }
                }
                for (var i = 0; i < tipHome.Length && tipOff + i < q.Length; i++)
                    tipHome[i] = q[tipOff + i];
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
                PreviewGeometry = desc is not null ? MechanismPreviewGeometry.Build(desc) : null,
                Mechanism = mechanism,
                HipStanceRadians = hs,
                FemurStanceRadians = fs,
                TibiaStanceRadians = ts,
            };

            TrajectoryGoo? trajGoo = null;
            Curve? pathCurveOut = null;
            List<Plane> pathPlanesOut = [];
            LeggedGait.TerrainHeight? terrain = null;
            IBodyPoseSolver? bodyPose = poseGoo?.Value;
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

                // Auto pose: TerrainSupport when Tn wired, else PathFollow (clearance = BodyZ).
                bodyPose ??= hasTerrainGeom || terrain is not null
                    ? new TerrainSupportBodyPose(clearanceMeters: 0)
                    : new PathFollowBodyPose(clearanceMeters: mechanism.NominalBodyClearance);

                if (!LeggedGaitRhino.TryBuild(
                        mechanism, bodyPose, pathCurve, pathPlanes, speed, stepLen, lift,
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
                    // Preview Walk: SSM/constraint messages stay named, but do not kill Tr —
                    // outdoor hills + odd N often dip McGhee–Frank margin slightly negative.
                    var ssmOnly = validation.Errors.Count > 0
                        && validation.Errors.All(e =>
                            e.Contains("SSM", StringComparison.OrdinalIgnoreCase));
                    if (!ssmOnly)
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

                    foreach (var e in validation.Errors)
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, StripMethodCite(e));
                }
                foreach (var warning in validation.Warnings)
                {
                    var cleaned = StripMethodCite(warning);
                    if (string.IsNullOrWhiteSpace(cleaned)) continue;
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, cleaned);
                }

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
                {
                    var cleaned = StripMethodCite(warn);
                    if (!string.IsNullOrWhiteSpace(cleaned))
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, cleaned);
                }
                if (double.IsFinite(gait.MinStaticStabilityMarginMeters))
                {
                    AddRuntimeMessage(
                        GH_RuntimeMessageLevel.Remark,
                        $"SSM min={gait.MinStaticStabilityMarginMeters:F4} m.");
                }
            }
            else
            {
                AddRuntimeMessage(
                    GH_RuntimeMessageLevel.Remark,
                    "Wire Path or Planes (≥2) for gait Tr → Preview. Or use tip-path Rb → Plan.");
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

            var preview = WalkingHexPreview.Build(mechanism, previewQ, previewBase, previewTerrain);
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

    /// <summary>DOI / method-stack blobs stay in Motus.NET MethodProvenance — not GH Status.</summary>
    private static string StripMethodCite(string message)
    {
        if (string.IsNullOrEmpty(message)) return message;
        var cleaned = System.Text.RegularExpressions.Regex.Replace(
            message,
            @"\s*\([^)]*doi:\s*[^)]+\)",
            "",
            System.Text.RegularExpressions.RegexOptions.IgnoreCase);
        cleaned = System.Text.RegularExpressions.Regex.Replace(
            cleaned,
            @"\s*;?\s*DOI\s+10\.[^\s);]+",
            "",
            System.Text.RegularExpressions.RegexOptions.IgnoreCase);
        // NuGet 0.13.0 appended DescribeStack() into Warning.
        cleaned = System.Text.RegularExpressions.Regex.Replace(
            cleaned,
            @"\s*LegIk3R=analytic[^.]*\.",
            "",
            System.Text.RegularExpressions.RegexOptions.IgnoreCase);
        return cleaned.Trim().TrimEnd(',', ';');
    }
}
