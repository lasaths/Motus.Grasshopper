using Grasshopper.Kernel;
using Motus.Core;
using Motus.Geometry;
using Motus.GH.Data;
using Motus.GH.Params;
using Rhino.Geometry;

namespace Motus.GH.Components;

// GUIDs: see Params/MotusTypedParams.cs header (0.9 N-leg Walk).

/// <summary>Leg recipe (3R analytic or numerical serial). Wire into Motus Mechanism.</summary>
public sealed class MotusLegComponent : MotusComponentBase
{
    public static readonly Guid Id = new("9a49a661-ff4c-4b96-bb57-c977ee6f9da2");

    public MotusLegComponent()
        : base(
            "Motus Leg",
            "Leg",
            "Leg lengths (m) → Leg goo. 3 lengths = LegIk3R; longer = numerical serial IK. Wire → Motus Mechanism.",
            "Model",
            "polygon")
    {
    }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddNumberParameter("Lengths", "L", "Link lengths (m). Default 3R: coxa,femur,tibia", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("Name", "N", "Optional leg name (Mechanism clones / uniquifies)", GH_ParamAccess.item, "leg");
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("Tip", "Tip", "Foot link name (default tibia / tool0)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddParameter(new Param_MotusLeg(), "Leg", "Leg", "Leg recipe → Motus Mechanism", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var lengths = new List<double>();
        var name = "leg";
        var tip = "";
        da.GetDataList(0, lengths);
        da.GetData(1, ref name);
        da.GetData(2, ref tip);

        if (lengths.Count == 0)
            lengths.AddRange([0.035, 0.08, 0.10]);

        if (lengths.Any(v => !double.IsFinite(v) || v <= 0))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Lengths must be finite and > 0 (m).");
            return;
        }

        if (string.IsNullOrWhiteSpace(name))
            name = "leg";

        try
        {
            LegDefinition def;
            if (lengths.Count == 3)
            {
                var foot = string.IsNullOrWhiteSpace(tip) ? "tibia" : tip.Trim();
                var ik = new LegIk3RSolver(lengths[0], lengths[1], lengths[2]);
                def = new LegDefinition(name.Trim(), hipInBody: null, ik, foot, lengths3R: lengths.ToArray());
            }
            else
            {
                var chain = SerialKinematicTrees.FromLengths(lengths, name: name.Trim());
                var foot = string.IsNullOrWhiteSpace(tip) ? "tool0" : tip.Trim();
                var ik = new NumericalLegIkSolver(chain, "base_link", foot);
                def = new LegDefinition(name.Trim(), hipInBody: null, ik, foot, chain: chain);
            }

            if (def.Validate() is { } err)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, err);
                return;
            }

            da.SetData(0, new LegDefinitionGoo(def));
        }
        catch (Exception ex)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
        }
    }

    public override Guid ComponentGuid => Id;
}

/// <summary>Body hips: Radial (N, BodyR, BodyZ) or Custom hip Planes.</summary>
public sealed class MotusBodyComponent : MotusComponentBase
{
    public static readonly Guid Id = new("92f0d969-c8ef-47c5-9ec7-514bebbd8441");

    public MotusBodyComponent()
        : base(
            "Motus Body",
            "Body",
            "Radial hips (N, BodyR, BodyZ) or Custom hip Planes → Bdy goo for Motus Mechanism.",
            "Model",
            "polygon")
    {
    }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddIntegerParameter("N", "N", "Radial hip count (ignored when Planes wired)", GH_ParamAccess.item, 6);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("BodyR", "Br", "Radial body radius to hip (m)", GH_ParamAccess.item, 0.06);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("BodyZ", "Bz", "Body / hip height clearance (m)", GH_ParamAccess.item, 0.07);
        p[p.ParamCount - 1].Optional = true;
        p.AddPlaneParameter("Planes", "Pl", "Optional custom hip planes (origins = hip mounts, m)", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddParameter(new Param_MotusBody(), "Body", "Bdy", "Hip frames → Motus Mechanism", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var n = 6;
        var br = 0.06;
        var bz = 0.07;
        var planes = new List<Plane>();
        da.GetData(0, ref n);
        da.GetData(1, ref br);
        da.GetData(2, ref bz);
        da.GetDataList(3, planes);

        if (!double.IsFinite(bz) || bz < 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "BodyZ must be finite and ≥ 0 (m).");
            return;
        }

        try
        {
            List<Frame> hips;
            int? metaN = null;
            double? metaR = null;

            var custom = planes.Where(pl => pl.IsValid).ToList();
            if (custom.Count >= 2)
            {
                hips = custom.Select(pl => new Frame(pl.OriginX, pl.OriginY, pl.OriginZ)).ToList();
                if (!double.IsFinite(bz) || Math.Abs(bz) < 1e-12)
                    bz = hips.Average(h => h.Z);
            }
            else
            {
                if (n < 2)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "N must be ≥ 2 (or wire ≥ 2 hip Planes).");
                    return;
                }

                if (!double.IsFinite(br) || br <= 0)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "BodyR must be finite and > 0 (m).");
                    return;
                }

                hips = new List<Frame>(n);
                for (var i = 0; i < n; i++)
                {
                    var yaw = i * (2.0 * Math.PI / n);
                    hips.Add(new Frame(br * Math.Cos(yaw), br * Math.Sin(yaw), bz));
                }

                metaN = n;
                metaR = br;
            }

            da.SetData(0, new LeggedBodyGoo(hips, bz, metaN, metaR));
        }
        catch (Exception ex)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
        }
    }

    public override Guid ComponentGuid => Id;
}

/// <summary>Assemble Body + Leg(s) → LeggedMechanism (auto gait).</summary>
public sealed class MotusMechanismComponent : MotusComponentBase
{
    public static readonly Guid Id = new("aa18b783-9a1c-44f8-bd2b-e508c3d372ac");

    private static readonly string[] HexNames =
    [
        "right-middle", "right-front", "left-front",
        "left-middle", "left-back", "right-back",
    ];

    public MotusMechanismComponent()
        : base(
            "Motus Mechanism",
            "Mech",
            "Assemble Bdy + Leg (clone to all hips) or Leg list → Mech for Motus Walk. Auto gait via GaitSchedule.Auto.",
            "Model",
            "polygon")
    {
    }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddParameter(new Param_MotusBody(), "Body", "Bdy", "Hip frames from Motus Body", GH_ParamAccess.item);
        p.AddParameter(new Param_MotusLeg(), "Leg", "Leg", "One Leg (cloned) or list matching hip count", GH_ParamAccess.list);
        p.AddBooleanParameter("AllowDynamicGait", "Dyn", "Allow N≤3 / MinStance&lt;3 gaits", GH_ParamAccess.item, false);
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("Tip", "Tip", "Tip leg name (default = first leg)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("HipStance", "Hs", "Coxa stance (rad, signed by leg side)", GH_ParamAccess.item, 7.5 * Math.PI / 180.0);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("FemurStance", "Fs", "Fallback femur (rad)", GH_ParamAccess.item, 30.0 * Math.PI / 180.0);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("TibiaStance", "Ts", "Fallback tibia (rad)", GH_ParamAccess.item, -30.0 * Math.PI / 180.0);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddParameter(new Param_MotusMechanism(), "Mechanism", "Mech", "Assembled walker → Motus Walk", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        LeggedBodyGoo? body = null;
        var legGoos = new List<LegDefinitionGoo>();
        var allowDyn = false;
        var tip = "";
        var hs = 7.5 * Math.PI / 180.0;
        var fs = 30.0 * Math.PI / 180.0;
        var ts = -30.0 * Math.PI / 180.0;
        da.GetData(0, ref body);
        da.GetDataList(1, legGoos);
        da.GetData(2, ref allowDyn);
        da.GetData(3, ref tip);
        da.GetData(4, ref hs);
        da.GetData(5, ref fs);
        da.GetData(6, ref ts);

        if (body is null || body.HipFrames.Count < 2)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Body needs ≥ 2 hip frames (Motus Body).");
            return;
        }

        var recipes = legGoos.Where(g => g?.Value is not null).Select(g => g.Value!).ToList();
        if (recipes.Count == 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Wire ≥ 1 Motus Leg.");
            return;
        }

        if (!double.IsFinite(hs) || !double.IsFinite(fs) || !double.IsFinite(ts))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Stance angles must be finite (rad).");
            return;
        }

        try
        {
            var hips = body.HipFrames;
            var n = hips.Count;
            var legs = new LegDefinition[n];

            if (recipes.Count == 1)
            {
                var src = recipes[0];
                for (var i = 0; i < n; i++)
                {
                    // Hex: classic insectoid names. Else uniquify base name across hips.
                    var name = n == 6 ? HexNames[i]
                        : n == 1 ? src.Name
                        : $"{src.Name}{i}";
                    // Clone NumericalLegIkSolver — it holds per-run seed state (unsafe to fan out).
                    var ik = src.Ik is NumericalLegIkSolver && src.Chain is not null
                        ? new NumericalLegIkSolver(src.Chain, "base_link", src.FootLink)
                        : src.Ik is LegIk3RSolver s3
                            ? new LegIk3RSolver(s3.Coxa, s3.Femur, s3.Tibia)
                            : src.Ik;
                    legs[i] = new LegDefinition(
                        name, hips[i], ik, src.FootLink, src.Lengths3R, src.Chain);
                }
            }
            else if (recipes.Count == n)
            {
                for (var i = 0; i < n; i++)
                {
                    var src = recipes[i];
                    legs[i] = new LegDefinition(
                        src.Name, hips[i], src.Ik, src.FootLink, src.Lengths3R, src.Chain);
                }
            }
            else
            {
                AddRuntimeMessage(
                    GH_RuntimeMessageLevel.Error,
                    $"Leg list count ({recipes.Count}) must be 1 (clone) or match hips ({n}).");
                return;
            }

            var tipName = string.IsNullOrWhiteSpace(tip) ? legs[0].Name : tip.Trim();
            var yaws = legs.Select((_, i) => Math.Atan2(hips[i].Y, hips[i].X)).ToArray();
            var gait = GaitSchedule.Auto(n, yaws);
            var mech = new LeggedMechanism(
                legs, gait, tipName, nominalBodyClearance: body.BodyZ, allowDynamicGait: allowDyn);

            if (mech.ValidateAndCalibrate(hs, fs, ts) is { } calErr)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, calErr);
                return;
            }

            da.SetData(0, new LeggedMechanismGoo(mech)
            {
                HipStance = hs,
                FemurStance = fs,
                TibiaStance = ts,
            });
        }
        catch (Exception ex)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
        }
    }

    public override Guid ComponentGuid => Id;
}

/// <summary>Advanced body-pose policy (PathFollow | TerrainSupport).</summary>
public sealed class MotusBodyPoseComponent : MotusComponentBase
{
    public static readonly Guid Id = new("76051f49-2641-4530-8b79-c5635a8e6eaf");

    public MotusBodyPoseComponent()
        : base(
            "Motus Body Pose",
            "BodyPose",
            "PathFollow or TerrainSupport body-pose policy → Pose goo for Motus Walk (optional).",
            "Model",
            "path")
    {
    }

    public override GH_Exposure Exposure => GH_Exposure.secondary;

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddTextParameter("Mode", "Mode", "PathFollow | TerrainSupport", GH_ParamAccess.item, "PathFollow");
        p.AddNumberParameter("Clearance", "Clr", "Body clearance above support (m); 0 = origin on plane", GH_ParamAccess.item, 0);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddParameter(new Param_MotusBodyPose(), "Pose", "Pose", "Body-pose solver → Motus Walk", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var mode = "PathFollow";
        var clr = 0.0;
        da.GetData(0, ref mode);
        da.GetData(1, ref clr);

        if (!double.IsFinite(clr))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Clearance must be finite (m).");
            return;
        }

        try
        {
            IBodyPoseSolver solver = (mode ?? "").Trim().ToLowerInvariant() switch
            {
                "terrainsupport" or "terrain" or "support" => new TerrainSupportBodyPose(clr),
                "pathfollow" or "path" or "" => new PathFollowBodyPose(clr),
                _ => throw new ArgumentException($"Mode must be PathFollow or TerrainSupport, got '{mode}'."),
            };
            da.SetData(0, new BodyPoseSolverGoo(solver));
        }
        catch (Exception ex)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
        }
    }

    public override Guid ComponentGuid => Id;
}
