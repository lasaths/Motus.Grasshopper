using Motus.Core;
using Motus.Geometry;
using Motus.Presets;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using GH_IO.Serialization;
using Motus.GH.Preview;
using System.Drawing;

namespace Motus.GH.Data;

public abstract class MotusGooBase<T> : GH_Goo<T> where T : class
{
    protected MotusGooBase(T value) => Value = value;
    protected MotusGooBase() => Value = null!;
    public override bool IsValid => Value != null;
    public override string TypeName => typeof(T).Name;
    public override string TypeDescription => $"Motus {typeof(T).Name}";
    public override IGH_Goo Duplicate() => MemberwiseClone() as IGH_Goo ?? this;
}

public sealed class ToolGoo : MotusGooBase<ToolDefinition>
{
    /// <summary>
    /// Optional actuated tool mechanism (e.g. gripper fingers) authored as a <see cref="RobotDescription"/>
    /// (Motus Urdf Link/Joint/Assemble family). When set, Motus Robot grafts
    /// <see cref="KinematicTree.Attach"/> onto the arm's tree at its tip link so TreeFK/preview drive the
    /// real mechanism instead of a squashed static mesh — see <see cref="ToolDefinition.Bindings"/>.
    /// Persisted via Motus.NET <see cref="UrdfWriter"/> inline XML on GH Internalise/save.
    /// </summary>
    public RobotDescription? Mechanism { get; set; }

    public ToolGoo() { }
    public ToolGoo(ToolDefinition tool) : base(tool) { }

    public override string ToString() => Value is null ? "Tool" : $"{Value.Name} ({Value.Tcp})";

    public override bool Write(GH_IWriter writer)
    {
        if (Value is null) return true;
        writer.SetString("ToolName", Value.Name);
        WriteFrame(writer, "Tcp", Value.Tcp);
        if (Value.Capabilities is { } caps)
        {
            var width = caps.Parameters.FirstOrDefault(p =>
                p.Name.Equals("width", StringComparison.Ordinal));
            if (ReferenceEquals(caps, ToolCapabilities.Robotiq2F85))
                writer.SetString("CapSchema", "Robotiq2F85");
            else if (width is not null)
            {
                writer.SetString("CapSchema", "Custom");
                writer.SetDouble("WidthMin", width.Min);
                writer.SetDouble("WidthMax", width.Max);
                writer.SetDouble("WidthDefault", width.Default);
            }
            else
                writer.SetString("CapSchema", "None");
        }
        else
            writer.SetString("CapSchema", "None");

        if (Value.Bindings is { Count: > 0 } bindings)
        {
            writer.SetInt32("BindingCount", bindings.Count);
            for (var i = 0; i < bindings.Count; i++)
            {
                var b = bindings[i];
                writer.SetString($"Binding_{i}_Param", b.Parameter);
                writer.SetString($"Binding_{i}_Joint", b.DriverJoint);
                writer.SetDouble($"Binding_{i}_Open", b.OpenValue);
                writer.SetDouble($"Binding_{i}_Closed", b.ClosedDriverValue);
            }
        }

        if (Value.Geometry is { } geom &&
            geom.Shape == CollisionShape.Mesh &&
            geom.MeshVertices is { Count: > 0 } verts &&
            verts.Count <= 50_000)
        {
            writer.SetInt32("GeomVertCount", verts.Count);
            for (var i = 0; i < verts.Count; i++)
            {
                writer.SetDouble($"GeomV_{i}_X", verts[i][0]);
                writer.SetDouble($"GeomV_{i}_Y", verts[i][1]);
                writer.SetDouble($"GeomV_{i}_Z", verts[i][2]);
            }
            if (geom.MeshIndices is { Count: > 0 } idx)
            {
                writer.SetInt32("GeomIndexCount", idx.Count);
                for (var i = 0; i < idx.Count; i++)
                    writer.SetInt32($"GeomI_{i}", idx[i]);
            }
            WriteFrame(writer, "GeomPose", geom.Pose);
        }

        if (Mechanism is not null)
            writer.SetString("MechanismUrdf", UrdfWriter.ToXml(Mechanism, inlineMeshes: true));

        return true;
    }

    public override bool Read(GH_IReader reader)
    {
        if (!reader.ItemExists("ToolName"))
            return true;

        var name = reader.GetString("ToolName");
        var tcp = ReadFrame(reader, "Tcp");
        ToolCapabilities? caps = null;
        if (reader.ItemExists("CapSchema"))
        {
            var schema = reader.GetString("CapSchema");
            if (string.Equals(schema, "Robotiq2F85", StringComparison.OrdinalIgnoreCase))
                caps = ToolCapabilities.Robotiq2F85;
            else if (string.Equals(schema, "Custom", StringComparison.OrdinalIgnoreCase) &&
                     reader.ItemExists("WidthMin") && reader.ItemExists("WidthMax"))
            {
                var min = reader.GetDouble("WidthMin");
                var max = reader.GetDouble("WidthMax");
                var def = reader.ItemExists("WidthDefault") ? reader.GetDouble("WidthDefault") : max;
                if (max > min &&
                    !double.IsNaN(min) && !double.IsInfinity(min) &&
                    !double.IsNaN(max) && !double.IsInfinity(max))
                    caps = ToolCapabilities.WidthSchema(min, max, def);
            }
        }

        IReadOnlyList<ToolDriverBinding>? bindings = null;
        if (reader.ItemExists("BindingCount"))
        {
            var n = reader.GetInt32("BindingCount");
            if (n > 0 && n <= 32)
            {
                var list = new List<ToolDriverBinding>(n);
                for (var i = 0; i < n; i++)
                {
                    if (!reader.ItemExists($"Binding_{i}_Joint")) continue;
                    list.Add(new ToolDriverBinding(
                        reader.ItemExists($"Binding_{i}_Param")
                            ? reader.GetString($"Binding_{i}_Param")
                            : "width",
                        reader.GetString($"Binding_{i}_Joint"),
                        reader.ItemExists($"Binding_{i}_Open") ? reader.GetDouble($"Binding_{i}_Open") : 0.085,
                        reader.ItemExists($"Binding_{i}_Closed") ? reader.GetDouble($"Binding_{i}_Closed") : 0.8));
                }
                if (list.Count > 0) bindings = list;
            }
        }

        CollisionObject? geometry = null;
        if (reader.ItemExists("GeomVertCount"))
        {
            var vc = reader.GetInt32("GeomVertCount");
            if (vc > 0 && vc <= 50_000)
            {
                var verts = new List<double[]>(vc);
                var ok = true;
                for (var i = 0; i < vc; i++)
                {
                    if (!reader.ItemExists($"GeomV_{i}_X")) { ok = false; break; }
                    var x = reader.GetDouble($"GeomV_{i}_X");
                    var y = reader.GetDouble($"GeomV_{i}_Y");
                    var z = reader.GetDouble($"GeomV_{i}_Z");
                    if (double.IsNaN(x) || double.IsInfinity(x) ||
                        double.IsNaN(y) || double.IsInfinity(y) ||
                        double.IsNaN(z) || double.IsInfinity(z))
                    {
                        ok = false;
                        break;
                    }
                    verts.Add([x, y, z]);
                }
                if (ok)
                {
                    var indices = new List<int>();
                    if (reader.ItemExists("GeomIndexCount"))
                    {
                        var ic = reader.GetInt32("GeomIndexCount");
                        for (var i = 0; i < ic && i < 500_000; i++)
                        {
                            if (reader.ItemExists($"GeomI_{i}"))
                                indices.Add(reader.GetInt32($"GeomI_{i}"));
                        }
                    }
                    var pose = reader.ItemExists("GeomPose_X") ? ReadFrame(reader, "GeomPose") : Frame.Identity;
                    geometry = CollisionObject.Mesh($"{name}_geom", pose, verts, indices);
                }
            }
        }

        Value = new ToolDefinition(name, tcp, geometry, caps) { Bindings = bindings };

        if (reader.ItemExists("MechanismUrdf"))
        {
            var xml = reader.GetString("MechanismUrdf");
            if (!string.IsNullOrWhiteSpace(xml) &&
                UrdfWriter.TryParse(xml, out var mech, out _))
                Mechanism = mech;
        }

        return true;
    }

    private static void WriteFrame(GH_IWriter writer, string key, Frame f)
    {
        writer.SetDouble($"{key}_X", f.X);
        writer.SetDouble($"{key}_Y", f.Y);
        writer.SetDouble($"{key}_Z", f.Z);
        writer.SetDouble($"{key}_Qw", f.Qw);
        writer.SetDouble($"{key}_Qx", f.Qx);
        writer.SetDouble($"{key}_Qy", f.Qy);
        writer.SetDouble($"{key}_Qz", f.Qz);
    }

    private static Frame ReadFrame(GH_IReader reader, string key)
    {
        if (!reader.ItemExists($"{key}_X")) return Frame.Identity;
        return new Frame(
            reader.GetDouble($"{key}_X"),
            reader.GetDouble($"{key}_Y"),
            reader.GetDouble($"{key}_Z"),
            reader.ItemExists($"{key}_Qw") ? reader.GetDouble($"{key}_Qw") : 1,
            reader.ItemExists($"{key}_Qx") ? reader.GetDouble($"{key}_Qx") : 0,
            reader.ItemExists($"{key}_Qy") ? reader.GetDouble($"{key}_Qy") : 0,
            reader.ItemExists($"{key}_Qz") ? reader.GetDouble($"{key}_Qz") : 0);
    }
}

public sealed class EndEffectorStateGoo : MotusGooBase<EndEffectorState>
{
    public EndEffectorStateGoo() { }
    public EndEffectorStateGoo(EndEffectorState state) : base(state) { }

    public override string ToString() => Value?.ToString() ?? "ToolState";
}

public sealed class JointStateGoo : MotusGooBase<JointState>
{
    public JointStateGoo() { }
    public JointStateGoo(JointState s) : base(s) { }
    public override string ToString() => $"Joints[{Value?.AxisCount}]";
}

/// <summary>One leg recipe (lengths / chain + IK) for Motus Mechanism.</summary>
public sealed class LegDefinitionGoo : MotusGooBase<LegDefinition>
{
    public LegDefinitionGoo() { }
    public LegDefinitionGoo(LegDefinition leg) : base(leg) { }

    public override string ToString() => Value is null
        ? "Leg"
        : Value.Lengths3R is { Count: 3 } L
            ? $"Leg '{Value.Name}' 3R [{L[0]:F3},{L[1]:F3},{L[2]:F3}] m"
            : $"Leg '{Value.Name}' {Value.DriverDof}DOF";
}

/// <summary>Body hip frames (+ optional Radial metadata) for Motus Mechanism.</summary>
public sealed class LeggedBodyGoo : MotusGooBase<List<Frame>>
{
    public double BodyZ { get; }
    public int? N { get; }
    public double? BodyR { get; }

    public IReadOnlyList<Frame> HipFrames => Value ?? [];

    public LeggedBodyGoo() : base([])
    {
        BodyZ = 0;
    }

    public LeggedBodyGoo(IReadOnlyList<Frame> hips, double bodyZ, int? n = null, double? bodyR = null)
        : base(hips as List<Frame> ?? hips.ToList())
    {
        BodyZ = bodyZ;
        N = n;
        BodyR = bodyR;
    }

    public override string ToString() =>
        $"Body N={HipFrames.Count} Bz={BodyZ:F3}" +
        (BodyR is { } r ? $" Br={r:F3}" : "");
}

/// <summary>Assembled N-leg walker + stance angles for Motus Walk.</summary>
public sealed class LeggedMechanismGoo : MotusGooBase<LeggedMechanism>
{
    public double HipStance { get; set; } = 7.5 * Math.PI / 180.0;
    public double FemurStance { get; set; } = 30.0 * Math.PI / 180.0;
    public double TibiaStance { get; set; } = -30.0 * Math.PI / 180.0;

    public LeggedMechanismGoo() { }
    public LeggedMechanismGoo(LeggedMechanism mechanism) : base(mechanism) { }

    public override string ToString() => Value is null
        ? "Mech"
        : $"Mech N={Value.LegCount} drivers={Value.DriverCount} tip={Value.TipLegName}";
}

/// <summary>Pluggable body-pose policy (<see cref="IBodyPoseSolver"/>).</summary>
public sealed class BodyPoseSolverGoo : MotusGooBase<IBodyPoseSolver>
{
    public BodyPoseSolverGoo() { }
    public BodyPoseSolverGoo(IBodyPoseSolver solver) : base(solver) { }

    public override string ToString() => Value?.MethodId ?? "Pose";
}

public sealed class TrajectoryGoo : MotusGooBase<Trajectory>
{
    public SerialJointChain? Chain { get; set; }
    public KinematicTree? Tree { get; set; }
    public StewartPlatform? Stewart { get; set; }
    public LeggedMechanism? Mechanism { get; set; }
    public double HipStanceRadians { get; set; } = LeggedGait.DefaultHipStanceRadians;
    public double FemurStanceRadians { get; set; } = LeggedGait.DefaultFemurStanceRadians;
    public double TibiaStanceRadians { get; set; } = LeggedGait.DefaultTibiaStanceRadians;
    public RobotCollisionModel? PreviewGeometry { get; set; }
    public Color?[]? PreviewMeshColors { get; set; }
    public Frame? BaseFrameOverride { get; set; }
    public MobilityModel.HolonomicSE2? MobilityGoal { get; set; }
    public ToolDefinition? ToolSnapshot { get; set; }
    public ToolCapabilities? ToolCapabilitiesSnapshot { get; set; }
    public IReadOnlyList<PlanningMessage>? DiagnosticsSnapshot { get; set; }
    public PlannerProvenance? ProvenanceSnapshot { get; set; }
    public JointState? TreeDriverHome { get; set; }
    /// <summary>Per-waypoint mobile base frames (driver-index parallel to trajectory points).</summary>
    public IReadOnlyList<Frame>? BasePath { get; set; }
    /// <summary>Optional ground height sampler (m) for Family=legged contact rings.</summary>
    public LeggedGait.TerrainHeight? TerrainSampler { get; set; }

    public TrajectoryGoo() { }
    public TrajectoryGoo(Trajectory t) : base(t) { }

    public RobotContext Context()
    {
        var model = Value!.Robot;
        var session = ApplyTool(model, ToolSnapshot, BaseFrameOverride);
        var preview = RobotPreviewGeometry.ForViewport(PreviewGeometry, ToolSnapshot);
        // Tree required so Robotiq tip-descendant meshes pose via TreeFK (not stuck at base).
        return new RobotContext(
            model, session, Chain, session.Preset.BaseFrame, session.Preset.ToolFrame,
            preview, PreviewMeshColors, Tree, Stewart, TreeDriverHome, MobilityGoal,
            Mechanism, HipStanceRadians, FemurStanceRadians, TibiaStanceRadians);
    }

    internal static RobotModel ApplyTool(RobotModel model, ToolDefinition? tool, Frame? baseOverride)
    {
        BaseFrame? baseFrame = baseOverride is { } bf ? new BaseFrame(bf) : null;
        return model.WithTool(tool, baseFrame);
    }

    public override string ToString() => $"Trajectory ({Value?.Points.Count} pts)";
}

public sealed class CollisionObjectGoo : MotusGooBase<CollisionObject>
{
    public CollisionObjectGoo() { }
    public CollisionObjectGoo(CollisionObject obj) : base(obj) { }
    public override string ToString() => Value?.Name ?? "CollisionObject";
}

public sealed class CollisionSceneGoo : MotusGooBase<CollisionScene>
{
    public CollisionSceneGoo() : base(new CollisionScene()) { }
    public CollisionSceneGoo(CollisionScene scene) : base(scene) { }
    public override string ToString() => $"CollisionScene ({Value.Objects.Count} objs)";
}

public sealed class PlanningGroupGoo : MotusGooBase<PlanningGroup>
{
    public PlanningGroupGoo() { }
    public PlanningGroupGoo(PlanningGroup group) : base(group) { }
    public override string ToString() => Value is null
        ? "PlanningGroup"
        : $"{Value.Name} ({Value.BaseLink}->{Value.TipLink})";
}

public sealed class AttachedBodyGoo : MotusGooBase<AttachedBody>
{
    public AttachedBodyGoo() { }
    public AttachedBodyGoo(AttachedBody body) : base(body) { }
    public override string ToString() => Value is null
        ? "AttachedBody"
        : $"{Value.Name} ({Value.Geometry.Shape})";
}

public sealed class MotionSegmentGoo : MotusGooBase<MotionSegment>
{
    public MotionSegmentGoo() { }
    public MotionSegmentGoo(MotionSegment segment) : base(segment) { }

    public override string ToString() => Value switch
    {
        PtpSegment ptp => $"PTP blend={ptp.BlendRadiusMeters:F3}m",
        LinSegment lin => $"LIN step={lin.StepMeters:F3}m blend={lin.BlendRadiusMeters:F3}m",
        CircSegment circ => $"CIRC samples={circ.ArcSamples} blend={circ.BlendRadiusMeters:F3}m",
        SetToolStateSegment set => $"SET dur={set.DurationSeconds:F2}s",
        WaitSegment wait => $"WAIT dur={wait.DurationSeconds:F2}s",
        _ => "Segment"
    };
}

// UrdfLinkGoo / UrdfJointGoo / RobotDescriptionGoo moved to MotusUrdfGoo.cs
// (dedicated file; RobotDescriptionGoo also gained GH document Write/Read persistence there).
