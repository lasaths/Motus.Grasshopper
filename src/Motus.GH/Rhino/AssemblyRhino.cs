#if MOTUS_CONSTRUCTION
using System.Globalization;
using System.Text;
using Motus.Construction;
using Motus.Core;
using Motus.Geometry;
using Rhino.Geometry;

namespace Motus.GH.Rhino;

/// <summary>Structure to assemble, from Rhino lines (metres, Z-up).</summary>
public sealed record AssemblyStructure(
    BambooTrussLayout.StructureDefinition Definition,
    IReadOnlyList<Point3d> Nodes,
    IReadOnlyList<Line> Members);

/// <summary>Site geometry: supports (structure nodes), obstacles, optional ground pads and stock (metres, Z-up).</summary>
public sealed record AssemblySite(
    IReadOnlyList<Point3d> Supports,
    IReadOnlyList<CollisionObject> Obstacles,
    IReadOnlyList<BoundingBox> GroundPads,
    Point3d? Stock,
    double? ClampHeight,
    IReadOnlyList<double>? HeadingsDeg);

/// <summary>One preview sample: robot parts and canes (Motus world == Rhino world, 4×4 row-major).</summary>
public sealed record AssemblyFrame(
    double T,
    string Label,
    bool Holding,
    IReadOnlyList<(CollisionObject Part, double[] World)> Robot,
    IReadOnlyList<(string Name, double[] World, double Length)> Canes);

/// <summary>A planned assembly with its preview samples.</summary>
public sealed record AssemblyRun(AssemblyPlanRequest Request, AssemblyPlan Plan, IReadOnlyList<AssemblyFrame> Frames, IReadOnlyList<string> Warnings)
{
    public int MemberCount => Plan.Members.Count;
    public int BuiltCount => Plan.Build.Built.Distinct(StringComparer.Ordinal).Count();
}

/// <summary>
/// Thin Rhino ↔ Motus.Construction glue for the Assembly components: coordinate conversion, request building and
/// preview sampling. All planning is <see cref="AssemblyPlanner.Plan"/> (Motus.NET); nothing here is structure-specific.
/// Frames: Rhino / Motus world is Z-up; the Motus.Construction layout is viewer Y-up, viewer (x, y, z) = Rhino (x, z, −y).
/// </summary>
public static class AssemblyRhino
{
    /// <summary>Node snap tolerance (m): line ends / supports within this distance share a node.</summary>
    public const double NodeTolerance = 0.005;

    /// <summary>Serialises planning and preview sampling (Motus.Construction keeps process-wide layout state).</summary>
    public static readonly object Sync = new();

    public static (double X, double Y, double Z) ToViewer(Point3d p) => (p.X, p.Z, -p.Y);

    /// <summary>Build a structure from member lines, optional explicit nodes / names and optional cane lengths (m).</summary>
    public static AssemblyStructure Structure(IReadOnlyList<Line> members, IReadOnlyList<Point3d>? nodes = null,
        IReadOnlyList<string>? names = null, IReadOnlyList<double>? caneLengths = null)
    {
        if (members.Count == 0) throw new ArgumentException("No members.");
        foreach (var m in members)
        {
            if (!m.IsValid || m.Length < 2 * NodeTolerance) throw new ArgumentException("Member lines must be valid and longer than the node tolerance.");
            if (!Finite(m.From) || !Finite(m.To)) throw new ArgumentException("Member lines must have finite coordinates.");
        }
        var pts = new List<Point3d>();
        if (nodes is { Count: > 0 })
        {
            foreach (var p in nodes)
            {
                if (!Finite(p)) throw new ArgumentException("Node points must be finite.");
                if (pts.Any(q => q.DistanceTo(p) < NodeTolerance)) throw new ArgumentException("Two node points coincide.");
                pts.Add(p);
            }
        }
        else
        {
            foreach (var m in members)
                foreach (var p in new[] { m.From, m.To })
                    if (!pts.Any(q => q.DistanceTo(p) < NodeTolerance)) pts.Add(p);
        }
        if (names is { Count: > 0 } && names.Count != pts.Count)
            throw new ArgumentException($"Names: {names.Count} given for {pts.Count} nodes.");
        var nodeNames = pts.Select((_, i) => names is { Count: > 0 } ? names[i].Trim() : "N" + i.ToString(CultureInfo.InvariantCulture)).ToList();
        if (nodeNames.Any(string.IsNullOrEmpty) || nodeNames.Distinct(StringComparer.Ordinal).Count() != nodeNames.Count)
            throw new ArgumentException("Node names must be non-empty and unique.");
        if (caneLengths is { Count: > 0 } && caneLengths.Count != members.Count)
            throw new ArgumentException($"Lengths: {caneLengths.Count} given for {members.Count} members.");

        int NodeOf(Point3d p)
        {
            var best = -1; var d = double.MaxValue;
            for (var i = 0; i < pts.Count; i++) { var di = pts[i].DistanceTo(p); if (di < d) { d = di; best = i; } }
            if (d > NodeTolerance) throw new ArgumentException($"Member end {Fmt(p)} is not on a node (tolerance {NodeTolerance} m).");
            return best;
        }

        var structNodes = pts.Select((p, i) => { var v = ToViewer(p); return new BambooTrussLayout.StructureNode(nodeNames[i], v.X, v.Y, v.Z); }).ToList();
        var ids = new HashSet<string>(StringComparer.Ordinal);
        var structMembers = new List<BambooTrussLayout.StructureMember>();
        for (var k = 0; k < members.Count; k++)
        {
            int a = NodeOf(members[k].From), b = NodeOf(members[k].To);
            if (a == b) throw new ArgumentException($"Member {k} starts and ends on node {nodeNames[a]}.");
            var id = nodeNames[a] + nodeNames[b];
            for (var n = 2; !ids.Add(id); n++) id = nodeNames[a] + nodeNames[b] + "-" + n.ToString(CultureInfo.InvariantCulture);
            double? len = null;
            if (caneLengths is { Count: > 0 })
            {
                var l = caneLengths[k];
                if (!double.IsFinite(l) || l <= 0) throw new ArgumentException($"Length of member {id} must be positive.");
                len = l;
            }
            structMembers.Add(new BambooTrussLayout.StructureMember(id, $"{nodeNames[a]}–{nodeNames[b]}", Glyph(structNodes[a], structNodes[b]), nodeNames[a], nodeNames[b], len));
        }
        return new AssemblyStructure(new BambooTrussLayout.StructureDefinition(structNodes, structMembers), pts, members);
    }

    static string Glyph(BambooTrussLayout.StructureNode a, BambooTrussLayout.StructureNode b)
    {
        double dx = b.X - a.X, dy = b.Y - a.Y;
        if (Math.Abs(dy) < NodeTolerance) return "_";
        if (Math.Abs(dx) < NodeTolerance) return "|";
        return dx * dy > 0 ? "/" : "\\";
    }

    /// <summary>
    /// The Motus.Construction request for a structure on a site. Unset site values follow
    /// <see cref="SiteDefaults.Chosen"/> (gap, pit depth, clamp height, heading, stock offset from the start support);
    /// the uniform cane length is the median node-to-node distance minus two node gaps.
    /// </summary>
    public static AssemblyPlanRequest Request(AssemblyStructure structure, AssemblySite site, string urdfXml,
        double clearanceMarginM, bool useShippedPlan, List<string> warnings)
    {
        if (string.IsNullOrWhiteSpace(urdfXml)) throw new ArgumentException("Robot URDF is empty.");
        if (!double.IsFinite(clearanceMarginM) || clearanceMarginM < 0 || clearanceMarginM > 0.02)
            throw new ArgumentException("Clearance margin must be in [0, 0.02] m.");
        if (site.Supports.Count == 0) throw new ArgumentException("Site needs at least one support point on a structure node.");
        var def = structure.Definition;
        var chosen = SiteDefaults.Chosen;
        var start0 = chosen.Stakes.First(s => s.Name == chosen.StartStake);

        // Supports → nodes.
        var supportNodes = new List<BambooTrussLayout.StructureNode>();
        foreach (var p in site.Supports)
        {
            if (!Finite(p)) throw new ArgumentException("Support points must be finite.");
            var i = Nearest(structure.Nodes, p, out var d);
            if (d > NodeTolerance) throw new ArgumentException($"Support {Fmt(p)} is not on a structure node (tolerance {NodeTolerance} m).");
            if (supportNodes.Contains(def.Nodes[i])) continue;
            supportNodes.Add(def.Nodes[i]);
        }
        if (site.HeadingsDeg is { Count: > 0 } h && h.Count != supportNodes.Count)
            throw new ArgumentException($"Headings: {h.Count} given for {supportNodes.Count} supports.");

        // Uniform cane length: median node distance − 2 gaps; members off it by > 1 mm get their own length.
        var gap = chosen.Gap;
        double Dist(BambooTrussLayout.StructureMember m)
        {
            var a = def.Nodes.First(n => n.Name == m.NodeA); var b = def.Nodes.First(n => n.Name == m.NodeB);
            return Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y) + (a.Z - b.Z) * (a.Z - b.Z));
        }
        var spans = def.Members.Select(Dist).OrderBy(x => x).ToList();
        var panel = spans[spans.Count / 2];
        var cane = panel - 2 * gap;
        if (cane <= 2 * BambooIcdCell.StrutRadius) throw new ArgumentException($"Members too short for the {gap * 1000:F0} mm node gap.");
        var members = def.Members.Select(m => m.Length is null && Math.Abs(Dist(m) - panel) > 0.001 ? m with { Length = Dist(m) - 2 * gap } : m).ToList();
        foreach (var m in members.Where(m => m.Length is not null && Math.Abs(m.Length.Value - cane) > 0.001))
            warnings.Add($"{m.Id}: cane {m.Length:F3} m (uniform {cane:F3} m).");
        def = def with { Members = members };

        var cx = def.Nodes.Average(n => n.X);
        var clampY = site.ClampHeight ?? start0.ClampY;
        if (!double.IsFinite(clampY) || clampY <= 0) throw new ArgumentException("Clamp height must be positive.");
        var stakes = supportNodes.Select((n, i) =>
        {
            var heading = site.HeadingsDeg is { Count: > 0 } hd ? hd[i] : n.X <= cx ? start0.HeadingDeg : 180 - start0.HeadingDeg;
            return SiteDefaults.Anchor("anchor-" + n.Name, n.X, n.Y, clampY, heading) with { Z = n.Z };
        }).ToList();
        var startNode = supportNodes[0];
        var startStake = stakes[0];
        double feedX, feedZ;
        if (site.Stock is { } stock)
        {
            if (!Finite(stock)) throw new ArgumentException("Stock point must be finite.");
            var v = ToViewer(stock); feedX = v.X; feedZ = v.Z;
        }
        else { feedX = startStake.X + (chosen.FeedX - start0.X); feedZ = startStake.Z + (chosen.FeedZ - start0.Z); }
        var firstAtStart = def.Members.FirstOrDefault(m => m.NodeA == startNode.Name || m.NodeB == startNode.Name) ?? def.Members[0];

        BambooTrussLayout.GroundDefinition? ground = null;
        if (site.GroundPads.Count > 0)
            ground = new BambooTrussLayout.GroundDefinition(
                site.GroundPads.Select((b, i) => ("ground-" + i.ToString(CultureInfo.InvariantCulture), b.Center.X, -b.Center.Y, (b.Max.X - b.Min.X) / 2, (b.Max.Y - b.Min.Y) / 2)).ToList(),
                Array.Empty<(string, double, double, double, double, double, double)>());

        var resolved = chosen with
        {
            Length = cane,
            NodeY = supportNodes.Min(n => n.Y),
            Stakes = stakes,
            StartStake = startStake.Name,
            FeedX = feedX,
            FeedZ = feedZ,
            WalkRAnchor = "placed-" + firstAtStart.Id,
            WalkLTarget = startStake.Name,
            Unreachable = new Dictionary<string, string>(),
            Structure = def,
            Obstacles = site.Obstacles.Count > 0 ? site.Obstacles : null,
            Ground = ground
        };
        return new AssemblyPlanRequest
        {
            Structure = def,
            Site = resolved,
            Obstacles = resolved.Obstacles,
            Ground = ground,
            RobotUrdfXml = urdfXml,
            Robot = RobotDefinition.BambooIcd,
            Settings = MotionConstraints.Current with { ClearanceMarginM = clearanceMarginM },
            UseShippedPlan = useShippedPlan
        };
    }

    /// <summary>
    /// Opt-in planning options (all off = request unchanged): stability-checked and/or planning-layer build search
    /// (<see cref="AssemblyPlanRequest.Search"/>, searched live) and certified segments (<see cref="MotionConstraints.CertifiedSegments"/>).
    /// </summary>
    public static AssemblyPlanRequest WithOptions(AssemblyPlanRequest r, bool stability, double robotMassKg, bool certified, bool layer, string? logPath)
    {
        if (stability && (!double.IsFinite(robotMassKg) || robotMassKg < 0)) throw new ArgumentException("Robot mass must be a finite value ≥ 0 kg.");
        if (logPath is not null && !layer) throw new ArgumentException("Log: turn on the planning layer (Ly) to write the outcome log.");
        if (certified) r = r with { Settings = (r.Settings ?? MotionConstraints.Current) with { CertifiedSegments = true } };
        if (stability || layer)
            r = r with
            {
                Search = new BambooBuildSearch.Options
                {
                    Stability = stability ? new BambooFrameStability.Settings { RobotMassKg = robotMassKg } : null,
                    Tamp = layer ? new BambooBuildSearch.TampSettings { OutcomeLogPath = logPath } : null
                }
            };
        return r;
    }

    /// <summary>Stable key of a request (replan only when it changes).</summary>
    public static string Key(AssemblyPlanRequest r)
    {
        var sb = new StringBuilder();
        static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);
        foreach (var n in r.Structure!.Nodes) sb.Append(n.Name).Append(':').Append(F(n.X)).Append(',').Append(F(n.Y)).Append(',').Append(F(n.Z)).Append(';');
        foreach (var m in r.Structure.Members) sb.Append(m.Id).Append(':').Append(m.NodeA).Append('-').Append(m.NodeB).Append(':').Append(m.Length is { } l ? F(l) : "-").Append(';');
        var s = r.Site!;
        sb.Append('|').Append(F(s.Length)).Append(',').Append(F(s.NodeY)).Append(',').Append(F(s.FeedX)).Append(',').Append(F(s.FeedZ));
        foreach (var t in s.Stakes) sb.Append(t.Name).Append(':').Append(F(t.X)).Append(',').Append(F(t.Z)).Append(',').Append(F(t.ClampY)).Append(',').Append(F(t.HeadingDeg)).Append(';');
        foreach (var o in r.Obstacles ?? Array.Empty<CollisionObject>()) sb.Append(o.Name).Append('#').Append(o.ContentHash).Append(';');
        foreach (var p in r.Ground?.Pads ?? []) sb.Append(p.Name).Append(':').Append(F(p.X)).Append(',').Append(F(p.Z)).Append(',').Append(F(p.HalfX)).Append(',').Append(F(p.HalfZ)).Append(';');
        sb.Append('|').Append(r.RobotUrdfXml!.GetHashCode(StringComparison.Ordinal)).Append('|').Append(F(r.Settings!.ClearanceMarginM)).Append('|').Append(r.UseShippedPlan)
            .Append('|').Append(r.Settings.CertifiedSegments).Append('|').Append(r.Search?.Stability is { } st ? F(st.RobotMassKg) : "-")
            .Append('|').Append(r.Search?.Tamp is { } tp ? tp.OutcomeLogPath ?? "+" : "-");
        return sb.ToString();
    }

    /// <summary>Plan (Motus.NET) and sample the preview frames. Blocking: call off the UI thread.</summary>
    public static AssemblyRun Run(AssemblyPlanRequest request, int samples = 600)
    {
        lock (Sync)
        {
            var plan = AssemblyPlanner.Plan(ForPlanner(request));
            var warnings = new List<string>();
            var frames = plan.Keys.Count == 0 ? new List<AssemblyFrame>() : SampleFrames(request, plan, samples, warnings);
            return new AssemblyRun(request, plan, frames, warnings);
        }
    }

    /// <summary>
    /// The planner tags the fingerprint of any URDF-loaded robot, so an embedded plan never matches a URDF input.
    /// When the shipped plan is wanted and the URDF is the built-in bamboo_icd chain, bind its cell instead of passing XML.
    /// </summary>
    static AssemblyPlanRequest ForPlanner(AssemblyPlanRequest request)
    {
        if (!request.UseShippedPlan || request.RobotUrdfXml is not { } xml || request.Robot is { } r && !ReferenceEquals(r, RobotDefinition.BambooIcd))
            return request;
        var oldSite = BambooTrussLayout.Current;
        var oldScene = BambooTrussLayout.Scene;
        try
        {
            if (request.Scene is not null) BambooTrussLayout.UseScene(request.Scene);
            BambooTrussLayout.Use(request.Site!);
            var cell = BambooIcdCell.Load(xml, BambooTrussLayout.StrutLength, RobotDefinition.BambooIcd);
            if (!cell.Seg.SequenceEqual(BambooIcdMotion.BambooIcdSeg) || !cell.LimitsDeg.SequenceEqual(BambooIcdMotion.BambooIcdLimitsDeg))
                return request;
            BambooTrussMotion.UseCell(cell);
            return request with { RobotUrdfXml = null, Robot = null };
        }
        finally
        {
            BambooTrussLayout.UseScene(oldScene);
            BambooTrussLayout.Use(oldSite);
        }
    }

    static List<AssemblyFrame> SampleFrames(AssemblyPlanRequest request, AssemblyPlan plan, int samples, List<string> warnings)
    {
        var oldSite = BambooTrussLayout.Current;
        var oldScene = BambooTrussLayout.Scene;
        try
        {
            if (request.Scene is not null) BambooTrussLayout.UseScene(request.Scene);
            BambooTrussLayout.Use(request.Site!);
            var robot = request.Robot ?? RobotDefinition.BambooIcd;
            var cell = BambooIcdCell.Load(request.RobotUrdfXml!, BambooTrussLayout.StrutLength, robot);
            if (!cell.Seg.SequenceEqual(BambooIcdMotion.Seg))
                warnings.Add("Preview gripper frames use the built-in chain lengths; the URDF chain differs.");
            var keys = plan.Keys;
            var cycles = plan.Build.Cycles;
            var scan = new BambooTrussScan(cell, keys, cycles, plan.Members, BambooTrussLayout.JawClosedMm);
            var times = keys.Select(k => k.T).Concat(Enumerable.Range(0, samples + 1).Select(i => i / (double)samples))
                .Select(t => Math.Clamp(t, 0, 1)).Distinct().OrderBy(t => t).ToList();
            var links = cell.Model.CollisionModel?.Links ?? Array.Empty<LinkCollisionGeometry>();
            var plates = robot.R.JawPlates.ToList();
            var frames = new List<AssemblyFrame>(times.Count);
            foreach (var t in times)
            {
                var s = BambooTrussMotion.SampleState(keys, t);
                var (lw, _) = scan.GrippersAt(t, s.Q);
                cell.SetMountViewer(lw);
                var parts = new List<(CollisionObject, double[])>();
                foreach (var m in cell.MountObjects(s.GL)) parts.Add((m, Transforms.FromFrame(m.Pose)));
                var mats = cell.Fk.ComputeLinkTransforms(BambooIcdCell.RadFromDeg(s.Q));
                var baseM = Transforms.FromFrame(cell.HangBaseFrame.Frame);
                foreach (var l in links)
                {
                    if (l.LinkIndex < 0 || l.LinkIndex >= mats.Count) continue;
                    var g = l.LocalGeometry;
                    var pose = g.Pose;
                    var pi = plates.IndexOf(g.Name);
                    if (pi >= 0) pose = new Frame((pi == 0 ? -1 : 1) * BambooIcdCell.PlateShift(s.GR), pose.Y, pose.Z, pose.Qw, pose.Qx, pose.Qy, pose.Qz);
                    parts.Add((g, Transforms.Multiply(Transforms.Multiply(baseM, mats[l.LinkIndex]), Transforms.FromFrame(pose))));
                }
                var activeId = cycles.Count > 0 ? scan.World[BambooTrussMotion.CycleIndexAt(keys, cycles, t)].ActiveId : null;
                var canes = new List<(string, double[], double)>();
                foreach (var (name, v) in scan.StrutPoses(t, s.Hold, s.Q))
                {
                    var world = Transforms.Multiply(BambooIcdCell.ViewerToMotus, Transforms.FromFrame(new Frame(v[0], v[1], v[2], v[6], v[3], v[4], v[5])));
                    var len = name == BambooIcdCell.Strut ? BambooTrussLayout.LengthOf(activeId) : BambooTrussLayout.LengthOf(name);
                    canes.Add((name, world, len));
                }
                frames.Add(new AssemblyFrame(t, s.Label, s.Hold == 1, parts, canes));
            }
            return frames;
        }
        finally
        {
            BambooTrussLayout.UseScene(oldScene);
            BambooTrussLayout.Use(oldSite);
        }
    }

    /// <summary>Nearest sampled frame to <paramref name="t"/> (0..1).</summary>
    public static AssemblyFrame? FrameAt(AssemblyRun run, double t)
    {
        var f = run.Frames;
        if (f.Count == 0) return null;
        t = Math.Clamp(double.IsFinite(t) ? t : 0, 0, 1);
        int lo = 0, hi = f.Count - 1;
        while (hi - lo > 1) { var mid = (lo + hi) / 2; if (f[mid].T <= t) lo = mid; else hi = mid; }
        return Math.Abs(f[lo].T - t) <= Math.Abs(f[hi].T - t) ? f[lo] : f[hi];
    }

    public static Transform ToTransform(double[] m)
    {
        var x = Transform.Identity;
        for (var r = 0; r < 4; r++)
            for (var c = 0; c < 4; c++)
                x[r, c] = m[r * 4 + c];
        return x;
    }

    /// <summary>Mesh of a posed Motus collision part (box or mesh shapes).</summary>
    public static Mesh? PartMesh(CollisionObject part, double[] world)
    {
        Mesh? mesh = null;
        if (part.Shape == CollisionShape.Box)
            mesh = Mesh.CreateFromBox(new Box(Plane.WorldXY, new Interval(-part.ExtentX, part.ExtentX), new Interval(-part.ExtentY, part.ExtentY), new Interval(-part.ExtentZ, part.ExtentZ)), 1, 1, 1);
        else if (part.Shape == CollisionShape.Mesh && part.MeshVertices is { } v && part.MeshIndices is { } idx)
        {
            mesh = new Mesh();
            foreach (var p in v) mesh.Vertices.Add(p[0], p[1], p[2]);
            for (var i = 0; i + 2 < idx.Count; i += 3) mesh.Faces.AddFace(idx[i], idx[i + 1], idx[i + 2]);
        }
        if (mesh is null) return null;
        mesh.Transform(ToTransform(world));
        mesh.Normals.ComputeNormals();
        return mesh;
    }

    /// <summary>Cane cylinder (radius <see cref="BambooIcdCell.StrutRadius"/>) along local Z, centred on the pose.</summary>
    public static Mesh CaneMesh(double[] world, double length)
    {
        var circle = new Circle(new Plane(new Point3d(0, 0, -length / 2), Vector3d.ZAxis), BambooIcdCell.StrutRadius);
        var mesh = Mesh.CreateFromCylinder(new Cylinder(circle, length), 1, BambooIcdCell.StrutSegments, true, true);
        mesh.Transform(ToTransform(world));
        return mesh;
    }

    static int Nearest(IReadOnlyList<Point3d> pts, Point3d p, out double d)
    {
        var best = -1; d = double.MaxValue;
        for (var i = 0; i < pts.Count; i++) { var di = pts[i].DistanceTo(p); if (di < d) { d = di; best = i; } }
        return best;
    }

    static bool Finite(Point3d p) => double.IsFinite(p.X) && double.IsFinite(p.Y) && double.IsFinite(p.Z);
    static string Fmt(Point3d p) => string.Format(CultureInfo.InvariantCulture, "({0:F3}, {1:F3}, {2:F3})", p.X, p.Y, p.Z);
}
#endif
