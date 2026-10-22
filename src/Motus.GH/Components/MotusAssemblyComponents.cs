#if MOTUS_CONSTRUCTION
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Motus.Construction;
using Motus.Core;
using Motus.GH.Data;
using Motus.GH.Rhino;
using Motus.GH.Urdf;
using Rhino;
using Rhino.Geometry;

namespace Motus.GH.Components;

// Assembly planning (Motus.Construction): thin wrappers — all planning is AssemblyPlanner.Plan in Motus.NET.
// Planning and preview only; no robot control.

public sealed class AssemblyStructureGoo : MotusGooBase<AssemblyStructure>
{
    public AssemblyStructureGoo() { }
    public AssemblyStructureGoo(AssemblyStructure v) : base(v) { }
    public override string ToString() => Value is null ? "Assembly Structure" : $"Structure ({Value.Definition.Nodes.Count} nodes, {Value.Definition.Members.Count} members)";
}

public sealed class AssemblySiteGoo : MotusGooBase<AssemblySite>
{
    public AssemblySiteGoo() { }
    public AssemblySiteGoo(AssemblySite v) : base(v) { }
    public override string ToString() => Value is null ? "Assembly Site" : $"Site ({Value.Supports.Count} supports, {Value.Obstacles.Count} obstacles)";
}

public sealed class AssemblyRunGoo : MotusGooBase<AssemblyRun>
{
    public AssemblyRunGoo() { }
    public AssemblyRunGoo(AssemblyRun v) : base(v) { }
    public override string ToString() => Value is null ? "Assembly Plan" : $"Assembly Plan ({Value.BuiltCount}/{Value.MemberCount} built, {Value.Plan.Verdict})";
}

/// <summary>Rhino lines (+ optional nodes, names, cane lengths) → assembly structure.</summary>
public sealed class MotusAssemblyStructureComponent : MotusComponentBase
{
    public static readonly Guid Id = new("8585f8a2-f9c2-46a2-8216-959233e54b3d");

    public MotusAssemblyStructureComponent()
        : base("Motus Assembly Structure", "AStruct",
            "Structure to assemble from member lines (m). List order = default build order; member id = start node + end node name.",
            "Assembly", "line-segments") { }

    protected override IReadOnlyList<string> AiKeywords { get; } =
        ["Wire: Rhino lines M (+ node points N, names Nm)", "Next: S -> Motus Assembly Plan S"];

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddLineParameter("Members", "M", "Member lines node to node (m)", GH_ParamAccess.list);
        p.AddPointParameter("Nodes", "N", "Optional node points (default: line end points)", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("Names", "Nm", "Optional node names, one per node (default N0, N1, …)", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Lengths", "L", "Optional cane length per member (m); default node distance − 2 node gaps", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddGenericParameter("Structure", "S", "Assembly structure → Motus Assembly Plan", GH_ParamAccess.item);
        p.AddTextParameter("Ids", "Id", "Member ids in build order", GH_ParamAccess.list);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var lines = new List<Line>(); var nodes = new List<Point3d>(); var names = new List<string>(); var lengths = new List<double>();
        if (!da.GetDataList(0, lines)) return;
        da.GetDataList(1, nodes); da.GetDataList(2, names); da.GetDataList(3, lengths);
        try
        {
            var s = AssemblyRhino.Structure(lines, nodes, names, lengths);
            da.SetData(0, new AssemblyStructureGoo(s));
            da.SetDataList(1, s.Definition.Members.Select(m => m.Id));
        }
        catch (ArgumentException ex) { AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message); }
    }

    public override Guid ComponentGuid => Id;
}

/// <summary>Supports, obstacles, optional ground and stock → assembly site.</summary>
public sealed class MotusAssemblySiteComponent : MotusComponentBase
{
    public static readonly Guid Id = new("d627c341-a801-4299-9c9e-dfe45d11a642");

    public MotusAssemblySiteComponent()
        : base("Motus Assembly Site", "ASite",
            "Assembly site (m): support points on structure nodes (first = robot start), obstacle Breps/Meshes, optional ground pads and stock point.",
            "Assembly", "bounding-box") { }

    protected override IReadOnlyList<string> AiKeywords { get; } =
        ["Wire: support points Sp on structure nodes; obstacles O", "Next: Si -> Motus Assembly Plan Si"];

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddPointParameter("Supports", "Sp", "Support points on structure nodes (clamp columns); first = start support", GH_ParamAccess.list);
        p.AddGeometryParameter("Obstacles", "O", "Optional obstacle Breps / Meshes / Boxes (m)", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
        p.AddGeometryParameter("Ground", "G", "Optional ground pads (top at Z = 0); default derived from the supports", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
        p.AddPointParameter("Stock", "St", "Optional stock point (canes picked here); default beside the start support", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Clamp", "Ch", "Optional clamp height on the support columns (m)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Heading", "Hd", "Optional clamp heading per support (deg); default mirrored toward the structure", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddGenericParameter("Site", "Si", "Assembly site → Motus Assembly Plan", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var supports = new List<Point3d>(); var obstacles = new List<IGH_GeometricGoo>(); var ground = new List<IGH_GeometricGoo>();
        var headings = new List<double>();
        Point3d stock = Point3d.Unset; double clamp = double.NaN;
        if (!da.GetDataList(0, supports)) return;
        da.GetDataList(1, obstacles); da.GetDataList(2, ground);
        var hasStock = da.GetData(3, ref stock);
        var hasClamp = da.GetData(4, ref clamp);
        da.GetDataList(5, headings);

        var objs = new List<CollisionObject>();
        for (var i = 0; i < obstacles.Count; i++)
        {
            var name = $"obstacle-{i}";
            var o = obstacles[i] switch
            {
                GH_Mesh m when m.Value is not null => CollisionMeshBuilder.FromMesh(m.Value, Plane.WorldXY, name),
                GH_Brep b when b.Value is not null => CollisionMeshBuilder.FromBrep(b.Value, Plane.WorldXY, name),
                GH_Box x when x.Value.IsValid => CollisionMeshBuilder.FromBrep(x.Value.ToBrep(), Plane.WorldXY, name),
                GH_Surface s when s.Value is not null => CollisionMeshBuilder.FromBrep(s.Value, Plane.WorldXY, name),
                _ => null
            };
            if (o is null) { AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, $"Obstacle {i}: not a Brep / Mesh / Box — skipped."); continue; }
            objs.Add(o);
        }
        var pads = ground.Where(g => g is not null).Select(g => g.Boundingbox).Where(b => b.IsValid).ToList();
        if (hasClamp && (!double.IsFinite(clamp) || clamp <= 0)) { AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Clamp height must be positive."); return; }
        da.SetData(0, new AssemblySiteGoo(new AssemblySite(supports, objs, pads, hasStock ? stock : null, hasClamp ? clamp : null, headings.Count > 0 ? headings : null)));
    }

    public override Guid ComponentGuid => Id;
}

/// <summary>Plan the assembly (Motus.NET AssemblyPlanner) off the UI thread when Run is on.</summary>
public sealed class MotusAssemblyPlanComponent : MotusComponentBase
{
    public static readonly Guid Id = new("dacb82c2-aaaa-4703-a074-ce482518c24f");

    readonly object _lock = new();
    AssemblyRun? _result;
    string? _resultKey;
    string? _error;
    string? _runningKey;

    public MotusAssemblyPlanComponent()
        : base("Motus Assembly Plan", "APlan",
            "Plan the robotic assembly of a structure on a site (Motus.Construction): pick, carry, place, walk; replay-checked for collisions. Runs in the background when Run is true. Planning / preview only.",
            "Assembly", "flow-arrow") { }

    protected override IReadOnlyList<string> AiKeywords { get; } =
        ["Wire: S from Assembly Structure, Si from Assembly Site, U = robot URDF path", "Next: P -> Motus Assembly Preview P", "Note: set Run true to plan; Ok = collision-free"];

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Structure", "S", "From Motus Assembly Structure", GH_ParamAccess.item);
        p.AddGenericParameter("Site", "Si", "From Motus Assembly Site", GH_ParamAccess.item);
        p.AddTextParameter("Robot", "U", "Robot URDF: file path (relative to the .gh / repo) or URDF XML", GH_ParamAccess.item);
        p.AddNumberParameter("Margin", "Cm", "Clearance margin (m), 0–0.02", GH_ParamAccess.item, 0.002);
        p[p.ParamCount - 1].Optional = true;
        p.AddBooleanParameter("Shipped", "Sh", "Use the embedded pre-computed plan when the input matches it (fast)", GH_ParamAccess.item, false);
        p[p.ParamCount - 1].Optional = true;
        p.AddBooleanParameter("Run", "R", "Plan (in the background) when true", GH_ParamAccess.item, false);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddGenericParameter("Plan", "P", "Assembly plan → Motus Assembly Preview", GH_ParamAccess.item);
        p.AddTextParameter("Tasks", "T", "Task sequence", GH_ParamAccess.list);
        p.AddTextParameter("Verdict", "V", "Built members and collision verdict", GH_ParamAccess.item);
        p.AddBooleanParameter("CollisionFree", "Ok", "True when every member is built and the replay scan is collision-free", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        IGH_Goo? sGoo = null, siGoo = null; string urdf = ""; double margin = 0.002; bool shipped = false, run = false;
        if (!da.GetData(0, ref sGoo) || !da.GetData(1, ref siGoo) || !da.GetData(2, ref urdf)) return;
        da.GetData(3, ref margin); da.GetData(4, ref shipped); da.GetData(5, ref run);
        if (sGoo is not AssemblyStructureGoo { Value: { } structure }) { AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "S: wire Motus Assembly Structure."); return; }
        if (siGoo is not AssemblySiteGoo { Value: { } site }) { AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Si: wire Motus Assembly Site."); return; }

        AssemblyPlanRequest req;
        var warnings = new List<string>();
        try
        {
            var xml = LoadUrdf(urdf);
            req = AssemblyRhino.Request(structure, site, xml, margin, shipped, warnings);
        }
        catch (Exception ex) when (ex is ArgumentException or IOException or UnauthorizedAccessException)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
            return;
        }
        foreach (var w in warnings) AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, w);
        var key = AssemblyRhino.Key(req);

        AssemblyRun? result; string? resultKey, error, runningKey;
        lock (_lock) { result = _result; resultKey = _resultKey; error = _error; runningKey = _runningKey; }
        if (run && runningKey is null && resultKey != key) { Start(req, key); runningKey = key; }

        if (runningKey is not null)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Planning in the background…");
        if (resultKey != key)
        {
            if (runningKey is null)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, run ? "Waiting for the planner." : "Set Run to true to plan.");
            return;
        }
        if (error is not null) { AddRuntimeMessage(GH_RuntimeMessageLevel.Error, error); return; }
        if (result is null) return;
        foreach (var w in result.Warnings) AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, w);
        var ok = result.Plan.CollisionFree && result.BuiltCount == result.MemberCount;
        if (!ok) AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, result.Plan.Verdict);
        da.SetData(0, new AssemblyRunGoo(result));
        da.SetDataList(1, result.Plan.Build.Tasks.Select(t => t.Via is { } via ? $"{t.Identity} {t.Object} via {via}" : $"{t.Identity} {t.Object}"));
        da.SetData(2, $"{result.BuiltCount}/{result.MemberCount} built, {result.Plan.Verdict}");
        da.SetData(3, ok);
    }

    void Start(AssemblyPlanRequest req, string key)
    {
        lock (_lock) _runningKey = key;
        Task.Run(() =>
        {
            AssemblyRun? r = null; string? err = null;
            try { r = AssemblyRhino.Run(req); }
            catch (Exception ex) { err = "Planner failed: " + ex.Message; }
            lock (_lock) { _result = r; _error = err; _resultKey = key; _runningKey = null; }
            RhinoApp.InvokeOnUiThread((Action)(() => { if (OnPingDocument() is not null) ExpireSolution(true); }));
        });
    }

    static string LoadUrdf(string urdf)
    {
        var t = urdf.Trim();
        if (t.StartsWith('<')) return t;
        var path = UrdfPathResolver.ResolveUrdfPath(t);
        if (!File.Exists(path)) throw new IOException($"Robot URDF not found: {t}");
        return File.ReadAllText(path);
    }

    public override Guid ComponentGuid => Id;
}

/// <summary>Robot, canes and task label of an assembly plan at a time (0..1).</summary>
public sealed class MotusAssemblyPreviewComponent : MotusComponentBase
{
    public static readonly Guid Id = new("edc2b97a-2acb-461b-95af-0530cd3c983d");

    public MotusAssemblyPreviewComponent()
        : base("Motus Assembly Preview", "APrev",
            "Assembly plan at time t (0..1): robot part meshes, placed canes, the active cane (held or on the stock) and the task label.",
            "Assembly", "eye") { }

    protected override IReadOnlyList<string> AiKeywords { get; } =
        ["Wire: P from Motus Assembly Plan; t = slider 0..1"];

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Plan", "P", "From Motus Assembly Plan", GH_ParamAccess.item);
        p.AddNumberParameter("Time", "t", "Normalised time 0..1", GH_ParamAccess.item, 0);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddMeshParameter("Robot", "R", "Robot part meshes at t", GH_ParamAccess.list);
        p.AddMeshParameter("Placed", "C", "Placed canes at t", GH_ParamAccess.list);
        p.AddMeshParameter("Active", "A", "Active cane at t (held, or on the stock)", GH_ParamAccess.list);
        p.AddTextParameter("Task", "L", "Task label at t", GH_ParamAccess.item);
        p.AddBooleanParameter("Holding", "H", "True while the robot holds a cane", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        IGH_Goo? goo = null; double t = 0;
        if (!da.GetData(0, ref goo)) return;
        da.GetData(1, ref t);
        if (goo is not AssemblyRunGoo { Value: { } run }) { AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "P: wire Motus Assembly Plan."); return; }
        if (!double.IsFinite(t)) { AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "t must be finite."); return; }
        var f = AssemblyRhino.FrameAt(run, t);
        if (f is null) { AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Plan has no motion."); return; }
        da.SetDataList(0, f.Robot.Select(p => AssemblyRhino.PartMesh(p.Part, p.World)).Where(m => m is not null));
        da.SetDataList(1, f.Canes.Where(c => c.Name != BambooIcdCell.Strut).Select(c => AssemblyRhino.CaneMesh(c.World, c.Length)));
        da.SetDataList(2, f.Canes.Where(c => c.Name == BambooIcdCell.Strut).Select(c => AssemblyRhino.CaneMesh(c.World, c.Length)));
        da.SetData(3, f.Label);
        da.SetData(4, f.Holding);
    }

    public override Guid ComponentGuid => Id;
}
#endif
