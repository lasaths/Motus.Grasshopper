using System.Drawing;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Parameters;
using Grasshopper.Kernel.Types;
using GH_IO.Serialization;
using Motus.Core;
using Motus.Geometry;
using Motus.GH;
using Motus.GH.Data;
using Motus.GH.Params;
using Motus.GH.Urdf;
using Motus.GH.Loaders;
using Motus.GH.Preview;
using Motus.GH.Rhino;
using Motus.GH.UI;
using Rhino.Geometry;

namespace Motus.GH.Components;

/// <summary>
/// Motus Tool — TCP + Cap schema (face dropdown) + optional G/L or Rd+Bd.
/// Cap = ToolCapabilities schema for Tool State / export (not ToolMode, not bindings).
/// Pins stay stable (no VariableParameter morph) so GHX wires survive Cap changes.
/// </summary>
public sealed class MotusToolComponent : MotusComponentBase
{
    private const int MaxMeshVertices = 50_000;
    private List<Mesh> _previewMeshes = new();

    private string _cap = ToolCapContract.None;
    private PointF? _canvasPivot;

    public MotusToolComponent()
        : base(
            "Motus Tool",
            "Tool",
            "Define end-effector TCP and optional gripper geometry or mechanism. Cap (on-component) is parameter schema for Tool State / export — not ToolMode, not bindings.",
            "Model",
            "wrench") { }

    protected override IReadOnlyList<string> AiKeywords { get; } =
    [
        "Next: Tl->Motus Robot Tool Tl",
        "Note: Cap dropdown = schema (None|Robotiq2F85); Bd maps width→driver when Rd wired",
        "Wire: optional Motus Load Mesh to Geometry G (legacy Cap+STL)",
        "Wire: optional Motus Urdf Assemble Rd to Description Rd (actuated mechanism)",
    ];

    public override void CreateAttributes()
    {
        var pivot = _canvasPivot;
        if (pivot is null && Attributes is not null)
        {
            var p = Attributes.Pivot;
            if (p.X != 0 || p.Y != 0)
                pivot = p;
        }

        m_attributes = new DropDownAttributes(this, BuildDropdownModel, OnDropdownSelect);
        if (pivot is { } keep)
            m_attributes.Pivot = keep;
    }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddTextParameter("Name", "N", "Tool name", GH_ParamAccess.item, "tool");
        p.AddPlaneParameter(
            "TCP",
            "P",
            "TCP in flange frame (Z = tool axis); unwired + Description derives from its TipTcp",
            GH_ParamAccess.item,
            Plane.WorldXY);
        p[p.ParamCount - 1].Optional = true;
        p.AddGeometryParameter(
            "Geometry",
            "G",
            "Optional static gripper mesh or brep (TCP-local); ignored when Description wired",
            GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddPlaneParameter("GeomPlane", "L", "Geometry pose in TCP-local frame", GH_ParamAccess.item, Plane.WorldXY);
        p[p.ParamCount - 1].Optional = true;
        p.AddParameter(new Param_MotusRobotDescription(), "Description", "Rd",
            "Optional actuated mechanism (RobotDescription, e.g. Motus Urdf Assemble) grafted onto Motus Robot tip",
            GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter(
            "Binding",
            "Bd",
            "Driver joint for Cap width when Rd wired (default robotiq_left_knuckle for Cap=Robotiq2F85)",
            GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddParameter(new Param_MotusTool(), "Tool", "Tl", "Tool definition", GH_ParamAccess.item);

    public override bool Write(GH_IWriter writer)
    {
        writer.SetString("ToolCapabilities", _cap);
        if (Attributes is not null)
        {
            writer.SetDouble("CanvasPivotX", Attributes.Pivot.X);
            writer.SetDouble("CanvasPivotY", Attributes.Pivot.Y);
        }

        return base.Write(writer);
    }

    public override bool Read(GH_IReader reader)
    {
        if (reader.ItemExists("ToolCapabilities"))
            _cap = ToolCapContract.Normalize(reader.GetString("ToolCapabilities"));
        if (reader.ItemExists("CanvasPivotX") && reader.ItemExists("CanvasPivotY"))
        {
            _canvasPivot = new PointF(
                (float)reader.GetDouble("CanvasPivotX"),
                (float)reader.GetDouble("CanvasPivotY"));
        }

        var ok = base.Read(reader);
        if (Attributes is not null)
        {
            var p = Attributes.Pivot;
            if (p.X != 0 || p.Y != 0)
                _canvasPivot = p;
        }

        MigrateLegacyCapPin();
        // Remove legacy Cap pin if an older document still has it (face dropdown owns schema).
        var capIdx = IndexOf("Capabilities");
        if (capIdx >= 0)
            Params.UnregisterInputParameter(Params.Input[capIdx]);

        RestoreCanvasPivot();
        return ok;
    }

    private void MigrateLegacyCapPin()
    {
        var capIdx = IndexOf("Capabilities");
        if (capIdx < 0) return;
        var param = Params.Input[capIdx];
        if (param.SourceCount == 0 && param is Param_String ps && ps.PersistentDataCount > 0)
        {
            var v = ps.PersistentData.get_FirstItem(false)?.Value;
            if (!string.IsNullOrWhiteSpace(v))
                _cap = ToolCapContract.Normalize(v);
        }
    }

    private void RestoreCanvasPivot()
    {
        if (_canvasPivot is not { } p || Attributes is null) return;
        Attributes.Pivot = p;
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var name = "tool";
        var tcp = Plane.WorldXY;
        var geomPlane = Plane.WorldXY;
        var bindingJoint = "";
        IGH_GeometricGoo? geo = null;
        RobotDescriptionGoo? descriptionGoo = null;

        da.GetData(0, ref name);
        da.GetData(1, ref tcp);
        var tcpWired = Params.Input[1].SourceCount > 0;
        da.GetData(2, ref geo);
        da.GetData(3, ref geomPlane);
        da.GetData(4, ref descriptionGoo);
        da.GetData(5, ref bindingJoint);

        var mechanism = descriptionGoo?.Value;

        if (!ToolCapContract.TryParseSchema(_cap, out var caps))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error,
                "Cap must be None or Robotiq2F85 (parameter schema for Tool State / export).");
            return;
        }

        Message = ToolCapContract.Normalize(_cap);

        Frame tcpFrame;
        if (tcpWired && tcp.IsValid)
        {
            tcpFrame = FrameConversion.FromPlane(tcp);
        }
        else if (mechanism is not null)
        {
            try
            {
                tcpFrame = mechanism.TipTcp();
            }
            catch (Exception ex)
            {
                _previewMeshes = [];
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $"Description TipTcp failed: {ex.Message}");
                return;
            }
        }
        else if (tcp.IsValid)
        {
            tcpFrame = FrameConversion.FromPlane(tcp);
        }
        else
        {
            _previewMeshes = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error,
                "TCP plane must be valid, or wire a Description to derive it from TipTcp.");
            return;
        }

        CollisionObject? geometry = null;
        if (mechanism is not null && geo is not null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                "Description (Rd) and Geometry (G) both wired — ignoring G. Use Cap+STL (G) or actuated Description (Rd), not both.");
        }
        else if (geo is not null)
        {
            geometry = BuildGeometry(geo, geomPlane, name, out var error);
            if (error is not null)
            {
                _previewMeshes = [];
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, error);
                return;
            }
        }

        if (!TryResolveBindings(mechanism, caps, bindingJoint, out var bindings, out var bindingError))
        {
            _previewMeshes = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, bindingError!);
            return;
        }

        var tool = new ToolDefinition(
            string.IsNullOrWhiteSpace(name) ? "tool" : name.Trim(),
            tcpFrame,
            geometry,
            caps)
        {
            Bindings = bindings
        };
        _previewMeshes = geometry is null
            ? []
            : CollisionViewportPreview.MeshesFor(geometry);
        if (mechanism is not null)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                "Description (Rd) is live-wire only — Internalise on Tool drops the mechanism; keep Rd→Tool→Robot Tl.");
        da.SetData(0, new ToolGoo(tool) { Mechanism = mechanism });
    }

    private DropDownAttributes.Model BuildDropdownModel() =>
        new(
            ["Cap"],
            [ToolCapContract.Schemas],
            [ToolCapContract.Normalize(_cap)]);

    private void OnDropdownSelect(int listIndex, int itemIndex)
    {
        if (listIndex != 0) return;
        if (itemIndex < 0 || itemIndex >= ToolCapContract.Schemas.Length) return;
        var next = ToolCapContract.Schemas[itemIndex];
        if (next == ToolCapContract.Normalize(_cap)) return;
        RecordUndoEvent("Tool Cap");
        _cap = next;
        ExpireSolution(true);
    }

    private int IndexOf(string name)
    {
        for (var i = 0; i < Params.Input.Count; i++)
        {
            if (string.Equals(Params.Input[i].Name, name, StringComparison.Ordinal))
                return i;
        }

        return -1;
    }

    private static bool TryResolveBindings(
        RobotDescription? mechanism,
        ToolCapabilities? caps,
        string? bindingJoint,
        out IReadOnlyList<ToolDriverBinding>? bindings,
        out string? error)
    {
        bindings = null;
        error = null;
        if (mechanism is null) return true;

        if (!string.IsNullOrWhiteSpace(bindingJoint))
        {
            var joint = bindingJoint.Trim();
            if (!MechanismHasDriver(mechanism, joint))
            {
                error = $"Binding joint '{joint}' is not an actuated (non-mimic) driver on the Description.";
                return false;
            }

            bindings =
            [
                new ToolDriverBinding(
                    Parameter: "width",
                    DriverJoint: joint,
                    OpenValue: ToolParameterBinding.Robotiq2F85OpenWidthMeters,
                    ClosedDriverValue: ToolParameterBinding.Robotiq2F85ClosedDriverRadians)
            ];
            return true;
        }

        if (!ReferenceEquals(caps, ToolCapabilities.Robotiq2F85))
            return true;

        foreach (var b in ToolCapabilities.Robotiq2F85DefaultBindings)
        {
            if (!MechanismHasDriver(mechanism, b.DriverJoint))
            {
                error =
                    $"Cap=Robotiq2F85 needs driver '{b.DriverJoint}' on Description (or set Bd to your driver joint).";
                return false;
            }
        }

        bindings = ToolCapabilities.Robotiq2F85DefaultBindings;
        return true;
    }

    private static bool MechanismHasDriver(RobotDescription mechanism, string jointName)
    {
        foreach (var j in mechanism.Joints)
        {
            if (j.IsActuated && j.MimicJoint is null &&
                string.Equals(j.Name, jointName, StringComparison.OrdinalIgnoreCase))
                return true;
        }

        return false;
    }

    private static CollisionObject? BuildGeometry(
        IGH_GeometricGoo geo,
        Plane geomPlane,
        string name,
        out string? error)
    {
        error = null;
        CollisionObject? obj = null;
        if (geo is GH_Mesh ghm && ghm.Value is { IsValid: true } mesh)
            obj = CollisionMeshBuilder.FromMesh(mesh, geomPlane, $"{name}_geom");
        else if (geo is GH_Brep ghb && ghb.Value is { IsValid: true } brep)
            obj = CollisionMeshBuilder.FromBrep(brep, geomPlane, $"{name}_geom");

        if (obj is null)
        {
            error = "Geometry must be a valid Mesh or Brep.";
            return null;
        }

        if (obj.MeshVertices is { Count: > MaxMeshVertices } vertices)
        {
            error = $"Tool mesh has {vertices.Count} vertices (max {MaxMeshVertices}).";
            return null;
        }

        return obj;
    }

    public override BoundingBox ClippingBox => CollisionViewportPreview.MeshesBoundingBox(_previewMeshes);

    public override void DrawViewportMeshes(IGH_PreviewArgs args)
    {
        if (!Locked) CollisionViewportPreview.DrawMeshes(args, _previewMeshes);
    }

    public override Guid ComponentGuid => new("b7c4e2a1-9f3d-4b6e-8c1d-2a5f9e0b3d71");
}

public sealed class MotusLoadMeshComponent : MotusComponentBase
{
    public MotusLoadMeshComponent()
        : base("Motus Load Mesh", "LoadMesh", "Load an STL mesh file (meters)", "Model", "download-simple") { }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddTextParameter("Path", "P", "Path to .stl file", GH_ParamAccess.item);
        p.AddPlaneParameter("Plane", "L", "Mesh pose (origin = local origin)", GH_ParamAccess.item, Plane.WorldXY);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddMeshParameter("Mesh", "M", "Triangle mesh", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var path = "";
        var plane = Plane.WorldXY;
        if (!da.GetData(0, ref path) || string.IsNullOrWhiteSpace(path)) return;
        da.GetData(1, ref plane);

        try
        {
            path = UrdfPathResolver.ResolveUrdfPath(path);
            if (!path.EndsWith(".stl", StringComparison.OrdinalIgnoreCase))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Only .stl files are supported.");
                return;
            }

            var source = MeshFileLoader.LoadStl(path);
            if (source is null || !source.IsValid)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $"Could not read mesh: {path}");
                return;
            }

            var xform = Transform.PlaneToPlane(Plane.WorldXY, plane);
            var mesh = source.DuplicateMesh();
            mesh.Transform(xform);
            da.SetData(0, new GH_Mesh(mesh));
        }
        catch (Exception ex)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
        }
    }

    public override Guid ComponentGuid => new("c3d4e5f6-a7b8-4901-c234-56789abcdef2");
}
