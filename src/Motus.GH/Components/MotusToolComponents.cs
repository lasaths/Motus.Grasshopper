using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Motus.Core;
using Motus.GH;
using Motus.GH.Data;
using Motus.GH.Urdf;
using Motus.Rhino;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace Motus.GH.Components;

public sealed class MotusToolComponent : MotusComponentBase
{
    private const int MaxMeshVertices = 50_000;
    private List<Mesh> _previewMeshes = new();

    public MotusToolComponent()
        : base("Motus Tool", "Tool", "Define end-effector TCP and optional gripper geometry", "Model", "wrench") { }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddTextParameter("Name", "N", "Tool name", GH_ParamAccess.item, "tool");
        p.AddPlaneParameter("TCP", "P", "TCP in flange frame (Z = tool axis)", GH_ParamAccess.item, Plane.WorldXY);
        p.AddGeometryParameter("Geometry", "G", "Optional gripper mesh or brep (TCP-local)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddPlaneParameter("GeomPlane", "L", "Geometry pose in TCP-local frame", GH_ParamAccess.item, Plane.WorldXY);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddGenericParameter("Tool", "T", "Tool definition", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var name = "tool";
        var tcp = Plane.WorldXY;
        var geomPlane = Plane.WorldXY;
        IGH_GeometricGoo? geo = null;
        da.GetData(0, ref name);
        da.GetData(1, ref tcp);
        da.GetData(2, ref geo);
        da.GetData(3, ref geomPlane);

        if (!tcp.IsValid)
        {
            _previewMeshes = [];
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "TCP plane must be valid.");
            return;
        }

        CollisionObject? geometry = null;
        if (geo is not null)
        {
            geometry = BuildGeometry(geo, geomPlane, name, out var error);
            if (error is not null)
            {
                _previewMeshes = [];
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, error);
                return;
            }
        }

        var caps = name.Contains("robotiq", StringComparison.OrdinalIgnoreCase)
            ? ToolCapabilities.Robotiq2F85
            : null;
        var tool = new ToolDefinition(
            string.IsNullOrWhiteSpace(name) ? "tool" : name.Trim(),
            FrameConversion.FromPlane(tcp),
            geometry,
            caps);
        _previewMeshes = geometry is null
            ? []
            : CollisionViewportPreview.MeshesFor(geometry);
        da.SetData(0, new ToolGoo(tool));
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
