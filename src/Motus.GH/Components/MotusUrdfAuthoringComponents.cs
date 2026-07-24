using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Motus.Core;
using Motus.GH.Data;
using Motus.GH.Params;
using Motus.GH.Rhino;
using Motus.Geometry;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;

namespace Motus.GH.Components;

/// <summary>
/// Build a <see cref="UrdfLink"/> from native Rhino/GH geometry (any meshable: Box, Mesh, Brep,
/// Surface, Extrusion, SubD, …). Units are meters in the owning link frame.
/// </summary>
public sealed class MotusUrdfLinkComponent : MotusComponentBase
{
    public MotusUrdfLinkComponent()
        : base("Motus Urdf Link", "ULink", "URDF link from Rhino geometry (meters; Box/Mesh/Brep/Surface/…)", "Model", "stack") { }

    protected override IReadOnlyList<string> AiKeywords { get; } =
    [
        "Wire: any Rhino geometry (Box/Mesh/Brep/Surface/Extrusion/SubD) -> V",
        "Next: L->Motus Urdf Assemble Links",
    ];

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddTextParameter("Name", "N", "Link name (unique within the description)", GH_ParamAccess.item);
        p.AddGeometryParameter("Visual", "V", "Visual geometry in link frame (meters): Box, Mesh, Brep, Surface, Extrusion, SubD, …", GH_ParamAccess.list);
        p.AddGeometryParameter("Collision", "C", "Optional collision geometry; defaults to Visual when omitted", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddParameter(new Param_MotusUrdfLink(), "Link", "L", "URDF link", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var name = "";
        var visuals = new List<IGH_GeometricGoo>();
        var collisions = new List<IGH_GeometricGoo>();
        if (!da.GetData(0, ref name) || string.IsNullOrWhiteSpace(name))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Name is required.");
            return;
        }
        if (!da.GetDataList(1, visuals) || visuals.Count == 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "At least one Visual geometry is required.");
            return;
        }
        da.GetDataList(2, collisions);

        try
        {
            if (!TryConvertAll(visuals, out var visualGeoms, out var visualError))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, visualError!);
                return;
            }

            List<UrdfGeometry> collisionGeoms;
            if (collisions.Count == 0)
            {
                collisionGeoms = visualGeoms;
            }
            else if (!TryConvertAll(collisions, out collisionGeoms, out var collisionError))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, collisionError!);
                return;
            }

            da.SetData(0, new UrdfLinkGoo(new UrdfLink(name, visualGeoms, collisionGeoms)));
        }
        catch (Exception ex)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
        }
    }

    private static bool TryConvertAll(
        IReadOnlyList<IGH_GeometricGoo> geos,
        out List<UrdfGeometry> result,
        out string? error)
    {
        result = new List<UrdfGeometry>(geos.Count);
        error = null;
        foreach (var geo in geos)
        {
            if (!TryToUrdfGeometry(geo, out var urdf, out error) || urdf is null)
                return false;
            result.Add(urdf);
        }

        return result.Count > 0;
    }

    // ponytail: keep Box as Urdf box; everything else → mesh via Rhino meshing.
    private static bool TryToUrdfGeometry(IGH_GeometricGoo? goo, out UrdfGeometry? geometry, out string? error)
    {
        geometry = null;
        error = null;
        if (goo is null)
        {
            error = "Geometry item is empty.";
            return false;
        }

        if (goo is GH_Box { Value: { IsValid: true } box })
        {
            var sx = box.X.Length;
            var sy = box.Y.Length;
            var sz = box.Z.Length;
            if (sx <= 0 || sy <= 0 || sz <= 0)
            {
                error = "Box extents must be > 0.";
                return false;
            }

            geometry = UrdfGeometry.Box(sx, sy, sz, FrameConversion.FromPlane(box.Plane));
            return true;
        }

        if (!TryMeshAny(goo, out var mesh) || mesh is not { IsValid: true } || mesh.Faces.Count == 0)
        {
            error = "Could not mesh geometry — wire Box, Mesh, Brep, Surface, Extrusion, SubD, or similar (meters).";
            return false;
        }

        var vertices = new List<double[]>(mesh.Vertices.Count);
        foreach (var v in mesh.Vertices)
            vertices.Add([v.X, v.Y, v.Z]);

        var indices = new List<int>(mesh.Faces.Count * 3);
        foreach (var face in mesh.Faces)
        {
            indices.Add(face.A);
            indices.Add(face.B);
            indices.Add(face.C);
            if (!face.IsTriangle)
            {
                indices.Add(face.A);
                indices.Add(face.C);
                indices.Add(face.D);
            }
        }

        geometry = UrdfGeometry.Mesh(vertices, indices);
        return true;
    }

    private static bool TryMeshAny(IGH_GeometricGoo goo, out Mesh? mesh)
    {
        mesh = null;
        if (goo is GH_Mesh { Value: { IsValid: true } m } && m.Faces.Count > 0)
        {
            mesh = m;
            return true;
        }

        var gb = goo.ScriptVariable() as GeometryBase;
        if (gb is null && goo.CastTo(out GeometryBase cast))
            gb = cast;
        if (gb is null)
            return false;

        switch (gb)
        {
            case Mesh direct when direct.IsValid && direct.Faces.Count > 0:
                mesh = direct;
                return true;
            case Brep brep when brep.IsValid:
                mesh = JoinMeshes(Mesh.CreateFromBrep(brep, MeshingParameters.Default));
                return mesh is not null;
            case Extrusion extrusion: // before Surface — Extrusion : Surface
                mesh = JoinMeshes(Mesh.CreateFromBrep(extrusion.ToBrep(), MeshingParameters.Default));
                return mesh is not null;
            case Surface surface:
                mesh = JoinMeshes(Mesh.CreateFromBrep(Brep.CreateFromSurface(surface), MeshingParameters.Default));
                return mesh is not null;
            case SubD subd:
                mesh = Mesh.CreateFromSubD(subd, 2);
                return mesh is { IsValid: true } && mesh.Faces.Count > 0;
            default:
            {
                // Last resort: Brep conversion if Rhino can promote the type.
                var asBrep = Brep.TryConvertBrep(gb);
                if (asBrep is { IsValid: true })
                {
                    mesh = JoinMeshes(Mesh.CreateFromBrep(asBrep, MeshingParameters.Default));
                    return mesh is not null;
                }

                return false;
            }
        }
    }

    private static Mesh? JoinMeshes(Mesh[]? parts)
    {
        if (parts is not { Length: > 0 }) return null;
        var mesh = new Mesh();
        foreach (var part in parts)
            mesh.Append(part);
        return mesh.Faces.Count > 0 ? mesh : null;
    }

    public override Guid ComponentGuid => new Guid("2b3c4d5e-6f7a-4b2c-9d3e-4f5a6b7c8d92");
}

/// <summary>
/// Build a <see cref="UrdfJoint"/> connecting a parent/child link. The Axis line's start point is the
/// joint origin (parent-frame, meters) and its direction is the joint axis.
/// </summary>
public sealed class MotusUrdfJointComponent : MotusComponentBase
{
    public MotusUrdfJointComponent()
        : base("Motus Urdf Joint", "UJoint", "URDF joint (revolute/continuous/prismatic/fixed) between two links", "Model", "gear-six") { }

    protected override IReadOnlyList<string> AiKeywords { get; } =
    [
        "Note: Axis line Start = origin in parent frame, direction = joint axis",
        "Next: J->Motus Urdf Assemble Joints",
    ];

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddTextParameter("Name", "N", "Joint name (unique within the description)", GH_ParamAccess.item);
        p.AddTextParameter("Type", "T", "Joint type: Revolute, Continuous, Prismatic, Fixed (or R/C/P/F)", GH_ParamAccess.item, "Revolute");
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("Parent", "Pa", "Parent link name", GH_ParamAccess.item);
        p.AddTextParameter("Child", "Ch", "Child link name", GH_ParamAccess.item);
        p.AddLineParameter("Axis", "Ax", "Origin (Start, parent frame, meters) and axis direction (End-Start); default +Z at origin", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Lower", "Lo", "Lower limit (radians or meters); ignored for Fixed", GH_ParamAccess.item, 0.0);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Upper", "Up", "Upper limit (radians or meters); ignored for Fixed", GH_ParamAccess.item, 0.0);
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("MimicJoint", "Mj", "Optional joint name this joint mimics (q = Mult*q[target] + Off)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("MimicMult", "Mm", "Mimic multiplier", GH_ParamAccess.item, 1.0);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("MimicOffset", "Mo", "Mimic offset", GH_ParamAccess.item, 0.0);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddParameter(new Param_MotusUrdfJoint(), "Joint", "J", "URDF joint", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var name = "";
        var type = "Revolute";
        var parent = "";
        var child = "";
        var axis = Line.Unset;
        var lower = 0.0;
        var upper = 0.0;
        string? mimicJoint = null;
        var mimicMult = 1.0;
        var mimicOffset = 0.0;

        if (!da.GetData(0, ref name) || string.IsNullOrWhiteSpace(name))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Name is required.");
            return;
        }
        da.GetData(1, ref type);
        if (!da.GetData(2, ref parent) || string.IsNullOrWhiteSpace(parent))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Parent link name is required.");
            return;
        }
        if (!da.GetData(3, ref child) || string.IsNullOrWhiteSpace(child))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Child link name is required.");
            return;
        }
        if (!da.GetData(4, ref axis) || !axis.IsValid)
            axis = new Line(Point3d.Origin, new Point3d(0, 0, 1));
        da.GetData(5, ref lower);
        da.GetData(6, ref upper);
        da.GetData(7, ref mimicJoint);
        da.GetData(8, ref mimicMult);
        da.GetData(9, ref mimicOffset);

        try
        {
            var direction = axis.Direction;
            var joint = new UrdfJoint(
                name, type, parent, child,
                axis.FromX, axis.FromY, axis.FromZ,
                direction.X, direction.Y, direction.Z,
                lower, upper,
                mimicJoint, mimicMult, mimicOffset);
            da.SetData(0, new UrdfJointGoo(joint));
        }
        catch (Exception ex)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
        }
    }

    public override Guid ComponentGuid => new Guid("3c4d5e6f-7a8b-4c3d-ae4f-5a6b7c8d9ea3");
}

/// <summary>
/// Assemble links and joints into a validated <see cref="RobotDescription"/> tree. Recompute is debounced
/// so rapid upstream edits (dragging a slider, editing a list) settle before the (cheap but non-trivial)
/// topology validation runs — mirrors the ~100-150ms authoring budget from ADR 0002.
/// </summary>
public sealed class MotusUrdfAssembleComponent : MotusComponentBase
{
    private const int AssembleDebounceMs = 120;

    private int _gen;
    private string? _settledFingerprint;
    private RobotDescriptionGoo? _cachedGoo;

    public MotusUrdfAssembleComponent()
        : base("Motus Urdf Assemble", "UAssemble", "Validate and assemble URDF links/joints into a robot description tree", "Model", "tree-structure") { }

    protected override IReadOnlyList<string> AiKeywords { get; } =
    [
        "Wire: Motus Urdf Link L (list); Motus Urdf Joint J (list)",
        "Next: D->Motus Urdf Explode / Motus Urdf Attach / RobotDescriptionSession.Project",
    ];

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddTextParameter("Name", "N", "Robot description name", GH_ParamAccess.item, "robot");
        p[p.ParamCount - 1].Optional = true;
        p.AddParameter(new Param_MotusUrdfLink(), "Links", "L", "URDF links (exactly one must be the root — no parent joint)", GH_ParamAccess.list);
        p.AddParameter(new Param_MotusUrdfJoint(), "Joints", "J", "URDF joints connecting the links into a tree", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("Tip", "Tip", "Optional tip link name", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddParameter(new Param_MotusRobotDescription(), "Description", "D", "Assembled robot description", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var name = "robot";
        var linkGoos = new List<UrdfLinkGoo>();
        var jointGoos = new List<UrdfJointGoo>();
        string? tip = null;
        da.GetData(0, ref name);
        da.GetDataList(1, linkGoos);
        da.GetDataList(2, jointGoos);
        da.GetData(3, ref tip);

        var fingerprint = string.Join(
            "|",
            name,
            tip ?? "",
            string.Join(",", linkGoos.Select(g => RuntimeHelpers.GetHashCode(g.Value))),
            string.Join(",", jointGoos.Select(g => RuntimeHelpers.GetHashCode(g.Value))));

        if (fingerprint != _settledFingerprint)
        {
            var gen = ++_gen;
            _settledFingerprint = fingerprint;
            if (OnPingDocument() is Grasshopper.Kernel.GH_Document doc)
            {
                doc.ScheduleSolution(AssembleDebounceMs, _ =>
                {
                    if (gen != _gen || Locked) return;
                    ExpireSolution(false);
                });
            }

            if (_cachedGoo is not null)
            {
                da.SetData(0, _cachedGoo);
                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Inputs changed — settling…");
            }
            return;
        }

        var links = linkGoos.Select(g => g.Value).Where(v => v is not null).Cast<UrdfLink>().ToList();
        var joints = jointGoos.Select(g => g.Value).Where(v => v is not null).Cast<UrdfJoint>().ToList();

        if (!RobotDescription.TryAssemble(name, links, joints, tip, out var description, out var diagnostics))
        {
            _cachedGoo = null;
            foreach (var error in diagnostics.Errors)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, error);
            foreach (var warning in diagnostics.Warnings)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, warning);
            return;
        }

        foreach (var warning in diagnostics.Warnings)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, warning);

        _cachedGoo = new RobotDescriptionGoo(description!);
        da.SetData(0, _cachedGoo);
    }

    public override Guid ComponentGuid => new Guid("4d5e6f7a-8b9c-4d4e-bf5a-6b7c8d9eafb4");
}

/// <summary>Decompose a <see cref="RobotDescription"/> back into its flat link and joint lists.</summary>
public sealed class MotusUrdfExplodeComponent : MotusComponentBase
{
    public MotusUrdfExplodeComponent()
        : base("Motus Urdf Explode", "UExplode", "Decompose a robot description into its links and joints", "Model", "list-plus") { }

    protected override IReadOnlyList<string> AiKeywords { get; } =
    [
        "Wire: Motus Urdf Assemble D",
    ];

    protected override void RegisterInputParams(GH_InputParamManager p) =>
        p.AddParameter(new Param_MotusRobotDescription(), "Description", "D", "Robot description to decompose", GH_ParamAccess.item);

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddParameter(new Param_MotusUrdfLink(), "Links", "L", "URDF links", GH_ParamAccess.list);
        p.AddParameter(new Param_MotusUrdfJoint(), "Joints", "J", "URDF joints", GH_ParamAccess.list);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        RobotDescriptionGoo? descGoo = null;
        if (!da.GetData(0, ref descGoo) || descGoo?.Value is null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Description is required.");
            return;
        }

        var (links, joints) = descGoo.Value.Explode();
        da.SetDataList(0, links.Select(l => new UrdfLinkGoo(l)));
        da.SetDataList(1, joints.Select(j => new UrdfJointGoo(j)));
    }

    public override Guid ComponentGuid => new Guid("5e6f7a8b-9c0d-4e5f-ca6b-7c8d9eafb0c5");
}

/// <summary>
/// Graft a child <see cref="RobotDescription"/> (e.g. a gripper or turntable mechanism) onto a parent
/// description via a new fixed joint. The attach frame must be an identity rotation — <see cref="Plane"/>
/// origin only; pre-rotate the child's own links/axes for a rotated mount.
/// </summary>
public sealed class MotusUrdfAttachComponent : MotusComponentBase
{
    public MotusUrdfAttachComponent()
        : base("Motus Urdf Attach", "UAttach", "Graft a child robot description onto a parent link via a fixed joint", "Model", "paperclip") { }

    protected override IReadOnlyList<string> AiKeywords { get; } =
    [
        "Wire: Motus Urdf Assemble D (arm) -> Parent; Motus Urdf Assemble D (tool/mechanism) -> Child",
        "Note: Plane origin only — attach frame rotation must be identity",
    ];

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddParameter(new Param_MotusRobotDescription(), "Parent", "Pd", "Parent robot description", GH_ParamAccess.item);
        p.AddParameter(new Param_MotusRobotDescription(), "Child", "Cd", "Child robot description to graft on", GH_ParamAccess.item);
        p.AddTextParameter("ParentLink", "Pl", "Parent link to attach the child's root link to", GH_ParamAccess.item);
        p.AddPlaneParameter("Plane", "Pln", "Attach origin in the parent link's frame (identity rotation)", GH_ParamAccess.item, Plane.WorldXY);
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("JointName", "Jn", "Optional name for the new fixed joint", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddParameter(new Param_MotusRobotDescription(), "Description", "D", "Merged robot description", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        RobotDescriptionGoo? parentGoo = null;
        RobotDescriptionGoo? childGoo = null;
        var parentLink = "";
        var pl = Plane.WorldXY;
        string? jointName = null;

        if (!da.GetData(0, ref parentGoo) || parentGoo?.Value is null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Parent description is required.");
            return;
        }
        if (!da.GetData(1, ref childGoo) || childGoo?.Value is null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Child description is required.");
            return;
        }
        if (!da.GetData(2, ref parentLink) || string.IsNullOrWhiteSpace(parentLink))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "ParentLink is required.");
            return;
        }
        da.GetData(3, ref pl);
        da.GetData(4, ref jointName);
        if (!pl.IsValid) pl = Plane.WorldXY;

        // RobotDescription.Attach rejects non-identity rotation — fail here instead of stripping it.
        if (!IsIdentityOrientation(pl))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error,
                "Attach Plane must be identity orientation (translation only). Pre-rotate the child's geometry/axes for a rotated mount.");
            return;
        }

        try
        {
            var attachFrame = new Frame(pl.OriginX, pl.OriginY, pl.OriginZ);
            var merged = parentGoo.Value.Attach(childGoo.Value, parentLink, attachFrame, jointName);
            da.SetData(0, new RobotDescriptionGoo(merged));
        }
        catch (Exception ex)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
        }
    }

    private static bool IsIdentityOrientation(Plane pl) =>
        pl.XAxis.IsParallelTo(Vector3d.XAxis) == 1 &&
        pl.YAxis.IsParallelTo(Vector3d.YAxis) == 1 &&
        pl.ZAxis.IsParallelTo(Vector3d.ZAxis) == 1;

    public override Guid ComponentGuid => new Guid("6f7a8b9c-0d1e-4f60-db7c-8d9eafb0c1d6");
}
