using Grasshopper.Kernel;
using Motus.Core;
using Motus.GH;
using Motus.GH.Data;
using Motus.GH.Rhino;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Motus.GH.Components;

public sealed class MotusPlanningGroupComponent : MotusComponentBase
{
    public MotusPlanningGroupComponent() : base("Motus Planning Group", "Group", "Create or pass through a planning group", "Plan", "list-plus") { }

    protected override IReadOnlyList<string> AiKeywords { get; } =
    [
        "Next: G->Motus Plan Group (show pin)",
        "Wire: optional ColScene SRDF Groups G",
    ];

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Group", "G", "Optional existing planning group (e.g. from ColScene SRDF output)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("Name", "N", "Group name", GH_ParamAccess.item, "manipulator");
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("BaseLink", "B", "Base link name", GH_ParamAccess.item, "base_link");
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("TipLink", "Tip", "Tip link name", GH_ParamAccess.item, "tool0");
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("Joints", "J", "Joint names (leave empty to use base..tip shorthand)", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddGenericParameter("Group", "G", "Planning group", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        PlanningGroup? existing = null;
        var existingGoo = default(PlanningGroupGoo);
        if (da.GetData(0, ref existingGoo) && existingGoo?.Value is not null)
            existing = existingGoo.Value;

        var name = existing?.Name ?? "manipulator";
        var baseLink = existing?.BaseLink ?? "base_link";
        var tipLink = existing?.TipLink ?? "tool0";
        var joints = new List<string>();
        if (existing is not null)
            joints.AddRange(existing.JointNames);

        da.GetData(1, ref name);
        da.GetData(2, ref baseLink);
        da.GetData(3, ref tipLink);
        var userJoints = new List<string>();
        if (da.GetDataList(4, userJoints) && userJoints.Count > 0)
            joints = userJoints.Where(j => !string.IsNullOrWhiteSpace(j)).ToList();

        if (joints.Count == 0)
            joints.Add($"{baseLink}..{tipLink}");

        da.SetData(0, new PlanningGroupGoo(new PlanningGroup(name, baseLink, tipLink, joints)));
    }

    public override Guid ComponentGuid => new("91e2a9db-cfb4-4a6c-99a3-305ba27fdf1e");
}

public sealed class MotusAttachBodyComponent : MotusComponentBase
{
    public MotusAttachBodyComponent() : base("Motus Attach Body", "Attach", "Define an attached body in TCP-local frame", "Collision", "paperclip") { }

    protected override IReadOnlyList<string> AiKeywords { get; } =
    [
        "Wire: Motus Collision * Object O",
        "Next: A->Motus Plan Attach (show pin)",
    ];

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Object", "O", "Collision object geometry to attach", GH_ParamAccess.item);
        p.AddTextParameter("Name", "N", "Attached body name", GH_ParamAccess.item, "attached");
        p[p.ParamCount - 1].Optional = true;
        p.AddPlaneParameter("TcpLocal", "P", "TCP-local pose of attached geometry", GH_ParamAccess.item, Plane.WorldXY);
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("SourceName", "Src", "Optional scene object name to hide while attached", GH_ParamAccess.item, "");
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddGenericParameter("Attach", "A", "Attached body", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var objGoo = default(CollisionObjectGoo);
        if (!da.GetData(0, ref objGoo) || objGoo?.Value is null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Object must be a collision object.");
            return;
        }

        var name = "attached";
        var tcp = Plane.WorldXY;
        var sourceName = "";
        da.GetData(1, ref name);
        da.GetData(2, ref tcp);
        da.GetData(3, ref sourceName);

        if (string.IsNullOrWhiteSpace(name))
            name = objGoo.Value.Name;

        var attached = new AttachedBody(
            name,
            FrameConversion.FromPlane(tcp),
            objGoo.Value,
            string.IsNullOrWhiteSpace(sourceName) ? null : sourceName);
        da.SetData(0, new AttachedBodyGoo(attached));
    }

    public override Guid ComponentGuid => new("0c464ac8-0e1d-4c7a-9c8c-0a21f1046314");
}
