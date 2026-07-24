using Grasshopper.Kernel;
using GH_IO.Serialization;
using Motus.GH.Data;
using Motus.GH.Params;
using Motus.GH.UI;
using Motus.Presets;
using System;

namespace Motus.GH.Components;

/// <summary>
/// Thin GH wrapper: click Write → <see cref="UrdfWriter.Write"/>. No URDF XML in Grasshopper.
/// </summary>
public sealed class MotusUrdfExportComponent : MotusComponentBase
{
    private bool _run;
    private string? _lastPath;
    private string _status = "Click Write to export URDF.";

    public MotusUrdfExportComponent()
        : base(
            "Motus Export URDF",
            "UrdfExport",
            "Write a Motus RobotDescription to a .urdf file (Motus.NET UrdfWriter)",
            "Export",
            "export")
    {
    }

    protected override System.Collections.Generic.IReadOnlyList<string> AiKeywords { get; } =
    [
        "Wire: Motus Urdf Assemble D; Folder F",
        "Note: click Write — Motus.NET UrdfWriter only",
    ];

    public override void CreateAttributes() =>
        m_attributes = new ButtonAttributes(this, () => "Write", () => false, RequestWrite);

    private void RequestWrite()
    {
        _run = true;
        ExpireSolution(true);
    }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddParameter(
            new Param_MotusRobotDescription(),
            "Description",
            "D",
            "RobotDescription from Motus Urdf Assemble / Attach",
            GH_ParamAccess.item);
        p.AddTextParameter("Folder", "F", "Output folder for the .urdf (+ meshes/)", GH_ParamAccess.item);
        p.AddTextParameter("Name", "N", "Optional file name override (defaults to description name)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddTextParameter("Path", "P", "Path of the written .urdf file (set after Write)", GH_ParamAccess.item);
        p.AddTextParameter("Status", "Msg", "Status message", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        RobotDescriptionGoo? descGoo = null;
        var folder = "";
        var name = "";
        da.GetData(0, ref descGoo);
        da.GetData(1, ref folder);
        da.GetData(2, ref name);

        if (!_run)
        {
            da.SetData(0, _lastPath ?? "");
            da.SetData(1, _status);
            return;
        }

        _run = false;

        if (descGoo?.Value is null)
        {
            _status = "Wire a RobotDescription (Motus Urdf Assemble / Attach).";
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, _status);
            da.SetData(0, _lastPath ?? "");
            da.SetData(1, _status);
            return;
        }

        if (string.IsNullOrWhiteSpace(folder))
        {
            _status = "Set an output Folder.";
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, _status);
            da.SetData(0, _lastPath ?? "");
            da.SetData(1, _status);
            return;
        }

        try
        {
            var fileName = string.IsNullOrWhiteSpace(name) ? null : name.Trim();
            var path = UrdfWriter.Write(descGoo.Value, folder.Trim(), fileName, writeMeshes: true);
            _lastPath = path;
            var d = descGoo.Value;
            _status = $"Wrote {d.Links.Count} links / {d.Joints.Count} joints -> {path}";
            da.SetData(0, path);
            da.SetData(1, _status);
        }
        catch (Exception ex)
        {
            _status = ex.Message;
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, _status);
            da.SetData(0, _lastPath ?? "");
            da.SetData(1, _status);
        }
    }

    public override bool Write(GH_IWriter writer)
    {
        if (_lastPath is not null)
            writer.SetString("LastPath", _lastPath);
        return base.Write(writer);
    }

    public override bool Read(GH_IReader reader)
    {
        if (reader.ItemExists("LastPath"))
        {
            _lastPath = reader.GetString("LastPath");
            _status = $"Last write: {_lastPath}";
        }
        return base.Read(reader);
    }

    public override Guid ComponentGuid => new("2f6c1d3a-9b7e-4c5a-8e2d-6a1f4b3c7d90");
}
