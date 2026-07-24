using System.Globalization;
using Motus.Geometry;
using GH_IO.Serialization;

namespace Motus.GH.Data;

/// <summary>URDF authoring link — visual/collision geometry + mass/tint. See <see cref="UrdfLink"/>.</summary>
public sealed class UrdfLinkGoo : MotusGooBase<UrdfLink>
{
    public UrdfLinkGoo() { }
    public UrdfLinkGoo(UrdfLink link) : base(link) { }

    public override string ToString() => Value is null
        ? "Link"
        : $"{Value.Name} (V:{Value.Visuals.Count} C:{Value.Collisions.Count})";
}

/// <summary>URDF authoring joint connecting two links. See <see cref="UrdfJoint"/>.</summary>
public sealed class UrdfJointGoo : MotusGooBase<UrdfJoint>
{
    public UrdfJointGoo() { }
    public UrdfJointGoo(UrdfJoint joint) : base(joint) { }

    public override string ToString() => Value is null
        ? "Joint"
        : $"{Value.Name} ({Value.Kind}: {Value.ParentLink}->{Value.ChildLink})";
}

/// <summary>
/// URDF authoring robot graph (links + joints) — see <see cref="RobotDescription"/>. Distinct from
/// <see cref="RobotModelGoo"/>: this wraps the pre-assembly authoring tree, not a plannable robot.
/// </summary>
public sealed class RobotDescriptionGoo : MotusGooBase<RobotDescription>
{
    /// <summary>Persisted across GH doc save/reload. Live <see cref="MotusGooBase{T}.Value"/> is rebuilt
    /// by upstream Assemble/Attach on the next solve (no URDF XML in Grasshopper — Export uses Motus.NET).</summary>
    public string? PersistedName { get; set; }
    public string? PersistedFingerprint { get; set; }

    public RobotDescriptionGoo() { }
    public RobotDescriptionGoo(RobotDescription description) : base(description) { }

    public override string ToString() => Value is null
        ? PersistedName ?? "RobotDescription"
        : $"{Value.Name} (L:{Value.Links.Count} J:{Value.Joints.Count})";

    public override bool Write(GH_IWriter writer)
    {
        if (Value is null) return true;
        writer.SetString("Name", Value.Name);
        writer.SetString("Fingerprint", Value.Fingerprint.ToString(CultureInfo.InvariantCulture));
        return true;
    }

    public override bool Read(GH_IReader reader)
    {
        if (reader.ItemExists("Name"))
            PersistedName = reader.GetString("Name");
        if (reader.ItemExists("Fingerprint"))
            PersistedFingerprint = reader.GetString("Fingerprint");
        return true;
    }
}
