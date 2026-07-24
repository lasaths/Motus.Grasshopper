using Grasshopper.Kernel;
using Motus.GH.Data;
using Motus.GH.Resources;
using System.Drawing;

namespace Motus.GH.Params;

/// <summary>Typed Motus URDF link pin — visual/collision geometry plus mass/tint.</summary>
public sealed class Param_MotusUrdfLink : GH_PersistentParam<UrdfLinkGoo>
{
    public Param_MotusUrdfLink()
        : base("Urdf Link", "Lnk", "Motus URDF authoring link", "Motus", "Params") { }

    public override Guid ComponentGuid => new("011bd2b8-344d-40d9-ab9a-18c7ac7aece9");
    public override GH_Exposure Exposure => GH_Exposure.hidden;
    protected override Bitmap Icon => MotusIcon.Get("tree-structure", MotusIcon.SubcategoryColor("Model"));
    protected override GH_GetterResult Prompt_Singular(ref UrdfLinkGoo value) => GH_GetterResult.cancel;
    protected override GH_GetterResult Prompt_Plural(ref List<UrdfLinkGoo> values) => GH_GetterResult.cancel;
}

/// <summary>Typed Motus URDF joint pin — connects two authoring links.</summary>
public sealed class Param_MotusUrdfJoint : GH_PersistentParam<UrdfJointGoo>
{
    public Param_MotusUrdfJoint()
        : base("Urdf Joint", "Jnt", "Motus URDF authoring joint", "Motus", "Params") { }

    public override Guid ComponentGuid => new("04f9c4ba-dda4-4484-8f67-ad1df5bee83a");
    public override GH_Exposure Exposure => GH_Exposure.hidden;
    protected override Bitmap Icon => MotusIcon.Get("gear-six", MotusIcon.SubcategoryColor("Model"));
    protected override GH_GetterResult Prompt_Singular(ref UrdfJointGoo value) => GH_GetterResult.cancel;
    protected override GH_GetterResult Prompt_Plural(ref List<UrdfJointGoo> values) => GH_GetterResult.cancel;
}

/// <summary>Typed Motus robot description pin — pre-assembly URDF authoring graph (links + joints).</summary>
public sealed class Param_MotusRobotDescription : GH_PersistentParam<RobotDescriptionGoo>
{
    public Param_MotusRobotDescription()
        : base("Robot Description", "Rd", "Motus URDF authoring robot graph (pre-assembly)", "Motus", "Params") { }

    public override Guid ComponentGuid => new("852f43a0-23c2-4993-b26e-70b8ba67a1a7");
    public override GH_Exposure Exposure => GH_Exposure.hidden;
    protected override Bitmap Icon => MotusIcon.Get("stack", MotusIcon.SubcategoryColor("Model"));
    protected override GH_GetterResult Prompt_Singular(ref RobotDescriptionGoo value) => GH_GetterResult.cancel;
    protected override GH_GetterResult Prompt_Plural(ref List<RobotDescriptionGoo> values) => GH_GetterResult.cancel;
}
