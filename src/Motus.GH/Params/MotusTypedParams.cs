using Grasshopper.Kernel;
using Motus.GH.Data;
using Motus.GH.Resources;
using System.Drawing;

namespace Motus.GH.Params;

/// <summary>Typed Motus robot pin — rejects non-RobotModelGoo wires at connect time.</summary>
public sealed class Param_MotusRobot : GH_PersistentParam<RobotModelGoo>
{
    public Param_MotusRobot()
        : base("Robot", "Rb", "Motus robot model", "Motus", "Params") { }

    public override Guid ComponentGuid => new("a11e8488-943e-426f-b205-e8db5f684901");
    public override GH_Exposure Exposure => GH_Exposure.hidden;
    protected override Bitmap Icon => MotusIcon.Get("robot", MotusIcon.SubcategoryColor("Model"));
    protected override GH_GetterResult Prompt_Singular(ref RobotModelGoo value) => GH_GetterResult.cancel;
    protected override GH_GetterResult Prompt_Plural(ref List<RobotModelGoo> values) => GH_GetterResult.cancel;
}

/// <summary>Typed Motus trajectory pin.</summary>
public sealed class Param_MotusTrajectory : GH_PersistentParam<TrajectoryGoo>
{
    public Param_MotusTrajectory()
        : base("Trajectory", "Tr", "Motus trajectory", "Motus", "Params") { }

    public override Guid ComponentGuid => new("b22e8488-943e-426f-b205-e8db5f684902");
    public override GH_Exposure Exposure => GH_Exposure.hidden;
    protected override Bitmap Icon => MotusIcon.Get("flow-arrow", MotusIcon.SubcategoryColor("Plan"));
    protected override GH_GetterResult Prompt_Singular(ref TrajectoryGoo value) => GH_GetterResult.cancel;
    protected override GH_GetterResult Prompt_Plural(ref List<TrajectoryGoo> values) => GH_GetterResult.cancel;
}

/// <summary>Typed Motus joint state pin.</summary>
public sealed class Param_MotusJointState : GH_PersistentParam<JointStateGoo>
{
    public Param_MotusJointState()
        : base("State", "Js", "Motus joint state", "Motus", "Params") { }

    public override Guid ComponentGuid => new("c33e8488-943e-426f-b205-e8db5f684903");
    public override GH_Exposure Exposure => GH_Exposure.hidden;
    protected override Bitmap Icon => MotusIcon.Get("gear-six", MotusIcon.SubcategoryColor("Model"));
    protected override GH_GetterResult Prompt_Singular(ref JointStateGoo value) => GH_GetterResult.cancel;
    protected override GH_GetterResult Prompt_Plural(ref List<JointStateGoo> values) => GH_GetterResult.cancel;
}

/// <summary>Typed Motus collision scene pin.</summary>
public sealed class Param_MotusCollisionScene : GH_PersistentParam<CollisionSceneGoo>
{
    public Param_MotusCollisionScene()
        : base("Scene", "Sc", "Motus collision scene", "Motus", "Params") { }

    public override Guid ComponentGuid => new("d44e8488-943e-426f-b205-e8db5f684904");
    public override GH_Exposure Exposure => GH_Exposure.hidden;
    protected override Bitmap Icon => MotusIcon.Get("circles-three-plus", MotusIcon.SubcategoryColor("Collision"));
    protected override GH_GetterResult Prompt_Singular(ref CollisionSceneGoo value) => GH_GetterResult.cancel;
    protected override GH_GetterResult Prompt_Plural(ref List<CollisionSceneGoo> values) => GH_GetterResult.cancel;
}

/// <summary>Typed Motus motion segment pin.</summary>
public sealed class Param_MotusSegment : GH_PersistentParam<MotionSegmentGoo>
{
    public Param_MotusSegment()
        : base("Segment", "Seg", "Motus Move segment", "Motus", "Params") { }

    public override Guid ComponentGuid => new("e55e8488-943e-426f-b205-e8db5f684905");
    public override GH_Exposure Exposure => GH_Exposure.hidden;
    protected override Bitmap Icon => MotusIcon.Get("line-segments", MotusIcon.SubcategoryColor("Plan"));
    protected override GH_GetterResult Prompt_Singular(ref MotionSegmentGoo value) => GH_GetterResult.cancel;
    protected override GH_GetterResult Prompt_Plural(ref List<MotionSegmentGoo> values) => GH_GetterResult.cancel;
}

/// <summary>Typed Motus tool definition pin.</summary>
public sealed class Param_MotusTool : GH_PersistentParam<ToolGoo>
{
    public Param_MotusTool()
        : base("Tool", "Tl", "Motus tool definition", "Motus", "Params") { }

    public override Guid ComponentGuid => new("f66e8488-943e-426f-b205-e8db5f684906");
    public override GH_Exposure Exposure => GH_Exposure.hidden;
    protected override Bitmap Icon => MotusIcon.Get("wrench", MotusIcon.SubcategoryColor("Model"));
    protected override GH_GetterResult Prompt_Singular(ref ToolGoo value) => GH_GetterResult.cancel;
    protected override GH_GetterResult Prompt_Plural(ref List<ToolGoo> values) => GH_GetterResult.cancel;
}

/// <summary>Typed Motus end-effector state pin.</summary>
public sealed class Param_MotusToolState : GH_PersistentParam<EndEffectorStateGoo>
{
    public Param_MotusToolState()
        : base("ToolState", "Ts", "Motus end-effector state", "Motus", "Params") { }

    public override Guid ComponentGuid => new("a77e8488-943e-426f-b205-e8db5f684907");
    public override GH_Exposure Exposure => GH_Exposure.hidden;
    protected override Bitmap Icon => MotusIcon.Get("sliders-horizontal", MotusIcon.SubcategoryColor("Model"));
    protected override GH_GetterResult Prompt_Singular(ref EndEffectorStateGoo value) => GH_GetterResult.cancel;
    protected override GH_GetterResult Prompt_Plural(ref List<EndEffectorStateGoo> values) => GH_GetterResult.cancel;
}

// GUID map (0.9 N-leg Walk):
// Param_MotusLeg        b7a9381b-cbce-4df0-8e74-46d7ca62cea1
// Param_MotusBody       accf652b-0591-4d03-84ac-811a510cb2ef
// Param_MotusMechanism  4a2b9635-a730-4ee5-9272-266d1ce9bef4
// Param_MotusBodyPose   03e55c53-15d4-4b46-9927-33803788db85
// Motus Leg             9a49a661-ff4c-4b96-bb57-c977ee6f9da2
// Motus Body            92f0d969-c8ef-47c5-9ec7-514bebbd8441
// Motus Mechanism       aa18b783-9a1c-44f8-bd2b-e508c3d372ac
// Motus Body Pose       76051f49-2641-4530-8b79-c5635a8e6eaf
// Motus Walk (kept)     236f9a53-c07b-4663-bf27-950e20fb59ab
// Removed: Motus Hex c7a02fcb-… / Param_MotusHex 908aabb4-…

/// <summary>Typed Motus leg recipe pin.</summary>
public sealed class Param_MotusLeg : GH_PersistentParam<LegDefinitionGoo>
{
    public Param_MotusLeg()
        : base("Leg", "Leg", "Motus leg definition", "Motus", "Params") { }

    public override Guid ComponentGuid => new("b7a9381b-cbce-4df0-8e74-46d7ca62cea1");
    public override GH_Exposure Exposure => GH_Exposure.hidden;
    protected override Bitmap Icon => MotusIcon.Get("polygon", MotusIcon.SubcategoryColor("Model"));
    protected override GH_GetterResult Prompt_Singular(ref LegDefinitionGoo value) => GH_GetterResult.cancel;
    protected override GH_GetterResult Prompt_Plural(ref List<LegDefinitionGoo> values) => GH_GetterResult.cancel;
}

/// <summary>Typed Motus body (hip frames) pin.</summary>
public sealed class Param_MotusBody : GH_PersistentParam<LeggedBodyGoo>
{
    public Param_MotusBody()
        : base("Body", "Bdy", "Motus legged body hips", "Motus", "Params") { }

    public override Guid ComponentGuid => new("accf652b-0591-4d03-84ac-811a510cb2ef");
    public override GH_Exposure Exposure => GH_Exposure.hidden;
    protected override Bitmap Icon => MotusIcon.Get("polygon", MotusIcon.SubcategoryColor("Model"));
    protected override GH_GetterResult Prompt_Singular(ref LeggedBodyGoo value) => GH_GetterResult.cancel;
    protected override GH_GetterResult Prompt_Plural(ref List<LeggedBodyGoo> values) => GH_GetterResult.cancel;
}

/// <summary>Typed Motus legged mechanism pin (nick Mech).</summary>
public sealed class Param_MotusMechanism : GH_PersistentParam<LeggedMechanismGoo>
{
    public Param_MotusMechanism()
        : base("Mechanism", "Mech", "Motus legged mechanism", "Motus", "Params") { }

    public override Guid ComponentGuid => new("4a2b9635-a730-4ee5-9272-266d1ce9bef4");
    public override GH_Exposure Exposure => GH_Exposure.hidden;
    protected override Bitmap Icon => MotusIcon.Get("polygon", MotusIcon.SubcategoryColor("Model"));
    protected override GH_GetterResult Prompt_Singular(ref LeggedMechanismGoo value) => GH_GetterResult.cancel;
    protected override GH_GetterResult Prompt_Plural(ref List<LeggedMechanismGoo> values) => GH_GetterResult.cancel;
}

/// <summary>Typed Motus body-pose solver pin.</summary>
public sealed class Param_MotusBodyPose : GH_PersistentParam<BodyPoseSolverGoo>
{
    public Param_MotusBodyPose()
        : base("BodyPose", "Pose", "Motus body-pose solver", "Motus", "Params") { }

    public override Guid ComponentGuid => new("03e55c53-15d4-4b46-9927-33803788db85");
    public override GH_Exposure Exposure => GH_Exposure.hidden;
    protected override Bitmap Icon => MotusIcon.Get("path", MotusIcon.SubcategoryColor("Model"));
    protected override GH_GetterResult Prompt_Singular(ref BodyPoseSolverGoo value) => GH_GetterResult.cancel;
    protected override GH_GetterResult Prompt_Plural(ref List<BodyPoseSolverGoo> values) => GH_GetterResult.cancel;
}
