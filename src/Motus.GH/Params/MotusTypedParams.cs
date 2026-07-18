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
        : base("Segment", "Seg", "Motus motion segment", "Motus", "Params") { }

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
