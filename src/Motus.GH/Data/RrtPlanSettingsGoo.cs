using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Motus.GH.Planning;
using System.Globalization;

namespace Motus.GH.Data;

public sealed class RrtPlanSettingsGoo : GH_Goo<RrtPlanSettings>
{
    public RrtPlanSettingsGoo() => m_value = RrtPlanSettings.Defaults;

    public RrtPlanSettingsGoo(RrtPlanSettings value) => m_value = value;

    public override bool IsValid =>
        m_value.MaxIterations > 0 &&
        m_value.StepRadians > 0 &&
        m_value.MaxPlanTimeSeconds >= 0 &&
        m_value.GoalBias is >= 0 and <= 1;

    public override string TypeName => "RRT Settings";

    public override string TypeDescription => "Motus RRT planner tuning for joint goals with collision";

    public override IGH_Goo Duplicate() => new RrtPlanSettingsGoo(m_value);

    public override bool CastFrom(object source)
    {
        switch (source)
        {
            case RrtPlanSettingsGoo goo:
                m_value = goo.m_value;
                return true;
            case RrtPlanSettings settings:
                m_value = settings;
                return true;
            default:
                return false;
        }
    }

    public override string ToString()
    {
        var time = m_value.MaxPlanTimeSeconds > 0
            ? $", {m_value.MaxPlanTimeSeconds.ToString("F1", CultureInfo.InvariantCulture)}s limit"
            : string.Empty;
        return $"{m_value.PlannerLabel} · {m_value.MaxIterations} iter · step {m_value.StepRadians:F2} rad{time}";
    }
}
