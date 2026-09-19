using Motus.Core;
using Motus.Geometry;

namespace Motus.GH.Rhino;

public static class SerialReachSamples
{
    public static IReadOnlyList<Frame> Sample(RobotModel robot, SerialJointChain chain, int count, int seed)
    {
        if (count < 0 || count > 512) throw new ArgumentOutOfRangeException(nameof(count));
        var n = chain.Joints.Length;
        if (robot.Preset.JointLimits.Count < n) throw new ArgumentException("Robot limits do not cover the selected tip chain.");
        // All Motus planning layouts put tip-chain coordinates first. Side branches do not
        // change this TCP; no tree-driver/preset index correspondence is assumed.
        var random = new Random(seed);
        var strata = Enumerable.Range(0, n).Select(_ => Enumerable.Range(0, count).ToArray()).ToArray();
        foreach (var axis in strata)
            for (var i = axis.Length - 1; i > 0; i--)
            { var j = random.Next(i + 1); (axis[i], axis[j]) = (axis[j], axis[i]); }
        var fk = new SerialForwardKinematics(chain);
        var result = new Frame[count];
        for (var i = 0; i < count; i++)
        {
            var q = new double[n];
            for (var j = 0; j < n; j++)
            {
                var limit = robot.Preset.JointLimits[j];
                q[j] = limit.Min + (strata[j][i] + random.NextDouble()) / count * (limit.Max - limit.Min);
            }
            result[i] = fk.ComputeTcp(new JointState(q), robot.Preset.BaseFrame, robot.Preset.ToolFrame).Tcp;
        }
        return result;
    }
}
