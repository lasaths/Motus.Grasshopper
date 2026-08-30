using Motus.Core;
using Motus.Geometry;

namespace Motus.GH.Urdf;

/// <summary>
/// Shared tip-first + side-branch Plan DOF layout for Motus Robot AllDrivers and Motus Joint Table.
/// Tip-descendant drivers (e.g. Robotiq knuckles under tip) stay off Plan when
/// <paramref name="excludeTipDescendants"/> is true.
/// </summary>
public static class PlanDofComposer
{
    public readonly record struct Layout(
        IReadOnlyList<string> JointNames,
        IReadOnlyList<JointLimit> Limits,
        int TipAxisCount);

    public static Layout TipThenSideBranches(
        KinematicTree tree,
        IReadOnlyList<string> tipJointNames,
        string tipLink,
        bool excludeTipDescendants = true)
    {
        ArgumentNullException.ThrowIfNull(tree);
        tipJointNames ??= Array.Empty<string>();
        var tipSet = new HashSet<string>(tipJointNames, StringComparer.OrdinalIgnoreCase);
        var tipLinkIdx = tree.IndexOfLink(string.IsNullOrWhiteSpace(tipLink) ? "tool0" : tipLink);

        var names = new List<string>(tree.DriverCount);
        var limits = new List<JointLimit>(tree.DriverCount);

        void AddDriver(KinematicJoint j)
        {
            names.Add(j.Name);
            var vel = j.Velocity ?? Math.PI;
            limits.Add(new JointLimit(j.Lower, j.Upper, vel, vel * 2));
        }

        foreach (var tipName in tipJointNames)
        {
            for (var di = 0; di < tree.DriverCount; di++)
            {
                var j = tree.Joints[tree.DriverJointIndices[di]];
                if (string.Equals(j.Name, tipName, StringComparison.OrdinalIgnoreCase))
                {
                    AddDriver(j);
                    break;
                }
            }
        }

        var tipAxisCount = names.Count;

        for (var di = 0; di < tree.DriverCount; di++)
        {
            var j = tree.Joints[tree.DriverJointIndices[di]];
            if (tipSet.Contains(j.Name))
                continue;
            if (excludeTipDescendants && tipLinkIdx >= 0 &&
                LinkIsDescendantOf(tree, j.ChildLinkIndex, tipLinkIdx))
                continue;
            AddDriver(j);
        }

        return new Layout(names, limits, tipAxisCount);
    }

    private static bool LinkIsDescendantOf(KinematicTree tree, int linkIdx, int ancestorIdx)
    {
        if (linkIdx == ancestorIdx)
            return true;
        var byChild = new Dictionary<int, int>(tree.Joints.Count);
        foreach (var j in tree.Joints)
            byChild[j.ChildLinkIndex] = j.ParentLinkIndex;

        var guard = 0;
        var cur = linkIdx;
        while (byChild.TryGetValue(cur, out var parent))
        {
            if (parent == ancestorIdx)
                return true;
            if (++guard > 256)
                break;
            cur = parent;
        }
        return false;
    }
}
