using Motus.Core;
using Motus.Geometry;
using Motus.GH.Rhino;
using Rhino.Geometry;

namespace Motus.GH.Planning;

/// <summary>
/// Ground-contact rings for Family=legged TreeFK preview (foot tip near body-floor / terrain).
/// </summary>
internal static class LeggedContactPreview
{
    public const double GroundTolMeters = 0.02;
    public const double RingRadiusMeters = 0.028;

    public static void CollectGroundCircles(
        KinematicTree tree,
        RobotCollisionModel? previewGeometry,
        IReadOnlyList<double> trajectoryQ,
        IReadOnlyList<string>? armJointNames,
        JointState? treeDriverHome,
        Frame? dynamicBase,
        ICollection<Circle> dest,
        double groundTol = GroundTolMeters,
        double radius = RingRadiusMeters)
    {
        dest.Clear();
        if (tree.DriverCount <= 0 || radius <= 0)
            return;

        var driverQ = new double[tree.DriverCount];
        var fillErr = KinematicsPreview.TryFillTreeDriverQ(
            tree, trajectoryQ, armJointNames, treeDriverHome?.Positions, driverQ);
        if (fillErr is not null)
        {
            // Full-driver gait: trajectory already matches tree drivers.
            if (trajectoryQ.Count != tree.DriverCount)
                return;
            for (var i = 0; i < driverQ.Length; i++)
                driverQ[i] = trajectoryQ[i];
        }

        var tipLenByLink = TipLengthByTibiaLink(previewGeometry);
        var fk = new TreeForwardKinematics(tree);
        var mats = new double[tree.Links.Count][];
        for (var i = 0; i < mats.Length; i++)
            mats[i] = new double[16];
        fk.ComputeLinkTransformsInto(driverQ, mats);

        var baseM = dynamicBase is { } db
            ? Transforms.FromFrame(db)
            : Transforms.Identity();

        for (var li = 0; li < tree.Links.Count; li++)
        {
            var name = tree.Links[li].Name;
            if (!name.EndsWith("_tibia", StringComparison.OrdinalIgnoreCase))
                continue;

            if (!tipLenByLink.TryGetValue(name, out var tipLen) || tipLen <= 0)
                tipLen = 0.19; // ponytail: WalkHex default tibia if geom missing

            var local = Transforms.TransformPoint(mats[li], tipLen, 0, 0);
            var world = Transforms.TransformPoint(baseM, local[0], local[1], local[2]);
            // Body-floor ≈ base.Z; planted feet sit near it (flat or mild terrain).
            var baseZ = dynamicBase?.Z ?? 0;
            if (Math.Abs(world[2] - baseZ) > groundTol)
                continue;

            var center = new Point3d(world[0], world[1], world[2]);
            dest.Add(new Circle(new Plane(center, Vector3d.ZAxis), radius));
        }
    }

    private static Dictionary<string, double> TipLengthByTibiaLink(RobotCollisionModel? geom)
    {
        var map = new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase);
        if (geom is null) return map;

        foreach (var link in geom.Links)
        {
            if (!link.LinkName.EndsWith("_tibia", StringComparison.OrdinalIgnoreCase))
                continue;
            var obj = link.LocalGeometry;
            // Cylinder→Capsule: origin at mid-link (L/2 along X), ExtentY = L/2 → tip at Origin.X + ExtentY.
            var tip = obj.Shape switch
            {
                CollisionShape.Capsule => obj.Pose.X + obj.ExtentY,
                CollisionShape.Box => obj.Pose.X + obj.ExtentX,
                CollisionShape.Sphere => obj.Pose.X + obj.ExtentX,
                _ => 0,
            };
            if (tip > 0)
                map[link.LinkName] = tip;
        }

        return map;
    }
}
