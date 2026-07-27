using Motus.Core;
using Motus.Geometry;
using Motus.GH.Rhino;
using Rhino.Geometry;

namespace Motus.GH.Planning;

/// <summary>
/// Ground-contact rings for Family=legged TreeFK preview (foot tip near terrain / body-floor).
/// </summary>
internal static class LeggedContactPreview
{
    /// <summary>Planted if tip within this of terrain Z. Must stay below default WalkHex Lift (0.02 m).</summary>
    public const double GroundTolMeters = 0.008;
    public const double RingRadiusMeters = 0.028;

    public static void CollectGroundCircles(
        KinematicTree tree,
        RobotCollisionModel? previewGeometry,
        IReadOnlyList<double> trajectoryQ,
        IReadOnlyList<string>? armJointNames,
        JointState? treeDriverHome,
        Frame? dynamicBase,
        ICollection<Circle> dest,
        LeggedGait.TerrainHeight? terrain = null,
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
                tipLen = 0.10; // ponytail: WalkHex default Tb if geom missing

            var local = Transforms.TransformPoint(mats[li], tipLen, 0, 0);
            var world = Transforms.TransformPoint(baseM, local[0], local[1], local[2]);
            var expectedZ = SampleGroundZ(terrain, dynamicBase, world[0], world[1]);
            if (!double.IsFinite(expectedZ) || Math.Abs(world[2] - expectedZ) > groundTol)
                continue;

            var center = new Point3d(world[0], world[1], expectedZ);
            dest.Add(new Circle(new Plane(center, Vector3d.ZAxis), radius));
        }
    }

    private static double SampleGroundZ(
        LeggedGait.TerrainHeight? terrain, Frame? dynamicBase, double x, double y)
    {
        if (terrain is not null)
        {
            var z = terrain(x, y);
            if (double.IsFinite(z))
                return z;
        }

        return dynamicBase?.Z ?? 0;
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
            // WalkHex tibias are Mesh (0→L on +X); Capsule/Box used by other authors.
            var tip = obj.Shape switch
            {
                CollisionShape.Capsule => obj.Pose.X + obj.ExtentY,
                CollisionShape.Box => obj.Pose.X + obj.ExtentX,
                CollisionShape.Sphere => obj.Pose.X + obj.ExtentX,
                CollisionShape.Mesh when obj.MeshAabbMax is { Length: >= 1 } max =>
                    obj.Pose.X + max[0],
                _ => 0,
            };
            if (tip > 0)
                map[link.LinkName] = tip;
        }

        return map;
    }
}
