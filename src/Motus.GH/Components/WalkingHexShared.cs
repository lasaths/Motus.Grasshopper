using Motus.Core;
using Motus.Geometry;
using Motus.GH.Planning;
using Rhino.Geometry;
using System.Drawing;

namespace Motus.GH.Components;

/// <summary>Shared Motus Body/Leg/Mechanism → Walk build helpers (meters, radians).</summary>
internal static class WalkingHexShared
{
    internal static double[] BuildStanceQ(
        LeggedMechanism mechanism, double hip, double femur, double tibia, IReadOnlyList<double>? overrideQ)
    {
        if (overrideQ is { Count: > 0 } oq && oq.Count >= mechanism.DriverCount)
        {
            var q = new double[mechanism.DriverCount];
            for (var i = 0; i < q.Length; i++) q[i] = oq[i];
            return q;
        }

        return LeggedGaitRhino.BuildStanceQ(mechanism, hip, femur, tibia);
    }

    /// <summary>
    /// Preview URDF visuals matching <see cref="LeggedMechanism.Assemble"/> namespaced link names.
    /// </summary>
    internal static RobotDescription BuildDescription(LeggedMechanism mechanism)
    {
        var hips = mechanism.Legs.Select(l => l.HipInBody ?? Frame.Identity).ToList();
        var bodyR = EstimateBodyR(hips);
        var bodyZ = mechanism.NominalBodyClearance;
        var links = new List<UrdfLink>
        {
            new("body",
                [NGonPlateGeometry(hips, thickness: 0.02, bodyZ)],
                r: 1, g: 0.4, b: 0.7, a: 0.85),
        };
        var joints = new List<UrdfJoint>();

        for (var leg = 0; leg < mechanism.LegCount; leg++)
        {
            var def = mechanism.Legs[leg];
            var prefix = def.Name + "/";
            var hip = hips[leg];
            var mount = prefix + "mount";
            links.Add(new UrdfLink(mount, [], r: 1, g: 0.55, b: 0.15, a: 1));

            // Fixed graft mount at hip (matches Assemble Attach).
            joints.Add(new UrdfJoint(
                prefix + "attach", "fixed", "body", mount,
                hip.X, hip.Y, hip.Z, 0, 0, 1, 0, 0));

            if (def.Lengths3R is { Count: 3 } L)
            {
                var coxa = L[0];
                var femur = L[1];
                var tibia = L[2];
                var coxaLink = prefix + "coxa";
                var femurLink = prefix + "femur";
                var tibiaLink = prefix + "tibia";
                links.Add(new UrdfLink(coxaLink, [LegSegMesh(coxa, 0.014)], r: 1, g: 0.55, b: 0.15, a: 1));
                links.Add(new UrdfLink(femurLink, [LegSegMesh(femur, 0.014)], r: 1, g: 0.55, b: 0.15, a: 1));
                links.Add(new UrdfLink(tibiaLink, [LegSegMesh(tibia, 0.012)], r: 1, g: 0.55, b: 0.15, a: 1));
                joints.Add(new UrdfJoint(prefix + "hip", "revolute", mount, coxaLink,
                    0, 0, 0, 0, 0, 1, -Math.PI, Math.PI));
                joints.Add(new UrdfJoint(prefix + "femur", "revolute", coxaLink, femurLink,
                    coxa, 0, 0, 0, 1, 0, -Math.PI, Math.PI));
                joints.Add(new UrdfJoint(prefix + "tibia", "revolute", femurLink, tibiaLink,
                    femur, 0, 0, 0, 1, 0, -Math.PI, Math.PI));
            }
            else if (def.Chain is { } chain)
            {
                // Visual stick only — TreeFK poses via Assemble tree, not this URDF for numerical legs.
                _ = chain;
                _ = bodyR;
            }
        }

        if (!RobotDescription.TryAssemble(mechanism.ModelName, links, joints, tipLink: mechanism.TipLinkName,
                out var desc, out var diag, homeQ: null) || desc is null)
            throw new InvalidOperationException(string.Join("; ", diag.Errors));

        return desc;
    }

    private static double EstimateBodyR(IReadOnlyList<Frame> hips)
    {
        var r = 0.06;
        foreach (var h in hips)
        {
            var d = Math.Sqrt(h.X * h.X + h.Y * h.Y);
            if (d > r) r = d;
        }
        return r;
    }

    /// <summary>
    /// Leg segment along Motus link +X as a mesh (not Box — link length is +X, Box would be XYZ extents).
    /// </summary>
    private static UrdfGeometry LegSegMesh(double length, double diameter)
    {
        var r = diameter * 0.5;
        // ponytail: 8-corner box 0→length on +X.
        double[][] verts =
        [
            [0, -r, -r], [length, -r, -r], [length, r, -r], [0, r, -r],
            [0, -r, r], [length, -r, r], [length, r, r], [0, r, r],
        ];
        int[] indices =
        [
            0, 1, 2, 0, 2, 3,
            4, 6, 5, 4, 7, 6,
            0, 4, 5, 0, 5, 1,
            2, 6, 7, 2, 7, 3,
            0, 3, 7, 0, 7, 4,
            1, 5, 6, 1, 6, 2,
        ];
        return UrdfGeometry.Mesh(verts, indices);
    }

    /// <summary>Flat N-gon plate through hip XY at body Z.</summary>
    private static UrdfGeometry NGonPlateGeometry(IReadOnlyList<Frame> hips, double thickness, double bodyZ)
    {
        var n = hips.Count;
        var verts = new List<double[]>(Math.Max(n, 3) * 2);
        var hz = thickness * 0.5;
        for (var i = 0; i < n; i++)
        {
            verts.Add([hips[i].X, hips[i].Y, hz]);
            verts.Add([hips[i].X, hips[i].Y, -hz]);
        }

        var indices = new List<int>();
        for (var i = 1; i < n - 1; i++)
        {
            indices.Add(0); indices.Add(i * 2); indices.Add((i + 1) * 2);
            indices.Add(1); indices.Add((i + 1) * 2 + 1); indices.Add(i * 2 + 1);
        }
        for (var i = 0; i < n; i++)
        {
            var j = (i + 1) % n;
            var t0 = i * 2; var b0 = t0 + 1;
            var t1 = j * 2; var b1 = t1 + 1;
            indices.Add(t0); indices.Add(t1); indices.Add(b1);
            indices.Add(t0); indices.Add(b1); indices.Add(b0);
        }

        return UrdfGeometry.Mesh(verts, indices, origin: new Frame(0, 0, bodyZ));
    }

    internal static List<JointLimit> LimitsAllDrivers(KinematicTree tree)
    {
        var limits = new List<JointLimit>(tree.DriverCount);
        for (var di = 0; di < tree.DriverCount; di++)
        {
            var j = tree.Joints[tree.DriverJointIndices[di]];
            var vel = j.Velocity ?? Math.PI;
            limits.Add(new JointLimit(j.Lower, j.Upper, vel, vel * 2));
        }
        return limits;
    }

    internal static string[] DriverNames(KinematicTree tree)
    {
        var names = new string[tree.DriverCount];
        for (var di = 0; di < tree.DriverCount; di++)
            names[di] = tree.Joints[tree.DriverJointIndices[di]].Name;
        return names;
    }

    internal static List<JointLimit> LimitsAlongTip(KinematicTree tree, IReadOnlyList<string> tipJointNames)
    {
        var byName = new Dictionary<string, KinematicJoint>(StringComparer.OrdinalIgnoreCase);
        foreach (var j in tree.Joints)
            byName[j.Name] = j;

        var limits = new List<JointLimit>(tipJointNames.Count);
        foreach (var name in tipJointNames)
        {
            if (!byName.TryGetValue(name, out var j))
                throw new InvalidOperationException($"Tip joint '{name}' missing.");
            var vel = j.Velocity ?? Math.PI;
            limits.Add(new JointLimit(j.Lower, j.Upper, vel, vel * 2));
        }
        return limits;
    }
}


/// <summary>Legged viewport meshes: N-gon body from hip frames, orange 3R sticks, ground-contact rings.</summary>
internal static class WalkingHexPreview
{
    public readonly record struct Result(
        IReadOnlyList<Mesh> Meshes,
        IReadOnlyList<Color> Colors,
        IReadOnlyList<Line> Wires,
        IReadOnlyList<Circle> ContactCircles,
        Curve? SupportPolygon);

    public static Result Build(
        LeggedMechanism mechanism, double[] q, Frame? baseFrame = null,
        LeggedGait.TerrainHeight? terrain = null)
    {
        var meshes = new List<Mesh>();
        var colors = new List<Color>();
        var wires = new List<Line>();
        var contacts = new List<Circle>();
        var feet = new List<Point3d>();

        var bodyColor = Color.FromArgb(200, 255, 105, 180);
        var legColor = Color.FromArgb(220, 255, 140, 40);
        var jointColor = Color.FromArgb(230, 255, 160, 70);
        var contactColor = Color.FromArgb(180, 80, 200, 120);

        var n = mechanism.LegCount;
        var bodyPts = new Point3d[n];
        for (var i = 0; i < n; i++)
        {
            var hip = mechanism.HipBody(i);
            bodyPts[i] = new Point3d(hip.X, hip.Y, hip.Z);
        }
        if (NGonSlab(bodyPts, 0.02) is { } bodyMesh)
        {
            meshes.Add(bodyMesh);
            colors.Add(bodyColor);
        }

        const double segR = 0.007;
        const double jointR = 0.010;
        for (var leg = 0; leg < n; leg++)
        {
            var def = mechanism.Legs[leg];
            if (def.Lengths3R is not { Count: 3 } L)
                continue;
            var off = mechanism.DriverOffsets[leg];
            if (off + 2 >= q.Length)
                continue;

            var hipV = mechanism.HipBody(leg);
            var hip = new Point3d(hipV.X, hipV.Y, hipV.Z);
            var coxa = L[0];
            var femur = L[1];
            var tibia = L[2];
            var coxaA = q[off];
            var femurA = q[off + 1];
            var tibiaA = q[off + 2];

            var coxaDir = new Vector3d(Math.Cos(coxaA), Math.Sin(coxaA), 0);
            var knee = hip + coxaDir * coxa;
            var up = Vector3d.ZAxis;
            var femurDir = coxaDir * Math.Cos(femurA) - up * Math.Sin(femurA);
            femurDir.Unitize();
            var ankle = knee + femurDir * femur;
            var tibiaDir = coxaDir * Math.Cos(femurA + tibiaA) - up * Math.Sin(femurA + tibiaA);
            tibiaDir.Unitize();
            var foot = ankle + tibiaDir * tibia;
            feet.Add(foot);

            AddSeg(meshes, colors, wires, hip, knee, segR, jointR, legColor, jointColor);
            AddSeg(meshes, colors, wires, knee, ankle, segR, jointR, legColor, jointColor);
            AddSeg(meshes, colors, wires, ankle, foot, segR * 0.85, jointR * 0.85, legColor, jointColor);
        }

        var worldFeet = new Point3d[feet.Count];
        for (var i = 0; i < feet.Count; i++)
            worldFeet[i] = feet[i];
        Transform? bodyXform = baseFrame is { } bf0 ? BodyWorldXform(bf0) : null;
        if (bodyXform is { } x0)
        {
            for (var i = 0; i < worldFeet.Length; i++)
                worldFeet[i].Transform(x0);
        }

        var bodyMeshCount = meshes.Count;
        for (var i = 0; i < worldFeet.Length; i++)
        {
            var foot = worldFeet[i];
            var expectedZ = terrain is not null
                ? terrain(foot.X, foot.Y)
                : (baseFrame?.Z ?? 0);
            if (!double.IsFinite(expectedZ) || Math.Abs(foot.Z - expectedZ) > LeggedContactPreview.GroundTolMeters)
                continue;

            var ground = new Point3d(foot.X, foot.Y, expectedZ);
            contacts.Add(new Circle(new Plane(ground, Vector3d.ZAxis), LeggedContactPreview.RingRadiusMeters));
            if (Mesh.CreateFromCylinder(
                    new Cylinder(new Circle(new Plane(ground, Vector3d.ZAxis), LeggedContactPreview.RingRadiusMeters), 0.004),
                    16, 1) is { } pad)
            {
                meshes.Add(pad);
                colors.Add(contactColor);
            }
        }

        Curve? support = null;
        if (feet.Count >= 3)
        {
            var poly = new Polyline(feet);
            poly.Add(feet[0]);
            support = poly.ToNurbsCurve();
        }

        if (bodyXform is { } xform)
        {
            for (var i = 0; i < bodyMeshCount; i++)
                meshes[i].Transform(xform);
            for (var i = 0; i < wires.Count; i++)
            {
                var a = wires[i].From;
                var b = wires[i].To;
                a.Transform(xform);
                b.Transform(xform);
                wires[i] = new Line(a, b);
            }
            if (support is not null)
                support.Transform(xform);
        }

        return new Result(meshes, colors, wires, contacts, support);
    }

    private static Transform BodyWorldXform(Frame baseFrame)
    {
        var m = Motus.Geometry.Transforms.FromFrame(baseFrame);
        return new Transform
        {
            M00 = m[0], M01 = m[1], M02 = m[2], M03 = m[3],
            M10 = m[4], M11 = m[5], M12 = m[6], M13 = m[7],
            M20 = m[8], M21 = m[9], M22 = m[10], M23 = m[11],
            M30 = 0, M31 = 0, M32 = 0, M33 = 1,
        };
    }

    private static void AddSeg(
        List<Mesh> meshes, List<Color> colors, List<Line> wires,
        Point3d a, Point3d b, double r, double jr, Color leg, Color joint)
    {
        wires.Add(new Line(a, b));
        var len = a.DistanceTo(b);
        if (len > 1e-9)
        {
            var dir = b - a;
            dir.Unitize();
            var plane = new Plane(a, dir);
            if (Mesh.CreateFromCylinder(new Cylinder(new Circle(plane, r), len), 10, 1) is { } cyl)
            {
                meshes.Add(cyl);
                colors.Add(leg);
            }
        }
        if (Mesh.CreateFromSphere(new Sphere(a, jr), 8, 6) is { } ja)
        {
            meshes.Add(ja);
            colors.Add(joint);
        }
    }

    private static Mesh? NGonSlab(Point3d[] ring, double thickness)
    {
        if (ring.Length < 3) return null;
        var c = Point3d.Origin;
        foreach (var p in ring) c += p;
        c /= ring.Length;
        var mesh = new Mesh();
        mesh.Vertices.Add(c);
        foreach (var p in ring) mesh.Vertices.Add(p);
        for (var i = 0; i < ring.Length; i++)
            mesh.Faces.AddFace(0, i + 1, i + 2 <= ring.Length ? i + 2 : 1);
        mesh.Normals.ComputeNormals();

        var top = mesh.DuplicateMesh();
        top.Transform(Transform.Translation(0, 0, thickness * 0.5));
        var bottom = mesh.DuplicateMesh();
        bottom.Transform(Transform.Translation(0, 0, -thickness * 0.5));
        bottom.Flip(true, true, true);
        var slab = new Mesh();
        slab.Append(top);
        slab.Append(bottom);
        slab.Normals.ComputeNormals();
        slab.Compact();
        return slab.IsValid ? slab : null;
    }
}
