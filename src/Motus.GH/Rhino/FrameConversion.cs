using Motus.Core;
using Motus.Geometry;
using Rhino.Geometry;

namespace Motus.GH.Rhino;

/// <summary>
/// Motus FK frames use local X as the tool approach axis; Rhino planes use Z as the normal.
/// Stewart platform poses use axis-aligned plate mapping (<see cref="FromPlanePlate"/>) — Motus.NET
/// Stewart IK expects Motus XYZ ≈ world/plate axes, not the serial tool-approach remap.
/// </summary>
public static class FrameConversion
{
    public static Plane ToPlane(Frame frame)
    {
        var m = Transforms.FromFrame(frame);
        var origin = new Point3d(frame.X, frame.Y, frame.Z);
        // Matrix columns: 0 = tool approach (Motus X), 1 = Motus Y, 2 = Motus Z.
        var xAxis = new Vector3d(m[1], m[5], m[9]);
        var yAxis = new Vector3d(m[2], m[6], m[10]);
        if (!xAxis.Unitize() || !yAxis.Unitize())
            return new Plane(origin, Vector3d.ZAxis);
        return new Plane(origin, xAxis, yAxis);
    }

    /// <summary>Serial tool TCP: Rhino Z (normal) → Motus X (approach).</summary>
    public static Frame FromPlane(Plane plane)
    {
        if (!plane.IsValid)
            return new Frame(plane.OriginX, plane.OriginY, plane.OriginZ, 1, 0, 0, 0);

        var x = plane.XAxis;
        var y = plane.YAxis;
        var z = plane.ZAxis;
        if (!x.Unitize() || !y.Unitize() || !z.Unitize())
            return new Frame(plane.OriginX, plane.OriginY, plane.OriginZ, 1, 0, 0, 0);

        // Rhino Plane.Z → Motus local X (approach); Plane.X/Y → Motus Y/Z.
        var m = new double[]
        {
            z.X, x.X, y.X, plane.OriginX,
            z.Y, x.Y, y.Y, plane.OriginY,
            z.Z, x.Z, y.Z, plane.OriginZ,
            0, 0, 0, 1
        };
        return Transforms.ToFrame(m);
    }

    /// <summary>Stewart platform plate: Rhino X/Y/Z → Motus X/Y/Z (flat WorldXY ≡ Motus identity).</summary>
    public static Frame FromPlanePlate(Plane plane)
    {
        if (!plane.IsValid)
            return new Frame(plane.OriginX, plane.OriginY, plane.OriginZ, 1, 0, 0, 0);

        var x = plane.XAxis;
        var y = plane.YAxis;
        var z = plane.ZAxis;
        if (!x.Unitize() || !y.Unitize() || !z.Unitize())
            return new Frame(plane.OriginX, plane.OriginY, plane.OriginZ, 1, 0, 0, 0);

        var m = new double[]
        {
            x.X, y.X, z.X, plane.OriginX,
            x.Y, y.Y, z.Y, plane.OriginY,
            x.Z, y.Z, z.Z, plane.OriginZ,
            0, 0, 0, 1
        };
        return Transforms.ToFrame(m);
    }

    /// <summary>Inverse of <see cref="FromPlanePlate"/> for Stewart TCP display.</summary>
    public static Plane ToPlanePlate(Frame frame)
    {
        var m = Transforms.FromFrame(frame);
        var origin = new Point3d(frame.X, frame.Y, frame.Z);
        var xAxis = new Vector3d(m[0], m[4], m[8]);
        var yAxis = new Vector3d(m[1], m[5], m[9]);
        if (!xAxis.Unitize() || !yAxis.Unitize())
            return new Plane(origin, Vector3d.ZAxis);
        return new Plane(origin, xAxis, yAxis);
    }
}
