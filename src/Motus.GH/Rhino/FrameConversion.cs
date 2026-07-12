using Motus.Core;
using Motus.Geometry;
using Rhino.Geometry;

namespace Motus.GH.Rhino;

/// <summary>
/// Motus FK frames use local X as the tool approach axis; Rhino planes use Z as the normal.
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
}
