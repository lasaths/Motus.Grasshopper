using Motus.Core;
using Rhino.Geometry;

namespace Motus.Rhino;

public static class FrameConversion
{
    public static Plane ToPlane(Frame frame)
    {
        var q = new Quaternion(frame.Qx, frame.Qy, frame.Qz, frame.Qw);
        q.GetRotation(out var angle, out var axis);
        var rot = Transform.Rotation(angle, axis, Point3d.Origin);
        var move = Transform.Translation(frame.X, frame.Y, frame.Z);
        var plane = Plane.WorldXY;
        plane.Transform(move * rot);
        return plane;
    }

    public static Frame FromPlane(Plane plane)
    {
        var xf = Transform.PlaneToPlane(Plane.WorldXY, plane);
        xf.GetQuaternion(out var q);
        return new Frame(plane.OriginX, plane.OriginY, plane.OriginZ, q.A, q.B, q.C, q.D);
    }
}
