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

public static class RobotPreview
{
    // ponytail: stick-figure TCP path from joint angles (no FK mesh yet)
    public static Polyline TcpPathFromJointStates(IEnumerable<JointState> states, double linkLength = 0.15)
    {
        var pts = new List<Point3d> { Point3d.Origin };
        var angle = 0.0;
        var pos = Point3d.Origin;
        foreach (var s in states)
        {
            if (s.Positions.Length > 0) angle += s.Positions[0];
            pos = new Point3d(pos.X + Math.Cos(angle) * linkLength, pos.Y + Math.Sin(angle) * linkLength, pos.Z);
            pts.Add(pos);
        }
        return new Polyline(pts);
    }

    public static IEnumerable<Line> StickLinks(JointState state, double linkLength = 0.12)
    {
        var lines = new List<Line>();
        var angle = 0.0;
        var prev = Point3d.Origin;
        for (var i = 0; i < state.Positions.Length; i++)
        {
            angle += state.Positions[i];
            var next = new Point3d(prev.X + Math.Cos(angle) * linkLength, prev.Y + Math.Sin(angle) * linkLength, prev.Z + (i % 2 == 0 ? linkLength * 0.2 : 0));
            lines.Add(new Line(prev, next));
            prev = next;
        }
        return lines;
    }
}
