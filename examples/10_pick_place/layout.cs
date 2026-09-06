// Grasshopper Script Instance
#region Usings
using System;
using System.Collections.Generic;
using Rhino.Geometry;
using Grasshopper.Kernel;
#endregion

public class Script_Instance : GH_ScriptInstance
{
    private void RunScript(
		Point3d Tower,
		Point3d Columns,
		double Levels,
		double HalfX,
		double HalfY,
		double HalfZ,
		double ColPitch,
		double ColumnsN,
		ref object Brick,
		ref object Grasp,
		ref object Place)
    {
        // 5×4 rotating tower destack (top-first) → 5 columns × 4.
        // Rhino Z → Motus X (approach). Robotiq mesh fingers along −approach,
        // so fingers-down needs Motus X = world +Z = Rhino plane Z +Z.
        if (HalfX <= 0) HalfX = 0.04;
        if (HalfY <= 0) HalfY = 0.02;
        if (HalfZ <= 0) HalfZ = 0.01;
        if (ColPitch <= 0) ColPitch = 0.10;
        int levels = Math.Max(1, (int)Math.Round(Levels <= 0 ? 5 : Levels));
        int columnsN = Math.Max(1, (int)Math.Round(ColumnsN <= 0 ? 5 : ColumnsN));
        const int perLevel = 4;
        int total = levels * perLevel;
        if (columnsN * perLevel < total)
        {
            Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Error,
                $"Need ColumnsN×4 >= Levels×4 (have {columnsN * 4}, need {total}).");
            return;
        }

        var bricks = new List<Plane>(total);
        var grasps = new List<Plane>(total);
        var places = new List<Plane>(total);

        var offs = new[]
        {
            (-HalfX, -HalfY), (HalfX, -HalfY),
            (-HalfX, HalfY), (HalfX, HalfY)
        };

        Plane FingersDownTcp(Point3d origin, Vector3d shortAxis)
        {
            var x = Vector3d.CrossProduct(shortAxis, Vector3d.ZAxis);
            var up = Vector3d.ZAxis;
            var y = Vector3d.CrossProduct(up, x);
            if (!x.Unitize() || !y.Unitize())
                return new Plane(origin, Vector3d.ZAxis);
            // Robotiq closes along TCP Z = Rhino plane Y. Align it with the short edge.
            // Both grasp and place use this roll; Z stays world +Z.
            return new Plane(origin, -x, -y);
        }

        for (int level = levels - 1; level >= 0; level--)
        {
            double yaw = level * (Math.PI / 2.0);
            double cos = Math.Cos(yaw);
            double sin = Math.Sin(yaw);
            double z = Tower.Z + HalfZ + level * (2.0 * HalfZ);
            foreach (var (lx, ly) in offs)
            {
                double wx = Tower.X + lx * cos - ly * sin;
                double wy = Tower.Y + lx * sin + ly * cos;
                var brick = new Plane(
                    new Point3d(wx, wy, z),
                    new Vector3d(cos, sin, 0),
                    new Vector3d(-sin, cos, 0));
                grasps.Add(FingersDownTcp(brick.Origin, brick.YAxis));
                bricks.Add(brick);
            }
        }

        for (int i = 0; i < total; i++)
        {
            int col = i / perLevel;
            int height = i % perLevel;
            double z = Columns.Z + HalfZ + height * (2.0 * HalfZ);
            var origin = new Point3d(Columns.X, Columns.Y - col * ColPitch, z);
            places.Add(FingersDownTcp(origin, Vector3d.YAxis));
        }

        Brick = bricks;
        Grasp = grasps;
        Place = places;
    }
}
