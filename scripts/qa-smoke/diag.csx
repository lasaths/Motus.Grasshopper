using Motus.Core;
using Motus.Geometry;
using Motus.Presets;
using System.Reflection;

var resources = @"C:\Users\lasaths\GitHub\Motus.Grasshopper\scripts\qa-smoke\bin\Release\net8.0\resources\robots";
var robot = PresetLoader.LoadRobotModelByName("UR10e", resources);
var fk = new DhForwardKinematics(robot.Preset);
var joints = new JointState(new[] { 0.0, -1.2, 1.0, -1.4, -1.5708, 0.0 });
var baseF = robot.Preset.BaseFrame;
var origins = fk.ComputeLinkOrigins(joints.Positions, baseF.Frame);
Console.WriteLine($"origins count: {origins.Count}");
var prev = (baseF.Frame.X, baseF.Frame.Y, baseF.Frame.Z);
for (int i = 0; i < origins.Count; i++)
{
    var o = origins[i];
    var dx = o.X-prev.Item1; var dy = o.Y-prev.Item2; var dz = o.Z-prev.Item3;
    var len = Math.Sqrt(dx*dx+dy*dy+dz*dz);
    Console.WriteLine($"  link {i}: ({o.X:F3},{o.Y:F3},{o.Z:F3}) segLen={len:F3}");
    prev = (o.X, o.Y, o.Z);
}
var tcp = fk.ComputeTcp(joints, baseF, robot.Preset.ToolFrame).Tcp;
var last = origins[^1];
var tcpDist = Math.Sqrt((tcp.X-last.X)*(tcp.X-last.X)+(tcp.Y-last.Y)*(tcp.Y-last.Y)+(tcp.Z-last.Z)*(tcp.Z-last.Z));
Console.WriteLine($"TCP: ({tcp.X:F3},{tcp.Y:F3},{tcp.Z:F3}) distFromLastOrigin={tcpDist:F6}");
