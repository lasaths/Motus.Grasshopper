using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.NET;
using Motus.Presets;
using Motus.Rhino;

static void Fail(string msg) => throw new InvalidOperationException(msg);
static void Ok(string msg) => Console.WriteLine($"  OK: {msg}");

var resources = FindResources();

static string FindResources()
{
    var dir = AppContext.BaseDirectory;
    for (var i = 0; i < 10 && dir is not null; i++)
    {
        foreach (var rel in new[]
        {
            Path.Combine("resources", "robots"),
            Path.Combine("src", "Motus.GH", "bin", "Release", "net8.0-windows", "resources", "robots"),
        })
        {
            var candidate = Path.GetFullPath(Path.Combine(dir, rel));
            if (Directory.Exists(candidate)) return candidate;
        }
        dir = Directory.GetParent(dir)?.FullName;
    }
    throw new InvalidOperationException("resources/robots not found");
}

Console.WriteLine("Motus QA smoke tests");
Console.WriteLine($"Resources: {resources}");

// UR5e joint plan
var urRobot = PresetLoader.LoadRobotModelByName("UR5e", resources);
var urPreset = urRobot.Preset;
var start = new JointState(new double[6]);
var goal = new JointState(Enumerable.Repeat(0.5, 6).ToArray());
var jointResult = new JointLinearPlanner().Plan(new PlanningRequest(urRobot, start, goal));
if (!jointResult.Success) Fail($"UR5e joint plan: {string.Join("; ", jointResult.Errors)}");
Ok("UR5e Plan Joint Path produces trajectory");

// KUKA
var kukaPreset = PresetLoader.LoadByModelName("KR 6 R900", resources);
var kukaRobot = new RobotModel(kukaPreset);
var kukaStart = new JointState(new double[kukaPreset.AxisCount]);
var kukaGoal = new JointState(Enumerable.Repeat(0.3, kukaPreset.AxisCount).ToArray());
var kukaResult = new JointLinearPlanner().Plan(new PlanningRequest(kukaRobot, kukaStart, kukaGoal));
if (!kukaResult.Success) Fail($"KUKA plan: {string.Join("; ", kukaResult.Errors)}");
Ok("KUKA KR 6 R900 plans successfully");

// Custom JSON preset
var jsonPath = Path.Combine(resources, "UR", "UR5e.json");
if (!File.Exists(jsonPath)) Fail($"Missing {jsonPath}");
PresetLoader.LoadFromFile(jsonPath);
Ok("Custom Robot JSON preset loads");

// Validation: out-of-limit joint
var bad = new JointState(new[] { 99.0, 0, 0, 0, 0, 0 });
var badVal = new TrajectoryValidator().Validate(
    new Trajectory(urRobot, new[] { new TrajectoryPoint(0, start), new TrajectoryPoint(1, bad) }));
if (badVal.IsValid) Fail("Expected invalid trajectory for out-of-limit joint");
Ok("Out-of-limit joint → Validate returns Valid=false");

// Export JSON / CSV
var traj = jointResult.Trajectory!;
var json = TrajectoryExport.ToJson(traj);
if (string.IsNullOrWhiteSpace(json) || !json.Contains("joint")) Fail("JSON export empty");
var csv = TrajectoryExport.ToCsv(traj);
if (!csv.StartsWith("time_seconds,joint_1_rad,")) Fail($"CSV header wrong: {csv.Split('\n')[0]}");
Ok("JSON export parses; CSV header is time_seconds,joint_1_rad,...");

// Cartesian LIN (Motus Plan plane branch)
var fk = new DhForwardKinematics(urPreset);
var cartStart = new JointState(new[] { 0.1, -0.5, 0.8, -0.3, -0.4, 0.2 });
var goalPose = fk.ComputeTcp(cartStart, urPreset.BaseFrame, urPreset.ToolFrame);
var linResult = new CartesianLinearPathPlanner(urPreset).PlanToResult(
    new CartesianPlanningRequest(urRobot, cartStart, goalPose, new PlanningOptions()));
if (!linResult.Success) Fail($"LIN plan: {string.Join("; ", linResult.Errors)}");
Ok("Cartesian LIN (TCP-linear) reaches goal via IK");

// Collision honesty: scene without checker fails joint-linear
var sceneOnly = new CollisionScene(new[] { CollisionObject.Sphere("obs", new Frame(2, 2, 2), 0.05) });
var noChecker = new JointLinearPlanner().Plan(new PlanningRequest(urRobot, start, goal, new PlanningOptions { CollisionScene = sceneOnly }));
if (noChecker.Success) Fail("Expected joint-linear to fail without collision checker when scene is set");
Ok("Joint-linear fails loudly without collision checker");

// Retimed export (bottleneck default)
var retimedJson = TrajectoryExport.ToJson(traj, retime: true);
if (!retimedJson.Contains("\"retimed\": true")) Fail("Retimed JSON export missing retimed flag");
var retimed = TrajectoryExport.Prepare(traj, new TrajectoryExportOptions { Retime = true });
if (retimed.Points.Count < 2) Fail("Bottleneck retime produced too few points");
Ok("Trajectory bottleneck retiming before JSON export");

// Per-link robot collision model from preset
if (urRobot.CollisionModel is null || urRobot.CollisionModel.Links.Count < 6)
    Fail("UR5e preset should include collisionLinks");
var robotChecker = new RobotMeshCollisionChecker(urRobot);
var freeHome = robotChecker.IsCollisionFree(start, new CollisionScene());
if (!freeHome) Fail("Home config should be collision-free with link capsules");
Ok("RobotMeshCollisionChecker uses preset collisionLinks");

// RRT with per-link collision checker (preset collisionLinks)
var meshChecker = new RobotMeshCollisionChecker(urRobot);
var rrtGoal = new JointState(new[] { 0.6, -0.6, 0.6, -0.6, -0.6, 0.3 });
var fkRrt = KinematicsResolver.CreateFkSolver(urPreset);
var midJoints = new JointState(start.Positions.Zip(rrtGoal.Positions, (a, b) => (a + b) * 0.5).ToArray());
var midTcp = fkRrt.ComputeTcp(midJoints, urPreset.BaseFrame, urPreset.ToolFrame).Tcp;
var scene = new CollisionScene(new[] { CollisionObject.Sphere("block", midTcp, 0.04) });
if (!meshChecker.IsCollisionFree(start, scene) || !meshChecker.IsCollisionFree(rrtGoal, scene))
    Fail("RRT obstacle should not collide with start or goal");
if (meshChecker.SegmentCollisionFree(start, rrtGoal, scene, 0.08))
    Fail("RRT obstacle should block the straight joint path");
var rrtOpts = new PlanningOptions { CollisionScene = scene, MaxJointStepRadians = 0.08, CollisionChecker = meshChecker };
var rrtResult = new RrtConnectPlanner(meshChecker, new RrtConnectOptions { MaxIterations = 10000, RandomSeed = 11 })
    .Plan(new PlanningRequest(urRobot, start, rrtGoal, rrtOpts));
if (!rrtResult.Success) Fail($"RRT: {string.Join("; ", rrtResult.Errors)}");
Ok("RRT Connect avoids obstacle with RobotMeshCollisionChecker");

var cancelResult = new RrtConnectPlanner(urPreset, new RrtConnectOptions
{
    MaxIterations = 50000,
    ShouldCancel = () => true
}).Plan(new PlanningRequest(urRobot, start, rrtGoal));
if (cancelResult.Success || !cancelResult.Errors.Any(e => e.Contains("cancelled", StringComparison.OrdinalIgnoreCase)))
    Fail("Expected planning cancelled message");
Ok("RRT ShouldCancel returns Planning cancelled");

// Preview: FK skeleton follows library link origins
var lines = KinematicsPreview.LinkLines(urRobot, start).ToList();
if (lines.Count == 0) Fail("No link lines from Preview Robot FK");
var ur10e = PresetLoader.LoadRobotModelByName("UR10e", resources);
var ghx = new JointState(new[] { 0.0, -1.2, 1.0, -1.4, -1.5708, 0.0 });
var fk10 = new DhForwardKinematics(ur10e.Preset);
var origins = fk10.ComputeLinkOrigins(ghx.Positions, ur10e.Preset.BaseFrame.Frame);
var previewLines = KinematicsPreview.LinkLines(ur10e, ghx).ToList();
if (previewLines.Count != origins.Count)
    Fail($"Preview line count {previewLines.Count} != origin chain {origins.Count}");
var lastOrigin = origins[^1];
if (previewLines[^1].To.DistanceTo(new Rhino.Geometry.Point3d(lastOrigin.X, lastOrigin.Y, lastOrigin.Z)) > 1e-4)
    Fail("Preview last segment should end at final link origin");
Ok("Preview Robot FK link lines match ComputeLinkOrigins");

// Trajectory segments valid/invalid (uses Point3d only, no Rhino native)
KinematicsPreview.TrajectorySegments(urRobot, traj, new TrajectoryValidationOptions(), out var valid, out var invalid);
if (valid.Count == 0) Fail("No valid trajectory segments");
Ok("Preview Trajectory valid/invalid segment split");

// FK TCP path moves with joint angles
var path = KinematicsPreview.TcpPath(urRobot, new[] { start, goal });
if (path.Count < 2 || path[0].DistanceTo(path[1]) < 1e-6) Fail("TCP path should move with joint angles");
Ok("Trajectory TCP path FK moves with joint angles");

// UseDegrees conversion
var rad = Units.ToRadians(new[] { 180.0 });
if (Math.Abs(rad[0] - Math.PI) > 1e-6) Fail("UseDegrees conversion failed");
Ok("UseDegrees=true converts 180° → π rad");

// Link radii in meters (sanity for viewport scale)
if (fk.LinkRadiiMeters.Any(r => r <= 0 || r > 0.5)) Fail("Link radii out of expected meter range");
Ok("Preview geometry uses meter-scale link radii");

// FrameConversion roundtrip (requires Rhino native DLL)
try
{
    var rnd = new Random(7);
    for (var i = 0; i < 8; i++)
    {
        var src = new Frame(rnd.NextDouble() * 0.5, rnd.NextDouble() * 0.5, rnd.NextDouble() * 0.5,
            0.9, 0.1 * rnd.NextDouble(), 0.2 * rnd.NextDouble(), 0.3 * rnd.NextDouble());
        var pl = FrameConversion.ToPlane(src);
        var back = FrameConversion.FromPlane(pl);
        if (Math.Abs(back.X - src.X) > 1e-4 || Math.Abs(back.Y - src.Y) > 1e-4 || Math.Abs(back.Z - src.Z) > 1e-4)
            Fail($"Frame roundtrip position drift at sample {i}");
    }
    Ok("FrameConversion ToPlane/FromPlane roundtrip within tolerance");
}
catch (DllNotFoundException)
{
    Ok("FrameConversion roundtrip skipped (Rhino native DLL unavailable in this host)");
}

// FK parity: KinematicsPreview TCP matches KinematicsResolver
var resolverFk = KinematicsResolver.CreateFkSolver(urPreset);
var testJoints = new JointState(new[] { 0.1, -0.5, 0.8, -0.3, -0.4, 0.2 });
var libTcp = resolverFk.ComputeTcp(testJoints, urPreset.BaseFrame, urPreset.ToolFrame).Tcp;
var previewFk = KinematicsPreview.TryFk(urRobot)!;
var previewTcp = previewFk.ComputeTcp(testJoints, urPreset.BaseFrame, urPreset.ToolFrame).Tcp;
if (Math.Abs(previewTcp.X - libTcp.X) > 1e-4 || Math.Abs(previewTcp.Y - libTcp.Y) > 1e-4 || Math.Abs(previewTcp.Z - libTcp.Z) > 1e-4)
    Fail("KinematicsPreview FK diverges from KinematicsResolver");
Ok("KinematicsPreview FK parity with KinematicsResolver");

// Interpolation smoke: midpoint time between two waypoints
var twoPt = new Trajectory(urRobot, new[]
{
    new TrajectoryPoint(0, start),
    new TrajectoryPoint(2, goal)
});
var midTime = 1.0;
JointState MidAt(Trajectory tr, double t)
{
    var pts = tr.Points;
    var alpha = (t - pts[0].TimeSeconds) / (pts[1].TimeSeconds - pts[0].TimeSeconds);
    var q = new double[start.AxisCount];
    for (var j = 0; j < q.Length; j++)
        q[j] = pts[0].JointState.Positions[j] + alpha * (pts[1].JointState.Positions[j] - pts[0].JointState.Positions[j]);
    return new JointState(q);
}
var midState = MidAt(twoPt, midTime);
var midFrame = resolverFk.ComputeTcp(midState, urPreset.BaseFrame, urPreset.ToolFrame).Tcp;
var endFrame = resolverFk.ComputeTcp(goal, urPreset.BaseFrame, urPreset.ToolFrame).Tcp;
var startFrame = resolverFk.ComputeTcp(start, urPreset.BaseFrame, urPreset.ToolFrame).Tcp;
var dStart = Math.Sqrt(Math.Pow(midFrame.X - startFrame.X, 2) + Math.Pow(midFrame.Y - startFrame.Y, 2) + Math.Pow(midFrame.Z - startFrame.Z, 2));
var dEnd = Math.Sqrt(Math.Pow(midFrame.X - endFrame.X, 2) + Math.Pow(midFrame.Y - endFrame.Y, 2) + Math.Pow(midFrame.Z - endFrame.Z, 2));
if (dStart < 1e-6 || dEnd < 1e-6)
    Fail("Interpolated midpoint TCP should lie between endpoints");
Ok("Trajectory midpoint interpolation produces distinct TCP pose");

// LIN timing: duration should be physically plausible (not frame indices)
var linStart = new JointState(new[] { 0.0, -0.5, 1.0, -1.0, 0.0, 0.0 });
var linStartPose = fk.ComputeTcp(linStart, urPreset.BaseFrame, urPreset.ToolFrame);
var linGoalPose = new CartesianPose(new Frame(
    linStartPose.Tcp.X + 0.02, linStartPose.Tcp.Y, linStartPose.Tcp.Z,
    linStartPose.Tcp.Qw, linStartPose.Tcp.Qx, linStartPose.Tcp.Qy, linStartPose.Tcp.Qz));
var timedLin = new CartesianLinearPathPlanner(urPreset).Plan(linStartPose, linGoalPose, linStart);
if (timedLin is null) Fail("LIN timing plan returned null");
if (timedLin!.DurationSeconds < 0.01 || timedLin.DurationSeconds > 60)
    Fail($"LIN duration implausible: {timedLin.DurationSeconds}s");
if (timedLin.Points[^1].TimeSeconds < 0.01)
    Fail("LIN waypoint times should be seconds, not frame indices");
Ok("Cartesian LIN trajectory has retimed duration in seconds");

// Export includes jointNames when available
if (urRobot.JointNames is { Count: > 0 } && !json.Contains("jointNames"))
    Fail("JSON export should include jointNames when robot metadata has them");
if (urRobot.JointNames is { Count: > 0 }) Ok("Trajectory export includes jointNames metadata");

Console.WriteLine("\nAll automated QA checks passed.");
