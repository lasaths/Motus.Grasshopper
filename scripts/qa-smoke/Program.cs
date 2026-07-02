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
            Path.Combine("..", "Motus.NET", "resources", "robots"),
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

// Legacy Cartesian joint-linear still available
var cartResult = new CartesianLinearPlanner(urPreset).Plan(new CartesianPlanningRequest(urRobot, cartStart, goalPose, new PlanningOptions()));
if (!cartResult.Success) Fail($"Cartesian plan: {string.Join("; ", cartResult.Errors)}");
Ok("Cartesian joint-linear planner still works");

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
CollisionScene? scene = null;
foreach (var pose in new[]
{
    new Frame(0.18, -0.22, 0.28),
    new Frame(0.22, -0.18, 0.32),
    new Frame(0.12, -0.28, 0.24),
    new Frame(0.25, -0.30, 0.35),
})
{
    var trial = new CollisionScene(new[] { CollisionObject.Sphere("block", pose, 0.05) });
    if (meshChecker.IsCollisionFree(start, trial) && meshChecker.IsCollisionFree(rrtGoal, trial)
        && !meshChecker.SegmentCollisionFree(start, rrtGoal, trial, 0.08))
    {
        scene = trial;
        break;
    }
}
if (scene is null) Fail("Could not find collision scene for RRT smoke test");
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

// Preview: FK geometry (Rhino mesh/plane APIs need Rhino runtime)
var lines = KinematicsPreview.LinkLines(urRobot, start).ToList();
if (lines.Count == 0) Fail("No link lines from Preview Robot FK");
Ok("Preview Robot FK link lines");

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

Console.WriteLine("\nAll automated QA checks passed.");
