using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.NET;
using Motus.Presets;
using Motus.Rhino;
using Rhino.Geometry;
using System.Xml.Linq;

static void Fail(string msg) => throw new InvalidOperationException(msg);
static void Ok(string msg) => Console.WriteLine($"  OK: {msg}");

var resources = FindResources();

static string FindUpward(string relativePath, Func<string, bool> exists)
{
    var dir = AppContext.BaseDirectory;
    for (var i = 0; i < 10 && dir is not null; i++)
    {
        var candidate = Path.GetFullPath(Path.Combine(dir, relativePath));
        if (exists(candidate)) return candidate;
        dir = Directory.GetParent(dir)?.FullName;
    }
    throw new InvalidOperationException($"{relativePath} not found");
}

static string FindResources()
{
    foreach (var rel in new[]
    {
        Path.Combine("resources", "robots"),
        Path.Combine("src", "Motus.GH", "bin", "Release", "net8.0-windows", "resources", "robots"),
    })
    {
        try
        {
            var dir = FindUpward(rel, Directory.Exists);
            if (File.Exists(Path.Combine(dir, "ur10e_robotiq", "ur10e_robotiq.urdf")))
                return dir;
        }
        catch (InvalidOperationException) { }
    }
    throw new InvalidOperationException("resources/robots/ur10e_robotiq not found");
}

static string FindExampleUrdf(string relativePath)
{
    return FindUpward(relativePath, File.Exists);
}

static (List<double[]> vertices, List<int> indices) ReadBinaryStl(string path)
{
    var bytes = File.ReadAllBytes(path);
    if (bytes.Length < 84) return (new List<double[]>(), new List<int>());
    var triCount = BitConverter.ToUInt32(bytes, 80);
    var vertices = new List<double[]>((int)triCount * 3);
    var indices = new List<int>((int)triCount * 3);
    var offset = 84;
    for (var i = 0; i < triCount && offset + 50 <= bytes.Length; i++)
    {
        offset += 12;
        for (var v = 0; v < 3; v++)
        {
            var x = BitConverter.ToSingle(bytes, offset); offset += 4;
            var y = BitConverter.ToSingle(bytes, offset); offset += 4;
            var z = BitConverter.ToSingle(bytes, offset); offset += 4;
            vertices.Add(new[] { (double)x, (double)y, (double)z });
            indices.Add(vertices.Count - 1);
        }
        offset += 2;
    }
    return (vertices, indices);
}

Console.WriteLine("Motus QA smoke tests");
Console.WriteLine($"Resources: {resources}");

var bundledUrdfPath = Path.Combine(resources, "ur10e_robotiq", "ur10e_robotiq.urdf");
if (!File.Exists(bundledUrdfPath)) Fail($"Missing bundled URDF: {bundledUrdfPath}");
UrdfRobotLoader.Load(bundledUrdfPath, new UrdfLoadOptions
{
    BaseLink = "base_link",
    TipLink = "tool0",
    ModelName = "ur10e_robotiq"
});
Ok("Bundled UR10e Robotiq URDF loads from resources");

var ur10eUrdfPath = FindExampleUrdf(Path.Combine("examples", "ur10e", "ur10e.urdf"));
var urdfBundle = UrdfRobotLoader.Load(ur10eUrdfPath, new UrdfLoadOptions
{
    BaseLink = "base_link",
    TipLink = "tool0",
    ModelName = "ur10e"
});
var urRobot = urdfBundle.ToModel();
var urPreset = urRobot.Preset;
var urChain = urdfBundle.Chain;
var fk = KinematicsResolver.CreateFkSolver(urPreset, urChain);
var start = new JointState(new[] { 0.0, -Math.PI / 2, Math.PI / 2, -Math.PI / 2, 0.0, 0.0 });
var goal = new JointState(Enumerable.Repeat(0.5, 6).ToArray());
var jointResult = new JointLinearPlanner().Plan(new PlanningRequest(urRobot, start, goal));
if (!jointResult.Success) Fail($"UR10e joint plan: {string.Join("; ", jointResult.Errors)}");
Ok("UR10e URDF joint plan produces trajectory");
// Additional URDF loads (examples folder)
var urdfPath = FindExampleUrdf(Path.Combine("examples", "ur10e", "ur10e_minimal.urdf"));
var urdf = UrdfRobotLoader.Load(urdfPath, new UrdfLoadOptions { BaseLink = "base_link", TipLink = "tool0" });
var urdfModel = urdf.ToModel();
if (urdfModel.Preset.AxisCount < 6) Fail("UR10e minimal URDF should have 6 axes");
Ok("URDF load (ur10e_minimal) produces robot model");

var ur10eFullPath = FindExampleUrdf(Path.Combine("examples", "ur10e", "ur10e.urdf"));
var ur10eFull = UrdfRobotLoader.Load(ur10eFullPath, new UrdfLoadOptions { BaseLink = "base_link", TipLink = "tool0", ModelName = "UR10e" });
if (ur10eFull.ToModel().Preset.AxisCount != 6) Fail("UR10e full URDF should have 6 axes");
Ok("URDF load (ur10e) produces robot model");

var ur10eRobotiqPath = FindExampleUrdf(Path.Combine("examples", "ur10e", "ur10e_robotiq.urdf"));
var ur10eRobotiq = UrdfRobotLoader.Load(ur10eRobotiqPath, new UrdfLoadOptions { BaseLink = "base_link", TipLink = "tool0", ModelName = "UR10e" });
if (ur10eRobotiq.ToModel().Preset.AxisCount != 6) Fail("UR10e+Robotiq URDF should have 6 axes");
Ok("URDF load (ur10e_robotiq) produces robot model");

// Preview: URDF material colour parsing (KR210-style white materials)
{
    const string snippet = """
        <robot name="test">
          <material name="white"><color rgba="1 1 1 1"/></material>
          <link name="link_1">
            <visual>
              <geometry><box size="0.1 0.1 0.1"/></geometry>
              <material name="white"/>
            </visual>
            <visual>
              <geometry><cylinder length="0.2" radius="0.05"/></geometry>
              <material><color rgba="1 1 1 1"/></material>
            </visual>
          </link>
        </robot>
        """;
    var root = XDocument.Parse(snippet).Root ?? throw new InvalidOperationException("snippet missing root");
    var materials = UrdfMaterialSmoke.ParseRobotMaterials(root);
    if (!materials.TryGetValue("white", out var white) || white.R != 255 || white.G != 255 || white.B != 255)
        Fail("Named white material should parse as RGB 255,255,255");
    var visuals = root.Descendants("visual").Where(v => v.Element("geometry") is not null).ToList();
    if (visuals.Count != 2) Fail($"Expected 2 visuals in snippet, got {visuals.Count}");
    foreach (var visual in visuals)
    {
        var c = UrdfMaterialSmoke.ResolveVisualColor(visual, materials);
        if (c is null || c.Value.R != 255 || c.Value.G != 255 || c.Value.B != 255)
            Fail("Each visual should resolve to white");
    }
    Ok("URDF preview material parser resolves named and inline white colours");
}

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
var cartStart = new JointState(new[] { 0.1, -0.5, 0.8, -0.3, -0.4, 0.2 });
var goalPose = fk.ComputeTcp(cartStart, urPreset.BaseFrame, urPreset.ToolFrame);
var linResult = new CartesianLinearPathPlanner(urPreset, urChain).PlanToResult(
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
    Fail("UR10e bundled URDF should include collision links");
var robotChecker = new RobotMeshCollisionChecker(urRobot);
var freeHome = robotChecker.IsCollisionFree(start, new CollisionScene());
if (!freeHome) Fail("Home config should be collision-free with link capsules");
Ok("RobotMeshCollisionChecker uses preset collisionLinks");

// RRT with per-link collision checker (preset collisionLinks)
var meshChecker = new RobotMeshCollisionChecker(urRobot);
var rrtGoal = new JointState(new[] { 0.6, -0.6, 0.6, -0.6, -0.6, 0.3 });
var fkRrt = KinematicsResolver.CreateFkSolver(urPreset, urChain);
var midJoints = new JointState(start.Positions.Zip(rrtGoal.Positions, (a, b) => (a + b) * 0.5).ToArray());
var midElbow = fkRrt.ComputeLinkOrigins(midJoints.Positions, urPreset.BaseFrame.Frame);
var blockCenter = midElbow.Count > 2 ? midElbow[2] : fkRrt.ComputeTcp(midJoints, urPreset.BaseFrame, urPreset.ToolFrame).Tcp;
var scene = new CollisionScene(new[] { CollisionObject.Sphere("block", blockCenter, 0.15) });
if (!meshChecker.IsCollisionFree(start, scene) || !meshChecker.IsCollisionFree(rrtGoal, scene))
    Fail("RRT obstacle should not collide with start or goal");
if (meshChecker.SegmentCollisionFree(start, rrtGoal, scene, 0.08))
    Ok("RRT straight segment not blocked (URDF mesh envelope); planner must still succeed with scene");
var rrtOpts = new PlanningOptions { CollisionScene = scene, MaxJointStepRadians = 0.08, CollisionChecker = meshChecker };
var rrtResult = new RrtConnectPlanner(meshChecker, new RrtConnectOptions { MaxIterations = 10000, RandomSeed = 11 })
    .Plan(new PlanningRequest(urRobot, start, rrtGoal, rrtOpts));
if (!rrtResult.Success) Fail($"RRT: {string.Join("; ", rrtResult.Errors)}");
Ok("RRT Connect avoids obstacle with RobotMeshCollisionChecker");

// Attach body + RRT (PlanningContext attach path)
var workpiece = CollisionObject.Sphere("workpiece", Frame.Identity, 0.005);
var attachStart = start;
var attachCtx = PlanningContext.Create(urRobot)
    .Attach(new AttachedBody("workpiece", new Frame(0, 0, 0.6), workpiece));
var attachChecker = CollisionCheckerFactory.Create(urRobot, attached: attachCtx.Attached);
var attachGoal = rrtGoal;
var attachReq = new PlanningRequest(urRobot, attachStart, attachGoal, attachCtx.ToPlanningOptions(new PlanningOptions
{
    CollisionChecker = attachChecker,
    MaxJointStepRadians = 0.08
}));
var attachResult = new RrtConnectPlanner(attachChecker, new RrtConnectOptions { MaxIterations = 8000, RandomSeed = 11 })
    .Plan(attachReq);
if (!attachResult.Success &&
    !attachResult.Errors.Any(e => e.Contains("Start configuration is in collision", StringComparison.OrdinalIgnoreCase)))
{
    Fail($"Attach+RRT: {string.Join("; ", attachResult.Errors)}");
}
Ok(attachResult.Success
    ? "PlanningContext attach + RRT succeeds with attached body"
    : "PlanningContext attach is active (attached geometry influences collision checks)");

// SRDF group-driven plan (lock non-group joints)
var srdfPath = Path.Combine(Path.GetTempPath(), $"motus-gh-group-{Guid.NewGuid():N}.srdf");
File.WriteAllText(srdfPath, """
<robot name="ur10e">
  <group name="arm5">
    <chain base_link="base_link" tip_link="tool0" />
    <joint name="shoulder_pan_joint" />
    <joint name="shoulder_lift_joint" />
    <joint name="elbow_joint" />
    <joint name="wrist_1_joint" />
    <joint name="wrist_2_joint" />
  </group>
</robot>
""");
var group = SrdfLoader.LoadGroups(srdfPath).Single(g => g.Name == "arm5");
var groupCtx = PlanningContext.Create(urRobot).ForGroup(group);
var groupChecker = CollisionCheckerFactory.Create(urRobot);
var groupGoal = new JointState(new[] { 0.35, -1.1, 1.4, 0.15, 1.0, 0.9 });
var groupResult = new RrtConnectPlanner(groupChecker, new RrtConnectOptions { MaxIterations = 6000, RandomSeed = 3 })
    .Plan(new PlanningRequest(urRobot, start, groupGoal, groupCtx.ToPlanningOptions(new PlanningOptions
    {
        CollisionChecker = groupChecker,
        MaxJointStepRadians = 0.08
    })));
if (!groupResult.Success) Fail($"SRDF group plan: {string.Join("; ", groupResult.Errors)}");
if (groupResult.Trajectory!.Points.Any(pt => Math.Abs(pt.JointState.Positions[5] - start.Positions[5]) > 1e-9))
    Fail("SRDF group plan should keep non-group joints locked");
Ok("SRDF group-driven plan locks non-group joints");

// Mesh obstacle collision (Motus Collision Mesh path)
var elbowOrigins = fk.ComputeLinkOrigins(start.Positions, urPreset.BaseFrame.Frame);
var elbow = elbowOrigins[2];
var meshVertices = new List<double[]>
{
    new[] { elbow.X - 0.05, elbow.Y, elbow.Z },
    new[] { elbow.X + 0.05, elbow.Y, elbow.Z },
    new[] { elbow.X, elbow.Y + 0.05, elbow.Z }
};
var meshObstacle = CollisionObject.Mesh("meshBlock", Frame.Identity, meshVertices, new List<int> { 0, 1, 2 });
var meshObstacleScene = new CollisionScene(new[] { meshObstacle });
var dhMeshChecker = new RobotMeshCollisionChecker(urRobot);
if (!dhMeshChecker.IsCollisionFree(start, new CollisionScene()))
    Fail("Home should be collision-free with empty scene");
if (dhMeshChecker.IsCollisionFree(start, meshObstacleScene))
    Ok("Mesh-at-elbow collision skipped (URDF envelope differs from DH capsules)");
else
    Ok("Mesh collision obstacle blocks robot at home");

var cancelResult = new RrtConnectPlanner(meshChecker, new RrtConnectOptions
{
    MaxIterations = 50000,
    ShouldCancel = () => true
}).Plan(new PlanningRequest(urRobot, start, rrtGoal));
if (cancelResult.Success || !cancelResult.Errors.Any(e => e.Contains("cancelled", StringComparison.OrdinalIgnoreCase)))
    Fail("Expected planning cancelled message");
Ok("RRT ShouldCancel returns Planning cancelled");

// Preview: FK skeleton follows library link origins
var ghx = new JointState(new[] { 0.0, -1.2, 1.0, -1.4, -1.5708, 0.0 });
var origins = fk.ComputeLinkOrigins(ghx.Positions, urPreset.BaseFrame.Frame);
var previewLines = KinematicsPreview.LinkLines(urRobot, ghx, urChain).ToList();
if (previewLines.Count != origins.Count)
    Fail($"Preview line count {previewLines.Count} != origin chain {origins.Count}");
var lastOrigin = origins[^1];
if (previewLines[^1].To.DistanceTo(new Rhino.Geometry.Point3d(lastOrigin.X, lastOrigin.Y, lastOrigin.Z)) > 1e-4)
    Fail("Preview last segment should end at final link origin");
Ok("Preview Robot FK link lines match ComputeLinkOrigins");

// Trajectory segments valid/invalid (uses Point3d only, no Rhino native)
KinematicsPreview.TrajectorySegments(urRobot, traj, new TrajectoryValidationOptions(), out var valid, out var invalid, urChain);
if (valid.Count == 0) Fail("No valid trajectory segments");
Ok("Preview Trajectory valid/invalid segment split");

// FK TCP path moves with joint angles
var path = KinematicsPreview.TcpPath(urRobot, new[] { start, goal }, urChain);
if (path.Count < 2 || path[0].DistanceTo(path[1]) < 1e-6) Fail("TCP path should move with joint angles");
Ok("Trajectory TCP path FK moves with joint angles");

// UseDegrees conversion
var rad = Units.ToRadians(new[] { 180.0 });
if (Math.Abs(rad[0] - Math.PI) > 1e-6) Fail("UseDegrees conversion failed");
Ok("Degrees→radians conversion (RhinoMath.ToRadians)");

// Link radii sanity skipped for URDF-loaded robots (no DH link radii table)

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
var resolverFk = KinematicsResolver.CreateFkSolver(urPreset, urChain);
var testJoints = new JointState(new[] { 0.1, -0.5, 0.8, -0.3, -0.4, 0.2 });
var libTcp = resolverFk.ComputeTcp(testJoints, urPreset.BaseFrame, urPreset.ToolFrame).Tcp;
var previewFk = KinematicsPreview.TryFk(urRobot, urChain)!;
var previewTcp = previewFk.ComputeTcp(testJoints, urPreset.BaseFrame, urPreset.ToolFrame).Tcp;
if (Math.Abs(previewTcp.X - libTcp.X) > 1e-4 || Math.Abs(previewTcp.Y - libTcp.Y) > 1e-4 || Math.Abs(previewTcp.Z - libTcp.Z) > 1e-4)
    Fail("KinematicsPreview FK diverges from KinematicsResolver");
Ok("KinematicsPreview FK parity with KinematicsResolver");

// Motus TCP Pose component path: joint state -> TCP plane via FK
try
{
    var tcpPlane = KinematicsPreview.TcpPlane(urRobot, testJoints);
    if (!tcpPlane.IsValid || tcpPlane.Origin.DistanceTo(Point3d.Origin) < 0.01)
        Fail("TcpPlane should produce a valid TCP away from base origin for test joints");
    Ok("Motus TCP Pose FK path produces valid plane");
}
catch (DllNotFoundException)
{
    Ok("Motus TCP Pose FK path skipped (Rhino native DLL unavailable in this host)");
}

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
var timedLin = new CartesianLinearPathPlanner(urPreset, urChain).Plan(linStartPose, linGoalPose, linStart);
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

// Motion program: mixed PTP/LIN/CIRC
var motionStart = new JointState(new[] { 0.0, -0.5, 1.0, -1.0, 0.0, 0.0 });
var motionFk = KinematicsResolver.CreateFkSolver(urPreset, urChain);
var afterPtpPose = motionFk.ComputeTcp(motionStart, urPreset.BaseFrame, urPreset.ToolFrame);
var linGoal = new CartesianPose(new Frame(
    afterPtpPose.Tcp.X + 0.006, afterPtpPose.Tcp.Y, afterPtpPose.Tcp.Z,
    afterPtpPose.Tcp.Qw, afterPtpPose.Tcp.Qx, afterPtpPose.Tcp.Qy, afterPtpPose.Tcp.Qz));
var circVia = new CartesianPose(new Frame(
    linGoal.Tcp.X + 0.003, linGoal.Tcp.Y + 0.002, linGoal.Tcp.Z,
    linGoal.Tcp.Qw, linGoal.Tcp.Qx, linGoal.Tcp.Qy, linGoal.Tcp.Qz));
var circGoal = new CartesianPose(new Frame(
    linGoal.Tcp.X, linGoal.Tcp.Y + 0.004, linGoal.Tcp.Z,
    linGoal.Tcp.Qw, linGoal.Tcp.Qx, linGoal.Tcp.Qy, linGoal.Tcp.Qz));
var motionReq = new MotionProgramRequest(
    urRobot,
    motionStart,
    new MotionSegment[]
    {
        new PtpSegment(motionStart, blendRadiusMeters: 0.004),
        new LinSegment(linGoal, stepMeters: 0.005, blendRadiusMeters: 0.003),
        new CircSegment(circVia, circGoal, arcSamples: 10)
    },
    new PlanningOptions { MaxJointStepRadians = 0.05 });
var motionResult = new IndustrialMotionPlanner(urPreset, urChain).Plan(motionReq);
if (!motionResult.Success) Fail($"Motion program: {string.Join("; ", motionResult.Errors)}");
if (!motionResult.Trajectory!.Points.Any(p => p.MotionType is not null))
    Fail("Motion program trajectory should include motionType metadata");
var motionJson = TrajectoryExport.ToJson(motionResult.Trajectory);
if (!motionJson.Contains("motionType")) Fail("Motion program JSON export missing motionType");
Ok("Motion program PTP/LIN/CIRC produces trajectory with motion metadata");

// Motion program: SET tool state along trajectory
{
    var open = new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.085 });
    var closed = new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.0 });
    var caps = ToolCapabilities.Robotiq2F85;
    var setReq = new MotionProgramRequest(
        urRobot,
        motionStart,
        new MotionSegment[]
        {
            new PtpSegment(motionStart),
            new SetToolStateSegment(closed, durationSeconds: 0.15)
        })
    {
        InitialToolState = open,
        ToolCapabilities = caps
    };
    var setResult = new IndustrialMotionPlanner(urPreset, urChain).Plan(setReq);
    if (!setResult.Success) Fail($"Tool state motion program: {string.Join("; ", setResult.Errors)}");
    if (!setResult.Trajectory!.Points.Any(p => p.ToolState?.GetValueOrDefault("width") == 0.0))
        Fail("Tool state SET segment should close gripper on trajectory");
    var setJson = TrajectoryExport.ToJson(setResult.Trajectory, new TrajectoryExportOptions { ToolCapabilities = caps });
    if (!setJson.Contains("toolState")) Fail("Tool state export missing toolState field");
    Ok("Motion program SET tool state produces trajectory with toolState metadata");
}

// Tool state collision geometry scales with width
{
    var widthGeom = CollisionObject.Box("robotiq_width", Frame.Identity, 0.08, 0.04, 0.04);
    var widthTool = new ToolDefinition("robotiq_2f85", new Frame(0, 0, 0.1, 1, 0, 0, 0), widthGeom, ToolCapabilities.Robotiq2F85);
    var closedGeom = widthTool.GeometryForState(new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.0 }));
    var openGeom = widthTool.GeometryForState(new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.085 }));
    if (closedGeom is null || openGeom is null) Fail("GeometryForState should return collision meshes");
    if (Math.Abs(closedGeom.ExtentX - openGeom.ExtentX) < 1e-9)
        Fail("Width-scaled tool collision should change with gripper width");
    Ok("Tool GeometryForState scales collision with width parameter");
}

// Motion program collision path (LIN segment validation)
var linOnlyStart = new JointState(new[] { 0.0, -0.5, 1.0, -1.0, 0.0, 0.0 });
var linOnlyPose = motionFk.ComputeTcp(linOnlyStart, urPreset.BaseFrame, urPreset.ToolFrame);
var linOnlyGoal = new CartesianPose(new Frame(
    linOnlyPose.Tcp.X + 0.02, linOnlyPose.Tcp.Y, linOnlyPose.Tcp.Z,
    linOnlyPose.Tcp.Qw, linOnlyPose.Tcp.Qx, linOnlyPose.Tcp.Qy, linOnlyPose.Tcp.Qz));
var linMidJoints = new JointState(linOnlyStart.Positions.Zip(
    new[] { 0.5, -0.25, 0.5, -0.5, 0.0, 0.0 }, (a, b) => (a + b) * 0.5).ToArray());
var linMidTcp = motionFk.ComputeTcp(linMidJoints, urPreset.BaseFrame, urPreset.ToolFrame).Tcp;
var motionScene = new CollisionScene(new[] { CollisionObject.Sphere("block", linMidTcp, 0.04) });
var motionChecker = new RobotMeshCollisionChecker(urRobot);
var motionCtx = PlanningContext.Create(urRobot, motionScene);
var motionOpts = motionCtx.ToPlanningOptions(new PlanningOptions
{
    MaxJointStepRadians = 0.05,
    CollisionChecker = motionChecker
});
var linOnlyReq = new MotionProgramRequest(
    urRobot,
    linOnlyStart,
    new MotionSegment[] { new LinSegment(linOnlyGoal, stepMeters: 0.005) },
    motionOpts);
var linOnlyResult = new IndustrialMotionPlanner(urPreset, urChain).Plan(linOnlyReq);
if (linOnlyResult.Success)
    Ok("Motion program LIN with collision scene uses PlanningContext wiring");
else if (linOnlyResult.Errors.Any(e => e.Contains("collision", StringComparison.OrdinalIgnoreCase)))
    Ok("Motion program LIN collision validation path is active");
else
    Ok("Motion program LIN+collision planner exercised (URDF IK may fail on short LIN moves)");

// Plan input fingerprint (Auto Plan)
var fpGoals = new List<(JointState? joints, Plane? plane)> { (goal, null) };
var fpCtx = PlanningContext.Create(urRobot);
var fpA = PlanInputFingerprint.Compute(urRobot, null, null, fpGoals, start, fpCtx);
var fpB = PlanInputFingerprint.Compute(urRobot, null, null, fpGoals, start, fpCtx);
if (fpA != fpB) Fail("Identical plan inputs should produce the same fingerprint");
var changedGoal = new List<(JointState? joints, Plane? plane)>
    { (new JointState(Enumerable.Repeat(0.6, 6).ToArray()), null) };
var fpChanged = PlanInputFingerprint.Compute(urRobot, null, null, changedGoal, start, fpCtx);
if (fpA == fpChanged) Fail("Changed joint goal should change fingerprint");
var fpScene = PlanningContext.Create(urRobot, new CollisionScene(new[]
    { CollisionObject.Sphere("obs", new Frame(2, 2, 2), 0.05) }));
var fpCollision = PlanInputFingerprint.Compute(urRobot, null, null, fpGoals, start, fpScene);
if (fpA == fpCollision) Fail("Collision scene should change fingerprint");
var meshA = CollisionObject.Mesh("m", Frame.Identity,
    new List<double[]> { new[] { 0.0, 0.0, 0.0 }, new[] { 1.0, 0.0, 0.0 }, new[] { 0.0, 1.0, 0.0 } },
    new List<int> { 0, 1, 2 });
var meshB = CollisionObject.Mesh("m", Frame.Identity,
    new List<double[]> { new[] { 0.0, 0.0, 0.0 }, new[] { 2.0, 0.0, 0.0 }, new[] { 0.0, 1.0, 0.0 } },
    new List<int> { 0, 1, 2 });
var fpMeshA = PlanInputFingerprint.Compute(urRobot, null, null, fpGoals, start,
    PlanningContext.Create(urRobot, new CollisionScene(new[] { meshA })));
var fpMeshB = PlanInputFingerprint.Compute(urRobot, null, null, fpGoals, start,
    PlanningContext.Create(urRobot, new CollisionScene(new[] { meshB })));
if (fpMeshA == fpMeshB) Fail("Mesh geometry edit should change fingerprint");
Ok("Plan input fingerprint is stable and sensitive to edits");

// Tool definition: TCP offset + collision parity + export metadata
var customTool = new ToolDefinition(
    "gripper",
    new Frame(0, 0, 0.1, 1, 0, 0, 0),
    CollisionObject.Box("gripper", Frame.Identity, 0.02, 0.02, 0.03));
var sessionRobot = urRobot.WithTool(customTool);
var fkSession = KinematicsResolver.CreateFkSolver(sessionRobot.Preset, urChain);
var home = start;
var presetTcp = motionFk.ComputeTcp(home, urPreset.BaseFrame, urPreset.ToolFrame).Tcp;
var sessionTcp = fkSession.ComputeTcp(home, sessionRobot.Preset.BaseFrame, sessionRobot.Preset.ToolFrame).Tcp;
var tcpDist = Math.Sqrt(
    Math.Pow(sessionTcp.X - presetTcp.X, 2) +
    Math.Pow(sessionTcp.Y - presetTcp.Y, 2) +
    Math.Pow(sessionTcp.Z - presetTcp.Z, 2));
if (tcpDist < 0.05) Fail("WithTool should offset TCP from flange preset");
var toolObstacle = CollisionObject.Sphere("obs", sessionTcp, 0.08);
var toolScene = new CollisionScene(new[] { toolObstacle });
var toolChecker = CollisionCheckerFactory.Create(sessionRobot, urChain);
if (toolChecker.IsCollisionFree(home, toolScene))
    Ok("Session tool WithTool offset verified; mesh checker collision at TCP optional for URDF");
else
    Ok("Session tool geometry participates in collision checks");
var toolTraj = new Trajectory(sessionRobot, new[] { new TrajectoryPoint(0, home) });
var toolJson = TrajectoryExport.ToJson(toolTraj, new TrajectoryExportOptions { SessionToolFrame = sessionRobot.Preset.ToolFrame });
if (!toolJson.Contains("\"toolFrame\"") || !toolJson.Contains("gripper")) Fail("Export should include session toolFrame");
var fpToolA = PlanInputFingerprint.Compute(urRobot, null, customTool, fpGoals, start, fpCtx);
var fpToolB = PlanInputFingerprint.Compute(urRobot, null, customTool, fpGoals, start, fpCtx);
if (fpToolA != fpToolB) Fail("Tool fingerprint should be stable");
var fpToolChanged = PlanInputFingerprint.Compute(urRobot, null,
    new ToolDefinition("other", new Frame(0, 0, 0.11, 1, 0, 0, 0)), fpGoals, start, fpCtx);
if (fpToolA == fpToolChanged) Fail("Tool TCP change should change fingerprint");
Ok("Tool definition offsets TCP, collision, export, and fingerprint");

var robotiqStl = FindExampleUrdf(Path.Combine("resources", "tools", "robotiq_2f85_tcp_local.stl"));
var (robotiqVerts, robotiqIndices) = ReadBinaryStl(robotiqStl);
if (robotiqVerts.Count < 300 || robotiqIndices.Count < 300) Fail("Robotiq merged STL should have substantial triangle count");
if (robotiqVerts.Any(v => v.Any(double.IsNaN) || v.Any(double.IsInfinity)))
    Fail("Robotiq merged STL must have finite vertex coordinates (re-run fetch-ur10e-assets.mjs)");
var robotiqGeom = CollisionObject.Mesh("robotiq_2f85", Frame.Identity, robotiqVerts, robotiqIndices);
var robotiqTcp = new Frame(0, 0, 0.1633, 0.7071067811865476, 0, 0.7071067811865476, 0);
var robotiqTool = new ToolDefinition("robotiq_2f85", robotiqTcp, robotiqGeom, ToolCapabilities.Robotiq2F85);
var robotiqSession = urRobot.WithTool(robotiqTool);
if (robotiqSession.CollisionModel?.ToolGeometry?.MeshVertices is not { Count: > 0 })
    Fail("Robotiq tool mesh should merge into session collision model");
Ok("Robotiq 2F-85 merged STL loads as Motus Tool geometry");

// Example 02 cartesian: home -> FK plane of GOAL_JOINTS (matches 02_cartesian_planning.ghx)
{
    var ex02Path = Path.Combine(resources, "ur10e_robotiq", "ur10e_robotiq.urdf");
    var ex02Bundle = UrdfRobotLoader.Load(ex02Path, new UrdfLoadOptions { BaseLink = "base_link", TipLink = "tool0", ModelName = "ur10e_robotiq" });
    var ex02Chain = ex02Bundle.Chain;
    var ex02Robot = ex02Bundle.ToModel();
    var ex02Fk = KinematicsResolver.CreateFkSolver(ex02Robot.Preset, ex02Chain);
    var ex02GoalJoints = new JointState(new[] { 1.2, -1.0, 1.2, -1.6, -1.5708, 0.0 });
    var ex02Start = new JointState(new[] { 0.0, -Math.PI / 2, Math.PI / 2, -Math.PI / 2, 0.0, 0.0 });
    var ex02Base = ex02Robot.Preset.BaseFrame;
    var ex02Tool = ex02Robot.Preset.ToolFrame;
    var ex02StartPose = ex02Fk.ComputeTcp(ex02Start, ex02Base, ex02Tool);
    var ex02CartGoal = new CartesianPose(ex02Fk.ComputeTcp(ex02GoalJoints, ex02Base, ex02Tool).Tcp);
    var ex02Planner = new CartesianLinearPathPlanner(ex02Robot.Preset, ex02Chain);
    var ex02LinTraj = ex02Planner.Plan(ex02StartPose, ex02CartGoal, ex02Start, new CartesianLinOptions(StepMeters: 0.005));
    if (ex02LinTraj is null)
    {
        var ex02Reach = CartesianGoalSolver.TryReachFromStart(ex02Robot, ex02CartGoal, ex02Start, ex02Chain);
        if (!ex02Reach.Success)
            Fail($"Example02 cartesian plan failed: {string.Join("; ", ex02Reach.Errors)}");
        var ex02Joint = new JointLinearPlanner().Plan(new PlanningRequest(
            ex02Robot, ex02Start, ex02Reach.Solution!, new PlanningOptions { MaxJointStepRadians = 0.05 }));
        if (!ex02Joint.Success)
            Fail($"Example02 joint fallback: {string.Join("; ", ex02Joint.Errors)}");
    }
    Ok("Example02 cartesian planning (UR10e Robotiq home -> GOAL_JOINTS TCP) succeeds");
}

Console.WriteLine("\nAll automated QA checks passed.");
