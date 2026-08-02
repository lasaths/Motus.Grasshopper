using Motus.Core;
using Motus.Geometry;
using Motus.GH.Data;
using Motus.GH.Urdf;
using Motus.Presets;
using System.Collections.Concurrent;
using System.Drawing;
using System.Xml.Linq;

namespace Motus.GH.Urdf;

internal static class UrdfRobotLoad
{
    private sealed record CachedRobot(
        RobotModel Model,
        SerialJointChain Chain,
        KinematicTree? Tree,
        RobotCollisionModel? PreviewGeometry,
        Color?[]? PreviewMeshColors,
        string UrdfSourcePath);

    private static readonly ConcurrentDictionary<string, CachedRobot> Cache = new(StringComparer.OrdinalIgnoreCase);

    public static RobotModelGoo Load(
        string path,
        string baseLink = "base_link",
        string tipLink = "tool0",
        bool allDrivers = false)
    {
        path = UrdfPathResolver.ResolveUrdfPath(path);
        if (!File.Exists(path))
            throw new FileNotFoundException($"URDF not found: {path}");

        var cacheKey = CacheKey(path, baseLink, tipLink, allDrivers);
        if (!Cache.TryGetValue(cacheKey, out var cached))
        {
            cached = LoadUncached(path, baseLink, tipLink, allDrivers);
            Cache[cacheKey] = cached;
        }

        return CreateGoo(cached);
    }

    internal static RobotPreviewVisuals? LoadPreviewVisuals(string path, string baseLink = "base_link", string tipLink = "tool0") =>
        UrdfVisualPreviewLoader.TryLoad(UrdfPathResolver.ResolveUrdfPath(path), baseLink, tipLink);

    private static string CacheKey(string path, string baseLink, string tipLink, bool allDrivers)
    {
        var full = Path.GetFullPath(path);
        return $"{full}|{baseLink}|{tipLink}|{allDrivers}|{UrdfWriteTimeCache.GetTicks(full)}";
    }

    private static CachedRobot LoadUncached(string path, string baseLink, string tipLink, bool allDrivers)
    {
        var urdfDir = Path.GetDirectoryName(Path.GetFullPath(path)) ?? ".";
        var options = new UrdfLoadOptions
        {
            BaseLink = baseLink,
            TipLink = tipLink,
            ModelName = Path.GetFileNameWithoutExtension(path)
        };

        UrdfRobot urdf;
        RobotPreviewVisuals? previewVisuals;
        if (path.EndsWith(".xacro", StringComparison.OrdinalIgnoreCase))
        {
            var xdoc = XacroPreprocessor.ExpandDocument(path);
            urdf = UrdfRobotLoader.Load(xdoc, options, urdfDir);
            previewVisuals = PreviewVisualsFor(urdf, path, baseLink, tipLink, xdoc, urdfDir);
        }
        else
        {
            urdf = UrdfRobotLoader.Load(path, options);
            previewVisuals = PreviewVisualsFor(urdf, path, baseLink, tipLink, null, urdfDir);
        }

        var model = allDrivers ? ModelFromAllTreeDrivers(urdf, tipLink) : urdf.ToModel();
        if (allDrivers && urdf.Tree is { } tree)
        {
            XDocument? docForCol = null;
            if (path.EndsWith(".xacro", StringComparison.OrdinalIgnoreCase))
                docForCol = XacroPreprocessor.ExpandDocument(path);
            else if (File.Exists(path))
                docForCol = XDocument.Load(path);

            if (docForCol?.Root is { } root)
            {
                var treeCol = UrdfCollisionLoader.LoadTree(root, tree, urdfDir, tipLink);
                if (treeCol is not null)
                {
                    model = new RobotModel(model.Preset, treeCol, model.JointNames);
                }
            }
        }

        return new CachedRobot(
            model,
            urdf.Chain,
            urdf.Tree,
            previewVisuals?.Geometry,
            previewVisuals?.MeshColors,
            path);
    }

    /// <summary>
    /// Plan/Joint State = tip-path drivers, then side-branch drivers (e.g. DKP).
    /// Tip-descendant tool drivers (Robotiq knuckle) stay off Plan DOF — TreeFK/tool bindings own them.
    /// Serial tip chain stays for FK/IK; prefer joint goals when branches are actuated.
    /// </summary>
    private static RobotModel ModelFromAllTreeDrivers(UrdfRobot urdf, string tipLink)
    {
        var tree = urdf.Tree
            ?? throw new InvalidOperationException("AllDrivers requires a kinematic tree from the URDF load.");
        if (tree.DriverCount <= 0)
            return urdf.ToModel();

        var tipNames = urdf.JointNames;
        var layout = PlanDofComposer.TipThenSideBranches(tree, tipNames, tipLink, excludeTipDescendants: true);
        if (layout.JointNames.Count == tipNames.Count)
            return urdf.ToModel();

        var tip = urdf.Preset;
        var preset = new RobotPreset
        {
            Manufacturer = tip.Manufacturer,
            ModelName = tip.ModelName,
            Family = tip.Family,
            AxisCount = layout.JointNames.Count,
            JointLimits = layout.Limits,
            ReachMeters = tip.ReachMeters,
            PayloadKg = tip.PayloadKg,
            BaseFrame = tip.BaseFrame,
            ToolFrame = tip.ToolFrame,
            Notes = tip.Notes,
            SourceNote = string.IsNullOrWhiteSpace(tip.SourceNote)
                ? "URDF tip + side-branch drivers"
                : $"{tip.SourceNote}; tip + side-branch drivers",
            Disclaimer = tip.Disclaimer
        };
        return new RobotModel(preset, urdf.CollisionModel, layout.JointNames);
    }

    private static RobotModelGoo CreateGoo(CachedRobot cached)
    {
        var goo = new RobotModelGoo(cached.Model)
        {
            Chain = cached.Chain,
            Tree = cached.Tree,
            PreviewGeometry = cached.PreviewGeometry,
            PreviewMeshColors = cached.PreviewMeshColors,
            UrdfSourcePath = cached.UrdfSourcePath
        };
        if (cached.Tree is { } tree
            && tree.DriverCount > cached.Model.Preset.AxisCount
            && goo.TreeDriverHome is null)
        {
            goo.TreeDriverHome = new JointState(new double[tree.DriverCount]);
        }
        goo.EnsureBundledTool();
        return goo;
    }

    private static RobotPreviewVisuals? PreviewVisualsFor(
        UrdfRobot urdf,
        string path,
        string baseLink,
        string tipLink,
        XDocument? xdoc = null,
        string? urdfDir = null)
    {
        urdfDir ??= Path.GetDirectoryName(Path.GetFullPath(path)) ?? ".";
        var visual = xdoc is not null
            ? UrdfVisualPreviewLoader.TryLoad(xdoc, urdfDir, baseLink, tipLink)
            : UrdfVisualPreviewLoader.TryLoad(path, baseLink, tipLink);

        if (visual?.Geometry.Links.Count > 0)
            return visual;

        if (urdf.CollisionModel?.Links.Count > 0)
            return new RobotPreviewVisuals(urdf.CollisionModel, new Color?[urdf.CollisionModel.Links.Count]);

        return null;
    }
}
