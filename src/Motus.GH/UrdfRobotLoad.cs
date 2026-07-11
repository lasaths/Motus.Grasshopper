using Motus.Core;
using Motus.Geometry;
using Motus.GH.Data;
using Motus.GH.Urdf;
using Motus.Presets;
using System.Collections.Concurrent;
using System.Drawing;
using System.Xml.Linq;

namespace Motus.GH;

internal static class UrdfRobotLoad
{
    private sealed record CachedRobot(
        RobotModel Model,
        SerialJointChain Chain,
        RobotCollisionModel? PreviewGeometry,
        Color?[]? PreviewMeshColors,
        string UrdfSourcePath);

    private static readonly ConcurrentDictionary<string, CachedRobot> Cache = new(StringComparer.OrdinalIgnoreCase);

    public static RobotModelGoo Load(string path, string baseLink = "base_link", string tipLink = "tool0")
    {
        path = UrdfPathResolver.ResolveUrdfPath(path);
        if (!File.Exists(path))
            throw new FileNotFoundException($"URDF not found: {path}");

        var cacheKey = CacheKey(path, baseLink, tipLink);
        if (!Cache.TryGetValue(cacheKey, out var cached))
        {
            cached = LoadUncached(path, baseLink, tipLink);
            Cache[cacheKey] = cached;
        }

        return CreateGoo(cached);
    }

    internal static RobotPreviewVisuals? LoadPreviewVisuals(string path, string baseLink = "base_link", string tipLink = "tool0") =>
        UrdfVisualPreviewLoader.TryLoad(UrdfPathResolver.ResolveUrdfPath(path), baseLink, tipLink);

    private static string CacheKey(string path, string baseLink, string tipLink)
    {
        var full = Path.GetFullPath(path);
        return $"{full}|{baseLink}|{tipLink}|{UrdfWriteTimeCache.GetTicks(full)}";
    }

    private static CachedRobot LoadUncached(string path, string baseLink, string tipLink)
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

        return new CachedRobot(
            urdf.ToModel(),
            urdf.Chain,
            previewVisuals?.Geometry,
            previewVisuals?.MeshColors,
            path);
    }

    private static RobotModelGoo CreateGoo(CachedRobot cached)
    {
        var goo = new RobotModelGoo(cached.Model)
        {
            Chain = cached.Chain,
            PreviewGeometry = cached.PreviewGeometry,
            PreviewMeshColors = cached.PreviewMeshColors,
            UrdfSourcePath = cached.UrdfSourcePath
        };
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
