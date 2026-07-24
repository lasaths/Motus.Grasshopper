using Motus.Core;
using Motus.Geometry;
using Motus.GH.Rhino;
using System.Linq;

namespace Motus.GH.Preview;

/// <summary>
/// Converts an authored tool <see cref="RobotDescription"/> mechanism (Motus Urdf Link/Assemble
/// family) into <see cref="LinkCollisionGeometry"/> entries for merging into a robot's preview geometry
/// after <see cref="Motus.Geometry.KinematicTree.Attach"/>. Only in-memory baked meshes/primitives are
/// supported — <see cref="UrdfGeometry.FilePath"/>-only visuals (no baked <see cref="UrdfGeometry.Vertices"/>)
/// are skipped, since mechanisms are typically authored directly from Grasshopper geometry, not URDF files.
/// </summary>
internal static class MechanismPreviewGeometry
{
    public static RobotCollisionModel? Build(RobotDescription mechanism)
    {
        var links = new List<LinkCollisionGeometry>();
        foreach (var link in mechanism.Links)
        {
            foreach (var visual in link.Visuals)
            {
                if (ToCollisionObject(link.Name, visual) is { } obj)
                    // TreeLinkIndex sentinel: mesh is posed via TreeFK + LinkName lookup, since mechanism
                    // links have no index in the arm's own serial-chain FK.
                    links.Add(new LinkCollisionGeometry(KinematicsPreview.TreeLinkIndex, link.Name, obj));
            }
        }

        return links.Count == 0 ? null : new RobotCollisionModel(links);
    }

    private static CollisionObject? ToCollisionObject(string linkName, UrdfGeometry g) => g.Kind switch
    {
        UrdfGeometryKind.Box => CollisionObject.Box(linkName, g.Origin, g.SizeX / 2, g.SizeY / 2, g.SizeZ / 2),
        UrdfGeometryKind.Sphere => CollisionObject.Sphere(linkName, g.Origin, g.Radius),
        // No dedicated cylinder primitive in CollisionObject; Capsule (rounded caps) is the closest visual
        // approximation available — same convention KinematicsPreview.ToRhinoMesh already uses elsewhere.
        UrdfGeometryKind.Cylinder => CollisionObject.Capsule(linkName, g.Origin, g.Radius, g.Length / 2),
        UrdfGeometryKind.Mesh when g.Vertices is { Count: > 0 } vertices && g.Indices is { Count: > 0 } indices =>
            CollisionObject.Mesh(linkName, g.Origin, vertices.ToList(), indices.ToList()),
        _ => null,
    };
}
