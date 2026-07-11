using Motus.Core;
using System.Drawing;

namespace Motus.GH.Urdf;

/// <summary>URDF visual geometry plus per-link preview colours (parallel to <see cref="Geometry"/>.Links).</summary>
public sealed record RobotPreviewVisuals(RobotCollisionModel Geometry, Color?[] MeshColors);
