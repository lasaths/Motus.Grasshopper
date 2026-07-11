namespace Motus.GH.Urdf;

internal static class UrdfPaths
{
    public static bool IsUnderDirectory(string resolvedPath, string rootDirectory)
    {
        if (string.IsNullOrWhiteSpace(resolvedPath) || string.IsNullOrWhiteSpace(rootDirectory))
            return false;

        var full = Path.GetFullPath(resolvedPath);
        var root = Path.GetFullPath(rootDirectory);
        if (!root.EndsWith(Path.DirectorySeparatorChar))
            root += Path.DirectorySeparatorChar;
        return full.StartsWith(root, StringComparison.OrdinalIgnoreCase);
    }
}
