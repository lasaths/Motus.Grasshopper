using System.Runtime.CompilerServices;
using System.Runtime.Loader;

/// <summary>
/// NuGet RhinoCommon is a reference assembly (not executable). Load the real DLL from a Rhino 8 install.
/// </summary>
internal static class RhinoBootstrap
{
    private static string? _rhinoCommonPath;

    [ModuleInitializer]
    internal static void Init()
    {
        _rhinoCommonPath = CandidatePaths().FirstOrDefault(File.Exists);
        if (_rhinoCommonPath is null)
        {
            throw new FileNotFoundException(
                "Executable RhinoCommon.dll not found. Install Rhino 8, or set Rhino8Dir to the Rhino 8 root " +
                "(folder that contains System\\RhinoCommon.dll). The NuGet RhinoCommon package is compile-only.");
        }

        var systemDir = Path.GetDirectoryName(_rhinoCommonPath)!;
        AssemblyLoadContext.Default.Resolving += (_, name) =>
        {
            if (name.Name is null) return null;
            // ponytail: same System folder as RhinoCommon (Eto, Rhino.UI, …)
            if (name.Name is "RhinoCommon" or "Rhino.UI" or "Eto" or "Ed.Eto")
            {
                var path = Path.Combine(systemDir, name.Name + ".dll");
                if (File.Exists(path))
                    return AssemblyLoadContext.Default.LoadFromAssemblyPath(path);
            }

            return null;
        };
    }

    private static IEnumerable<string> CandidatePaths()
    {
        var rhino8 = Environment.GetEnvironmentVariable("Rhino8Dir");
        if (!string.IsNullOrWhiteSpace(rhino8))
            yield return Path.Combine(rhino8, "System", "RhinoCommon.dll");

        if (OperatingSystem.IsWindows())
            yield return Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.ProgramFiles),
                "Rhino 8", "System", "RhinoCommon.dll");

        if (OperatingSystem.IsMacOS())
        {
            yield return "/Applications/Rhino 8.app/Contents/Frameworks/RhCore.framework/Versions/Current/Resources/RhinoCommon.dll";
        }
    }
}
