using System.Runtime.CompilerServices;
using System.Runtime.Loader;

internal static class RhinoBootstrap
{
    [ModuleInitializer]
    internal static void Init()
    {
        AssemblyLoadContext.Default.Resolving += static (_, name) =>
        {
            if (!string.Equals(name.Name, "RhinoCommon", StringComparison.OrdinalIgnoreCase))
                return null;

            foreach (var path in CandidatePaths())
            {
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

        // Last resort: beside the smoke exe (manual copy)
        yield return Path.Combine(AppContext.BaseDirectory, "RhinoCommon.dll");
    }
}
