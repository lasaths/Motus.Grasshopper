using Motus.Core;

namespace Motus.GH;

/// <summary>
/// Cap = ToolCapabilities schema only (not ToolMode, not bindings).
/// No name-sneak; Cap=None means null schema.
/// </summary>
public static class ToolCapContract
{
    public const string None = "None";
    public const string Robotiq2F85 = "Robotiq2F85";

    public static readonly string[] Schemas = [None, Robotiq2F85];

    public static string Normalize(string? raw)
    {
        var t = (raw ?? None).Trim();
        if (string.IsNullOrWhiteSpace(t) ||
            t.Equals(None, StringComparison.OrdinalIgnoreCase) ||
            t.Equals("Off", StringComparison.OrdinalIgnoreCase))
            return None;
        if (t.Equals(Robotiq2F85, StringComparison.OrdinalIgnoreCase) ||
            t.Equals("Robotiq", StringComparison.OrdinalIgnoreCase) ||
            t.Equals("2F85", StringComparison.OrdinalIgnoreCase))
            return Robotiq2F85;
        return t.Trim();
    }

    /// <summary>Parse Cap schema string. False when value is not None/Robotiq2F85.</summary>
    public static bool TryParseSchema(string? raw, out ToolCapabilities? caps)
    {
        caps = null;
        var n = Normalize(raw);
        if (n == None) return true;
        if (n == Robotiq2F85)
        {
            caps = ToolCapabilities.Robotiq2F85;
            return true;
        }

        return false;
    }

    /// <summary>
    /// Tool State Cap gate: wired Tool/Robot with null Cap → error; unwired → warning + Robotiq.
    /// </summary>
    public static bool TryResolveForToolState(
        ToolDefinition? tool,
        bool toolOrRobotWired,
        out ToolCapabilities caps,
        out string? error,
        out string? warning)
    {
        error = null;
        warning = null;
        if (tool?.Capabilities is { } c)
        {
            caps = c;
            return true;
        }

        if (!toolOrRobotWired)
        {
            caps = ToolCapabilities.Robotiq2F85;
            warning =
                "No Tool/Robot wired — assuming Robotiq 2F-85. Wire Motus Tool (Cap=Robotiq2F85) or Motus UR10e/Robot for real capabilities.";
            return true;
        }

        caps = null!;
        error = tool is null
            ? "Wired Robot has no Tool with Cap — attach a Motus Tool with Cap=Robotiq2F85, or use a robot that ships capabilities."
            : "Wired tool has no Cap — set Motus Tool Cap to Robotiq2F85 (parameter schema for Tool State / export; not ToolMode).";
        return false;
    }
}
