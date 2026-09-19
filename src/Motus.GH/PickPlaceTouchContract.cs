namespace Motus.GH;

/// <summary>
/// Motus Pick Place <c>Touch</c> gate — Detach-at-place needs gripper collision body names.
/// Shared with qa-smoke (Rhino-free). Empty Touch must fail closed (no Seg → Program Tr null).
/// </summary>
public static class PickPlaceTouchContract
{
    public const string EmptyTouchError =
        "Touch empty — Detach-at-place needs gripper collision body names (e.g. robotiq_2f85) or plan fails.";

    /// <summary>
    /// Normalize Touch body names: trim, drop blanks. Does not invent names.
    /// </summary>
    public static List<string> Normalize(IEnumerable<string?>? raw)
    {
        if (raw is null)
            return [];
        return raw
            .Where(s => !string.IsNullOrWhiteSpace(s))
            .Select(s => s!.Trim())
            .ToList();
    }

    /// <summary>
    /// Returns false when Touch is empty (no usable body names).
    /// </summary>
    public static bool TryRequireTouch(IEnumerable<string?>? raw, out IReadOnlyList<string> bodies, out string? error)
    {
        var list = Normalize(raw);
        bodies = list;
        if (list.Count == 0)
        {
            error = EmptyTouchError;
            return false;
        }

        error = null;
        return true;
    }
}
