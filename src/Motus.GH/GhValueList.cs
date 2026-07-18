using System.Drawing;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Special;

namespace Motus.GH;

internal static class GhValueList
{
    /// <summary>GH Value List type id (dropdown preset picker).</summary>
    public static readonly Guid ValueListComponentId = new("86fb34a5-80c2-4a87-bf57-c9a32572455f");

    public static void AttachDropdown(GH_Component owner, int inputIndex, IEnumerable<string> items, string? listNickName = null)
    {
        if (owner.Params.Input[inputIndex].SourceCount > 0) return;
        var doc = owner.OnPingDocument();
        if (doc is null) return;

        var list = new GH_ValueList
        {
            NickName = listNickName ?? "Model",
            ListMode = GH_ValueListMode.DropDown
        };
        list.ListItems.Clear();
        list.CreateAttributes();
        list.Attributes.Pivot = new PointF(owner.Attributes.Pivot.X - 120, owner.Attributes.Pivot.Y + 14 + inputIndex * 22);
        foreach (var item in items)
            list.ListItems.Add(new GH_ValueListItem(item, $"\"{item}\""));

        // Keep document persistent value (e.g. Closed) instead of always selecting the first item.
        var selected = ReadPersistentText(owner.Params.Input[inputIndex]);
        if (!string.IsNullOrWhiteSpace(selected))
        {
            for (var i = 0; i < list.ListItems.Count; i++)
            {
                if (!list.ListItems[i].Name.Equals(selected, StringComparison.OrdinalIgnoreCase)) continue;
                list.SelectItem(i);
                break;
            }
        }

        doc.AddObject(list, false);
        owner.Params.Input[inputIndex].AddSource(list);
    }

    private static string? ReadPersistentText(IGH_Param param)
    {
        if (param is not Grasshopper.Kernel.Parameters.Param_String ps || ps.PersistentDataCount <= 0)
            return null;
        return ps.PersistentData.get_FirstItem(false)?.Value;
    }
}
