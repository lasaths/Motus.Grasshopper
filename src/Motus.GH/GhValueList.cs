using System.Drawing;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Special;

namespace Motus.GH;

internal static class GhValueList
{
    /// <summary>GH Value List type id (dropdown preset picker).</summary>
    public static readonly Guid ValueListComponentId = new("86fb34a5-80c2-4a87-bf57-c9a32572455f");

    public static void AttachDropdown(GH_Component owner, int inputIndex, IEnumerable<string> items)
    {
        if (owner.Params.Input[inputIndex].SourceCount > 0) return;
        var doc = owner.OnPingDocument();
        if (doc is null) return;

        var list = new GH_ValueList
        {
            NickName = "Model",
            ListMode = GH_ValueListMode.DropDown
        };
        list.CreateAttributes();
        list.Attributes.Pivot = new PointF(owner.Attributes.Pivot.X - 120, owner.Attributes.Pivot.Y + 14 + inputIndex * 22);
        foreach (var item in items)
            list.ListItems.Add(new GH_ValueListItem(item, $"\"{item}\""));

        doc.AddObject(list, false);
        owner.Params.Input[inputIndex].AddSource(list);
    }
}
