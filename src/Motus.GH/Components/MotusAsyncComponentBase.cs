using Motus.GH.Async;
using Motus.GH.Resources;
using System.Drawing;

namespace Motus.GH.Components;

public abstract class MotusAsyncComponentBase : GH_AsyncComponent
{
    private readonly string _iconName;
    private readonly string _subcategory;

    protected MotusAsyncComponentBase(string name, string nickname, string desc, string sub, string iconName = "cube")
        : base(name, nickname, desc, "Motus", sub)
    {
        _subcategory = sub;
        _iconName = iconName;
    }

    protected override Bitmap Icon =>
        MotusIcon.Get(_iconName, MotusIcon.SubcategoryColor(_subcategory));
}
