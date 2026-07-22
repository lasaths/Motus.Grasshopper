using Motus.GH.Async;
using Motus.GH.Resources;
using System.Collections.Generic;
using System.Drawing;

namespace Motus.GH.Components;

public abstract class MotusAsyncComponentBase : GH_AsyncComponent
{
    private readonly string _iconName;
    private readonly string _subcategory;

    protected MotusAsyncComponentBase(string name, string nickname, string desc, string sub, string iconName)
        : base(name, nickname, desc, "Motus", sub)
    {
        _subcategory = sub;
        _iconName = iconName;
    }

    /// <summary>
    /// AI wiring hints for Cassis/MCP (search keywords only — not hover tooltips).
    /// </summary>
    // ponytail: Keywords keep recipes out of Description tooltips; Cassis must expose Keywords.
    protected virtual IReadOnlyList<string> AiKeywords => [];

    public override IEnumerable<string> Keywords
    {
        get
        {
            // ponytail: GH_DocumentObject.Keywords is null during GHA registration
            foreach (var k in base.Keywords ?? [])
                yield return k;
            foreach (var k in AiKeywords)
                yield return k;
        }
    }

    protected override Bitmap Icon =>
        MotusIcon.Get(_iconName, MotusIcon.SubcategoryColor(_subcategory));
}
