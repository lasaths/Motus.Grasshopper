using Grasshopper.Kernel;
using System.Threading;

namespace Motus.GH.Async;

/// <summary>
/// Holds compute logic for a <see cref="GH_AsyncComponent"/>. Snapshot inputs in
/// <see cref="GetData"/>, run heavy work in <see cref="DoWork"/>, commit on the GH thread in <see cref="SetData"/>.
/// </summary>
public abstract class WorkerInstance
{
    public GH_Component? Parent { get; set; }
    public CancellationToken CancellationToken { get; set; }
    public string Id { get; set; } = string.Empty;
    public string CompletionMessage { get; set; } = "Done";

    public abstract WorkerInstance Duplicate();
    public abstract void GetData(IGH_DataAccess da, GH_ComponentParamServer parameters);
    public abstract void DoWork(Action<string, double> reportProgress, Action done);
    public abstract void SetData(IGH_DataAccess da);

    /// <summary>Push worker results into the owning component cache on the UI thread before the SetData pass.</summary>
    public virtual void CommitCachedResults() { }
}
