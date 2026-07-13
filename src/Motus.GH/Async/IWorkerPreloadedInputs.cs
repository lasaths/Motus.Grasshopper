namespace Motus.GH.Async;

/// <summary>Workers that accept a pre-built input snapshot instead of re-reading data access.</summary>
internal interface IWorkerPreloadedInputs
{
    void ApplySnapshot(object snapshot);
}
