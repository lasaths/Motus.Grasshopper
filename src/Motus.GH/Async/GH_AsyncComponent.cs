using Grasshopper.Kernel;
using Rhino;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Timer = System.Timers.Timer;

namespace Motus.GH.Async;

/// <summary>
/// Grasshopper component base that runs <see cref="WorkerInstance"/> work on a background thread.
/// </summary>
public abstract class GH_AsyncComponent : GH_Component
{
    private readonly object _lifecycleLock = new();
    private readonly ConcurrentQueue<string> _workerErrors = new();
    private readonly Timer _displayProgressTimer;

    private int _runId;
    private int _completedWorkerCount;
    private int _setDataIndex;
    private bool _isRunning;
    private bool _isReadyToSetData;
    private bool _setDataExpireScheduled;
    private bool _documentIsSolving;
    private bool _setDataSolutionPending;
    private bool _workersCommitted;
    private object? _workerInputSnapshot;

    protected GH_AsyncComponent(string name, string nickname, string description, string category, string subCategory)
        : base(name, nickname, description, category, subCategory)
    {
        _displayProgressTimer = new Timer(333) { AutoReset = false };
        _displayProgressTimer.Elapsed += DisplayProgress;
        ProgressReports = new ConcurrentDictionary<string, double>();
        Workers = [];
        CancellationSources = [];
        Tasks = [];
    }

    public ConcurrentDictionary<string, double> ProgressReports { get; }
    public List<WorkerInstance> Workers { get; private set; }
    public List<CancellationTokenSource> CancellationSources { get; }
    public TaskCreationOptions? TaskCreationOptions { get; set; }
    public WorkerInstance? BaseWorker { get; set; }

    private List<Task> Tasks { get; set; }

    protected bool IsReadyToSetData
    {
        get { lock (_lifecycleLock) return _isReadyToSetData; }
    }

    protected bool IsOperationInProgress
    {
        get { lock (_lifecycleLock) return _isRunning || _isReadyToSetData; }
    }

    /// <summary>When false, an in-flight worker survives re-solves (e.g. cached idle passes).</summary>
    protected virtual bool ShouldAbortRunningWorkers() => true;

    public override void AddedToDocument(GH_Document doc)
    {
        base.AddedToDocument(doc);
        doc.SolutionStart += OnDocumentSolutionStart;
        doc.SolutionEnd += OnDocumentSolutionEnd;
    }

    public override void RemovedFromDocument(GH_Document doc)
    {
        doc.SolutionStart -= OnDocumentSolutionStart;
        doc.SolutionEnd -= OnDocumentSolutionEnd;
        base.RemovedFromDocument(doc);
    }

    private void OnDocumentSolutionStart(object? sender, GH_SolutionEventArgs e) =>
        _documentIsSolving = true;

    private void OnDocumentSolutionEnd(object? sender, GH_SolutionEventArgs e)
    {
        _documentIsSolving = false;
        if (_setDataSolutionPending)
            EmitSetDataSolutionNow();
    }

    protected void LaunchWorker(IGH_DataAccess da) => LaunchWorker(da, null);

    protected void LaunchWorker(IGH_DataAccess da, object? inputSnapshot)
    {
        _workerInputSnapshot = inputSnapshot;
        CollectWorker(da);
    }

    protected override void BeforeSolveInstance()
    {
        lock (_lifecycleLock)
        {
            if (_isReadyToSetData) return;
        }

        if (ShouldAbortRunningWorkers())
            ResetCurrentRun(cancelWorkers: true, message: null);
    }

    protected override void AfterSolveInstance()
    {
        List<Task>? tasksToStart = null;

        lock (_lifecycleLock)
        {
            if (!_isRunning && !_isReadyToSetData && Tasks.Count > 0)
            {
                _isRunning = true;
                tasksToStart = Tasks.ToList();
            }
        }

        if (tasksToStart is null)
        {
            ClearIdleMessage();
            return;
        }

        foreach (var task in tasksToStart)
            task.Start();
    }

    protected override void ExpireDownStreamObjects()
    {
        lock (_lifecycleLock)
        {
            // Keep stale downstream data only while background work is still running.
            if (_isRunning && !_isReadyToSetData)
                return;
        }

        base.ExpireDownStreamObjects();
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        bool readyToSetData;
        lock (_lifecycleLock)
            readyToSetData = _isReadyToSetData;

        if (!readyToSetData)
            return;

        SetWorkerData(da);
    }

    public void RequestCancellation() => ResetCurrentRun(cancelWorkers: true, message: "Cancelled");

    private void CollectWorker(IGH_DataAccess da)
    {
        if (BaseWorker is null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Worker class not provided.");
            return;
        }

        var currentWorker = BaseWorker.Duplicate();
        if (_workerInputSnapshot is not null && currentWorker is IWorkerPreloadedInputs preload)
            preload.ApplySnapshot(_workerInputSnapshot);
        _workerInputSnapshot = null;

        try
        {
            currentWorker.GetData(da, Params);
        }
        catch (Exception ex)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
            return;
        }

        if (currentWorker is IWorkerSkip { SkipWork: true })
            return;

        int runId;
        lock (_lifecycleLock)
            runId = _runId;

        var tokenSource = new CancellationTokenSource();
        currentWorker.CancellationToken = tokenSource.Token;
        currentWorker.Parent = this;
        currentWorker.Id = $"Run-{runId}-Worker-{da.Iteration}";

        var doneCalled = 0;
        Action done = () =>
        {
            if (Interlocked.Exchange(ref doneCalled, 1) == 0)
                MarkWorkerDone(runId);
        };

        Action<string, double> reportProgress = (id, value) => ReportProgress(runId, id, value);
        Task currentRun = TaskCreationOptions is { } options
            ? new Task(() => RunWorker(currentWorker, reportProgress, done, runId), tokenSource.Token, options)
            : new Task(() => RunWorker(currentWorker, reportProgress, done, runId), tokenSource.Token);

        lock (_lifecycleLock)
        {
            if (runId != _runId || _isReadyToSetData)
            {
                tokenSource.Cancel();
                tokenSource.Dispose();
                return;
            }

            CancellationSources.Add(tokenSource);
            Workers.Add(currentWorker);
            Tasks.Add(currentRun);
        }
    }

    private void SetWorkerData(IGH_DataAccess da)
    {
        WorkerInstance? worker = null;
        var finished = false;

        FlushWorkerErrors();

        lock (_lifecycleLock)
        {
            if (_setDataIndex < Workers.Count)
            {
                worker = Workers[_setDataIndex];
                _setDataIndex++;
            }

            finished = _setDataIndex >= Workers.Count;
        }

        if (worker is not null)
        {
            try
            {
                worker.SetData(da);
            }
            catch (Exception ex)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $"{worker.Id}: {ex.Message}");
            }
        }

        if (!finished)
            return;

        ResetCompletedRun();
        // Defer downstream expire — calling ExpireDownStreamObjects during this SetData pass
        // expires recipients (e.g. Preview) mid-solution.
        OnPingDocument()?.ScheduleSolution(1, _ => base.ExpireDownStreamObjects());
    }

    private void RunWorker(WorkerInstance worker, Action<string, double> reportProgress, Action done, int runId)
    {
        try
        {
            worker.DoWork(reportProgress, done);
        }
        catch (OperationCanceledException)
        {
        }
        catch (Exception ex)
        {
            if (IsCurrentRun(runId))
                _workerErrors.Enqueue($"{worker.Id}: {ex.Message}");
        }
        finally
        {
            done();
        }
    }

    private void ReportProgress(int runId, string id, double value)
    {
        if (!IsCurrentRun(runId))
            return;

        ProgressReports[id ?? string.Empty] = value;
        if (!_displayProgressTimer.Enabled)
            _displayProgressTimer.Start();
    }

    private void MarkWorkerDone(int runId)
    {
        var shouldExpire = false;

        lock (_lifecycleLock)
        {
            if (runId != _runId || _isReadyToSetData)
                return;

            _completedWorkerCount++;
            if (_completedWorkerCount == Workers.Count && Workers.Count > 0 && !_setDataExpireScheduled)
            {
                _isRunning = false;
                _isReadyToSetData = true;
                _setDataIndex = 0;
                _setDataExpireScheduled = true;
                shouldExpire = true;
            }
        }

        if (!shouldExpire)
            return;

        _setDataSolutionPending = true;
        RhinoApp.InvokeOnUiThread((Action)(() =>
        {
            CommitWorkerCachedResults();
            if (!_documentIsSolving)
                EmitSetDataSolutionNow();
        }));
    }

    private void CommitWorkerCachedResults()
    {
        List<WorkerInstance> workers;
        lock (_lifecycleLock)
        {
            if (!_isReadyToSetData || _workersCommitted)
                return;
            workers = Workers.ToList();
            _workersCommitted = true;
        }

        foreach (var worker in workers)
            worker.CommitCachedResults();
    }

    private void EmitSetDataSolutionNow()
    {
        lock (_lifecycleLock)
        {
            if (!_isReadyToSetData)
            {
                _setDataSolutionPending = false;
                return;
            }
        }

        if (OnPingDocument() is not GH_Document doc)
            return;

        _setDataSolutionPending = false;
        if (!_workersCommitted)
            CommitWorkerCachedResults();
        ExpireSolution(false);
        doc.NewSolution(false);
    }

    private bool IsCurrentRun(int runId)
    {
        lock (_lifecycleLock)
            return runId == _runId;
    }

    private void FlushWorkerErrors()
    {
        while (_workerErrors.TryDequeue(out var error))
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, error);
    }

    private void ResetCompletedRun()
    {
        var completionMessage = "Done";

        lock (_lifecycleLock)
        {
            if (Workers.Count > 0)
                completionMessage = Workers[^1].CompletionMessage ?? "Done";

            foreach (var source in CancellationSources)
                source.Dispose();

            CancellationSources.Clear();
            Workers.Clear();
            ProgressReports.Clear();
            Tasks.Clear();
            ClearWorkerErrors();

            _completedWorkerCount = 0;
            _setDataIndex = 0;
            _isRunning = false;
            _isReadyToSetData = false;
            _setDataExpireScheduled = false;
            _setDataSolutionPending = false;
            _workersCommitted = false;
            _runId++;
        }

        Message = string.IsNullOrWhiteSpace(completionMessage) ? string.Empty : completionMessage;
        OnDisplayExpired(true);
    }

    private void ResetCurrentRun(bool cancelWorkers, string? message)
    {
        lock (_lifecycleLock)
        {
            _runId++;

            if (cancelWorkers)
            {
                foreach (var source in CancellationSources)
                {
                    source.Cancel();
                    source.Dispose();
                }
            }

            CancellationSources.Clear();
            Workers.Clear();
            ProgressReports.Clear();
            Tasks.Clear();
            ClearWorkerErrors();

            _completedWorkerCount = 0;
            _setDataIndex = 0;
            _isRunning = false;
            _isReadyToSetData = false;
            _setDataExpireScheduled = false;
            _setDataSolutionPending = false;
            _workersCommitted = false;
        }

        Message = string.IsNullOrEmpty(message) ? string.Empty : message;
        OnDisplayExpired(true);
    }

    private void ClearIdleMessage()
    {
        lock (_lifecycleLock)
        {
            if (_isRunning || _isReadyToSetData || Tasks.Count > 0)
                return;
        }

        Message = string.Empty;
        OnDisplayExpired(true);
    }

    private void ClearWorkerErrors()
    {
        while (_workerErrors.TryDequeue(out _))
        {
        }
    }

    protected virtual string FormatProgressMessage(double fraction) =>
        fraction switch
        {
            >= 0.999 => "Done",
            <= 0.001 => "Planning…",
            _ => $"Planning… {(fraction * 100):0}%"
        };

    private void DisplayProgress(object? sender, System.Timers.ElapsedEventArgs e)
    {
        int workerCount;
        lock (_lifecycleLock)
            workerCount = Workers.Count;

        if (workerCount == 0 || ProgressReports.IsEmpty)
            return;

        double fraction;
        if (workerCount == 1)
            fraction = ProgressReports.Values.Last();
        else
        {
            var total = 0.0;
            foreach (var kvp in ProgressReports)
                total += kvp.Value;
            fraction = total / workerCount;
        }

        Message = FormatProgressMessage(fraction);
        RhinoApp.InvokeOnUiThread((Action)(() => OnDisplayExpired(true)));
    }
}

/// <summary>Optional marker for workers that collected inputs but should not run.</summary>
internal interface IWorkerSkip
{
    bool SkipWork { get; }
}
