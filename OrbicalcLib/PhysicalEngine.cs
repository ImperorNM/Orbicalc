using OrbicalcLib.Data;
using OrbicalcLib.Interfaces;
using OrbicalcLib.Util;
using PeterO.Numbers;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;

namespace OrbicalcLib
{
    public class PhysicalEngine
    {
        public int TargetUps
        {
            get => _targetUps;
            set
            {
                if (value <= 0)
                {
                    throw new ArgumentOutOfRangeException(nameof(value), "UPS <= 0");
                }
                _targetUps = value;
            }
        }

        public TimeSpan FixedDeltaTime => _fixedDeltaTime;
        public TimeSpan DeltaTime { get; private set; }
        public bool IsRunning { get; private set; }
        public long TotalTicks => Interlocked.Read(ref _totalTicks);

        public double CurrentUPS { get; private set; } = 60.0;
        public EDecimal TotalSimulationTimeSeconds => EDecimal.FromDouble(_totalTicks * _fixedDeltaTime.TotalSeconds);

        private int _targetUps = 60;
        private TimeSpan _fixedDeltaTime = TimeSpan.FromSeconds(10000.0);
        private Thread? _physicsThread;
        private CancellationTokenSource? _cts;
        private readonly Lock _stateLock = new();
        private readonly IEngineModule[] _modules;
        private readonly ISingleObjectHandler? _spaceshipHandler;
        private long _totalTicks;

        private readonly List<ManeuverNode> _activeManeuvers = new();
        private readonly Lock _maneuverLock = new();

        private readonly Stopwatch _perfStopwatch = Stopwatch.StartNew();
        private long _upsCount;
        private double _lastUpsUpdate;

        public PhysicalEngine(ISingleObjectHandler? spaceshipHandler, params IEngineModule[] modules)
        {
            _spaceshipHandler = spaceshipHandler;
            _modules = modules;
        }

        public PhysicalEngine(params IEngineModule[] modules) : this(null, modules) { }

        public void SetManeuvers(IEnumerable<ManeuverNode> nodes)
        {
            lock (_maneuverLock)
            {
                _activeManeuvers.Clear();
                _activeManeuvers.AddRange(nodes);
            }
        }

        public void StepSimulation()
        {
            long currentTick = _totalTicks;

            lock (_maneuverLock)
            {
                for (int i = 0; i < _activeManeuvers.Count; i++)
                {
                    var node = _activeManeuvers[i];

                    if (!node.Executed && currentTick >= node.TargetTick)
                    {
                        node.Executed = true;
                        _activeManeuvers[i] = node;

                        if (_spaceshipHandler?.Object != null)
                        {
                            var ship = _spaceshipHandler.Object;
                            var newVelocity = ship.Velocity + node.DeltaV;
                            _spaceshipHandler.Object = new SimObject(ship.Position, newVelocity, ship.Mass, ship.Size);
                        }
                    }
                }
            }

            Update(FixedDeltaTime);
            Interlocked.Increment(ref _totalTicks);

            _upsCount++;
            double elapsedSeconds = _perfStopwatch.Elapsed.TotalSeconds - _lastUpsUpdate;
            if (elapsedSeconds >= 0.5)
            {
                CurrentUPS = _upsCount / elapsedSeconds;
                _upsCount = 0;
                _lastUpsUpdate = _perfStopwatch.Elapsed.TotalSeconds;
            }
        }

        public void Start()
        {
            lock (_stateLock)
            {
                if (IsRunning) return;

                _cts = new CancellationTokenSource();
                IsRunning = true;

                _physicsThread = new Thread(() => EngineLoop(_cts.Token))
                {
                    Name = "PhysicsLoopThread",
                    IsBackground = true,
                };

                _physicsThread.Start();
            }
        }

        public void Stop()
        {
            CancellationTokenSource? cts;
            Thread? thread;

            lock (_stateLock)
            {
                if (!IsRunning) return;

                IsRunning = false;
                cts = _cts;
                thread = _physicsThread;

                _cts = null;
                _physicsThread = null;
            }

            cts?.Cancel();
            thread?.Join();
            cts?.Dispose();
        }

        private void EngineLoop(CancellationToken token)
        {
            var stopwatch = Stopwatch.StartNew();

            while (!token.IsCancellationRequested)
            {
                long frameStartTicks = stopwatch.ElapsedTicks;

                StepSimulation();

                double realTargetSeconds = 1.0 / TargetUps;
                double targetTicks = realTargetSeconds * Stopwatch.Frequency;

                while (stopwatch.ElapsedTicks - frameStartTicks < targetTicks)
                {
                    if (token.IsCancellationRequested) return;

                    double remainingMs = (targetTicks - (stopwatch.ElapsedTicks - frameStartTicks)) * 1000.0 / Stopwatch.Frequency;
                    if (remainingMs > 2.0)
                        Thread.Sleep(1);
                    else
                        Thread.SpinWait(10);
                }
            }
        }

        private void Update(TimeSpan deltaTime)
        {
            foreach (var module in _modules)
            {
                module.Update(deltaTime, FixedDeltaTime);
            }
        }
    }
}