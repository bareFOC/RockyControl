# Design

## Context

`DeterministicHarness::new()` + task spawn currently happens per `step_fixed` call. `MockDriver` is process-global, so stale wakers corrupt the queue. Asset the executor once per `SimTransport` lifetime.

## Decisions
1. `SimRobotIo` owns a leaked `DeterministicHarness` + leaked `raw::TaskStorage<Pin<Box<dyn Future>>>` + the IO in one leaked box; task spawned once at construction.
2. Host commands queued into `Arc<Mutex<VecDeque<HostToMcu>>>`; telemetry out via `Arc<Mutex<Option<McuToHost>>>`; recorded ticks via existing `record_tick` on the IO.
3. `step_fixed(steps, pkt, frame_dt)` enqueues pkt, advances mock clock by steps×5ms via the persistent harness, drains telemetry.
4. Drop reclaims boxes; no re-spawn.
5. Keep `tick()` compat path using the same persistent IO where practical.
