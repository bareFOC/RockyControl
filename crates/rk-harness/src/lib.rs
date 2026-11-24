//! Deterministic Embassy executor harness for native tests and WASM simulation.
//!
//! The harness drives [`embassy_executor::raw::Executor`] by hand: simulated
//! time is advanced in fixed 5 ms steps via [`embassy_time::MockDriver`] and the
//! executor is polled to quiescence after each advance. No wall clock, render
//! frame rate, or real-time scheduler is involved, so replay is reproducible.
//!
//! This crate is host/WASM-only. It deliberately enables
//! `embassy-time/mock-driver` (which forces `tick-hz-1_000_000`) and does **not**
//! enable any Embassy platform feature. The MCU firmware lives in a separate
//! Cargo workspace and keeps its own time driver.

use core::sync::atomic::{AtomicUsize, Ordering};

use embassy_executor::{Spawner, raw};
use embassy_time::{Duration, Instant, MockDriver};

/// Fixed simulated control period (200 Hz). One [`DeterministicHarness::step`]
/// advances simulated time by exactly this much.
pub const TICK_PERIOD: Duration = Duration::from_millis(5);

/// Default upper bound on executor polls per quiescence loop. A task that
/// re-wakes itself without ever awaiting a timer will hit this cap instead of
/// hanging the simulation.
pub const DEFAULT_MAX_POLLS_PER_STEP: usize = 1024;

/// Counts pender invocations so `poll_to_quiescence` can tell whether a poll
/// enqueued more work. The pender itself never calls `poll()`, so it cannot
/// violate the raw executor's no-reentrant-poll requirement.
static PEND_COUNT: AtomicUsize = AtomicUsize::new(0);

/// The raw executor requires a pender symbol when no platform feature is
/// enabled. Embassy 0.10 has no `pender_impl!` macro; the pender is simply an
/// `extern` function named `__pender` (see `embassy_executor::raw` docs).
#[unsafe(export_name = "__pender")]
fn pender(_context: *mut ()) {
    PEND_COUNT.fetch_add(1, Ordering::Relaxed);
}

/// A deterministic, manually-clocked Embassy executor.
///
/// Create one with [`DeterministicHarness::new`], spawn tasks with
/// [`DeterministicHarness::spawner`], then call [`DeterministicHarness::step`]
/// (or [`DeterministicHarness::step_n`]) to run the simulation.
pub struct DeterministicHarness {
    executor: &'static raw::Executor,
    max_polls_per_step: usize,
    started: bool,
}

impl Default for DeterministicHarness {
    fn default() -> Self {
        Self::new()
    }
}

impl DeterministicHarness {
    /// Create a harness with a heap-leaked raw executor and the default poll cap.
    pub fn new() -> Self {
        Self::with_max_polls(DEFAULT_MAX_POLLS_PER_STEP)
    }

    /// Create a harness with a custom poll cap.
    pub fn with_max_polls(max_polls_per_step: usize) -> Self {
        assert!(max_polls_per_step > 0, "poll cap must be positive");
        let executor = Box::leak(Box::new(raw::Executor::new(core::ptr::null_mut())));
        Self {
            executor,
            max_polls_per_step,
            started: false,
        }
    }

    /// Build a harness around a caller-owned, statically allocated executor.
    ///
    /// Useful when the simulation must avoid heap allocation (e.g. a global
    /// harness in a WASM module):
    ///
    /// ```ignore
    /// static EXECUTOR: StaticCell<raw::Executor> = StaticCell::new();
    /// let harness = DeterministicHarness::from_executor(EXECUTOR.init(raw::Executor::new(core::ptr::null_mut())));
    /// ```
    pub fn from_executor(executor: &'static raw::Executor) -> Self {
        Self {
            executor,
            max_polls_per_step: DEFAULT_MAX_POLLS_PER_STEP,
            started: false,
        }
    }

    /// Spawner for this harness's executor.
    pub fn spawner(&self) -> Spawner {
        self.executor.spawner()
    }

    /// Poll all queued tasks once at simulated time zero without advancing the
    /// clock. This lets newly spawned tasks register their first timer so the
    /// very first `step()` produces a tick. Called automatically by `step()`.
    pub fn start(&mut self) {
        if self.started {
            return;
        }
        self.started = true;
        self.poll_to_quiescence();
    }

    /// Advance simulated time by [`TICK_PERIOD`] and poll to quiescence.
    ///
    /// Returns the number of executor polls performed (useful for detecting a
    /// runaway task; bounded by the configured poll cap).
    pub fn step(&mut self) -> usize {
        self.start();
        MockDriver::get().advance(TICK_PERIOD);
        self.poll_to_quiescence()
    }

    /// Run `n` deterministic steps. Returns the total number of executor polls.
    pub fn step_n(&mut self, n: usize) -> usize {
        (0..n).map(|_| self.step()).sum()
    }

    /// Whether the executor has been started (first ticker registration done).
    ///
    /// Clock resets are only safe before the first `step()`/`start()`: once a
    /// task has registered an alarm against the process-global `MockDriver`,
    /// resetting the clock strands that alarm and its waker forever.
    pub fn is_started(&self) -> bool {
        self.started
    }

    /// Current simulated time.
    pub fn now(&self) -> Instant {
        Instant::now()
    }

    /// Reset simulated time and all registered alarms to zero.
    ///
    /// Callers should also re-spawn or reset their own task state; this only
    /// resets the mock time driver.
    pub fn reset_clock(&self) {
        MockDriver::get().reset();
    }

    fn poll_to_quiescence(&self) -> usize {
        let mut polls = 0;
        loop {
            let before = PEND_COUNT.load(Ordering::Relaxed);
            // SAFETY: `step`/`start` take `&mut self`, so there is exactly one
            // owner driving this executor. The pender only bumps a counter and
            // never calls `poll()`, so polling is never reentrant.
            unsafe { self.executor.poll() };
            polls += 1;
            if PEND_COUNT.load(Ordering::Relaxed) == before {
                return polls;
            }
            assert!(
                polls < self.max_polls_per_step,
                "Embassy executor did not quiesce after {polls} polls (cap {})",
                self.max_polls_per_step
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::AtomicUsize;
    use embassy_executor::task;
    use embassy_time::Ticker;
    use std::sync::Mutex;

    static TEST_LOCK: Mutex<()> = Mutex::new(());
    static TICKS: AtomicUsize = AtomicUsize::new(0);

    #[task]
    async fn counting_task() {
        let mut ticker = Ticker::every(TICK_PERIOD);
        loop {
            ticker.next().await;
            TICKS.fetch_add(1, Ordering::Relaxed);
        }
    }

    #[test]
    fn ten_steps_are_exactly_ten_ticks() {
        let _guard = TEST_LOCK.lock().unwrap();
        MockDriver::get().reset();
        TICKS.store(0, Ordering::Relaxed);

        let mut harness = DeterministicHarness::new();
        harness.spawner().spawn(counting_task().unwrap());
        harness.step_n(10);

        assert_eq!(TICKS.load(Ordering::Relaxed), 10);
        assert_eq!(harness.now(), Instant::from_millis(50));
    }
}
