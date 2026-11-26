//! Runtime-agnostic async control task shared by firmware and simulation.
//!
//! This module deliberately has **no** `embassy-time` (or any Embassy) dependency:
//! tick timing, the monotonic clock and all I/O are supplied by the [`RobotIo`]
//! implementation provided by the backend. The RP2350 board implements it with an
//! Embassy `Ticker` / `Instant` and its mailboxes; the deterministic simulator
//! implements it with the fixed-dt Embassy harness and its mock clock.
//!
//! One single control loop body ([`control_task`] + [`step_control_tick`]) therefore
//! runs identically on hardware and in replay-verified simulation. Boards only wrap
//! [`control_task`] in their own 3-line `#[embassy_executor::task]`; they never
//! duplicate kernel scheduling or I/O fan-out logic.

use core::future::Future;

use rk_control::{JointCalibration, RawServoCommand, RawServoFeedback};
use rk_proto::{BalanceGains, HostToMcu, ImpactWireConfig, MAX_JOINTS, McuToHost, UvcGaitConfig};

use crate::config::{CONTROL_DT_SEC, DEFAULT_JOINT_COUNT};
use crate::copper::{ControlTask, ControlTick};

/// All board- / harness-specific I/O required by one deterministic control tick.
///
/// Implementations must never read a wall clock: [`Self::now_ms`] and
/// [`Self::wait_tick`] must both be driven by the same deterministic time source
/// (Embassy time on the MCU, the mock clock in simulation).
pub trait RobotIo {
    /// Waits until the next fixed-rate control tick is due.
    ///
    /// Firmware implements this with `embassy_time::Ticker::next`; the simulator
    /// with a timer driven by the deterministic harness.
    fn wait_tick(&mut self) -> impl Future<Output = ()>;

    /// Monotonic time in milliseconds at the current tick.
    fn now_ms(&self) -> u32;

    /// Fixed control step duration in seconds (200 Hz / 5 ms by default).
    fn dt_sec(&self) -> f32 {
        CONTROL_DT_SEC
    }

    /// Robot profile used to construct the shared kernel.
    fn profile(&self) -> &'static rk_robot_profile::RobotProfile {
        &rk_robot_profile::generated::humanoid::PROFILE
    }

    /// Number of joints used to construct the shared kernel.
    fn joint_count(&self) -> usize {
        DEFAULT_JOINT_COUNT
    }

    /// Per-joint calibration used to construct the shared kernel.
    fn calibration(&self) -> [JointCalibration; MAX_JOINTS] {
        [JointCalibration::default(); MAX_JOINTS]
    }

    /// Optional persisted balance gains loaded at boot.
    fn initial_balance_gains(&self) -> Option<BalanceGains> {
        None
    }

    /// Optional persisted UVC gait config loaded at boot.
    fn initial_uvc_gait(&self) -> Option<UvcGaitConfig> {
        None
    }

    /// Optional persisted impact config loaded at boot.
    fn initial_impact_config(&self) -> Option<ImpactWireConfig> {
        None
    }

    /// Called when `ConfigCmd::CommitFlash` increments `flash_commit_count`.
    fn on_flash_commit(&mut self, _control: &ControlTask) {}

    /// Takes the freshest pending host command (latest-wins), if any.
    fn take_cmd(&mut self) -> Option<HostToMcu>;

    /// Returns `(rx_crc_errors, rx_version_mismatches)` from the wire transport.
    fn wire_error_counts(&self) -> (u16, u16);

    /// Reads one servo + IMU feedback snapshot.
    fn read_feedback(&mut self, dt_sec: f32) -> RawServoFeedback;

    /// Applies one servo command to the plant / hardware bus.
    fn write_command(&mut self, cmd: &RawServoCommand);

    /// Publishes one host telemetry frame.
    fn publish_telemetry(&mut self, telem: McuToHost);

    /// Records one control tick for replay (default: no-op on firmware).
    fn record_tick(&mut self, _tick: &ControlTick) {}

    /// Board-specific work after a clean tick (default: no-op).
    ///
    /// The RP2350 board uses this hook for its status LED and to feed the 250 ms
    /// hardware watchdog, so that safety hardware stays in board code.
    fn post_tick(&mut self, _now_ms: u32) {}
}

/// Blanket forwarding so a borrowed IO can be handed to [`control_task`] directly
/// (used by the simulator harness, which lends its owned IO to the spawned task).
impl<T: RobotIo + ?Sized> RobotIo for &mut T {
    async fn wait_tick(&mut self) {
        (**self).wait_tick().await;
    }

    fn now_ms(&self) -> u32 {
        (**self).now_ms()
    }

    fn dt_sec(&self) -> f32 {
        (**self).dt_sec()
    }

    fn profile(&self) -> &'static rk_robot_profile::RobotProfile {
        (**self).profile()
    }

    fn joint_count(&self) -> usize {
        (**self).joint_count()
    }

    fn calibration(&self) -> [JointCalibration; MAX_JOINTS] {
        (**self).calibration()
    }

    fn initial_balance_gains(&self) -> Option<BalanceGains> {
        (**self).initial_balance_gains()
    }

    fn initial_uvc_gait(&self) -> Option<UvcGaitConfig> {
        (**self).initial_uvc_gait()
    }

    fn initial_impact_config(&self) -> Option<ImpactWireConfig> {
        (**self).initial_impact_config()
    }

    fn on_flash_commit(&mut self, control: &ControlTask) {
        (**self).on_flash_commit(control);
    }

    fn take_cmd(&mut self) -> Option<HostToMcu> {
        (**self).take_cmd()
    }

    fn wire_error_counts(&self) -> (u16, u16) {
        (**self).wire_error_counts()
    }

    fn read_feedback(&mut self, dt_sec: f32) -> RawServoFeedback {
        (**self).read_feedback(dt_sec)
    }

    fn write_command(&mut self, cmd: &RawServoCommand) {
        (**self).write_command(cmd);
    }

    fn publish_telemetry(&mut self, telem: McuToHost) {
        (**self).publish_telemetry(telem);
    }

    fn record_tick(&mut self, tick: &ControlTick) {
        (**self).record_tick(tick);
    }

    fn post_tick(&mut self, now_ms: u32) {
        (**self).post_tick(now_ms);
    }
}

/// Executes exactly one deterministic control tick through [`RobotIo`].
///
/// Both the async firmware/harness task and the simulator's synchronous
/// compatibility path call this function, so kernel stepping and I/O fan-out exist
/// in one place only.
pub fn step_control_tick<I: RobotIo>(io: &mut I, control: &mut ControlTask) {
    let now_ms = io.now_ms();
    let dt_sec = io.dt_sec();
    let cmd = io.take_cmd();
    let wire_errors = io.wire_error_counts();
    let fb = io.read_feedback(dt_sec);

    let prev_commits = control.kernel().flash_commit_count();
    let (step, tick) =
        control.step_recorded(now_ms, dt_sec, cmd.as_ref(), &fb, wire_errors);

    if control.kernel().flash_commit_count() != prev_commits {
        io.on_flash_commit(control);
    }
    io.write_command(&step.servo_cmd);
    io.publish_telemetry(step.telemetry);
    io.record_tick(&tick);
    io.post_tick(now_ms);
}

/// The single shared async control loop.
///
/// Wrapped by each backend in a thin `#[embassy_executor::task]`; all control logic
/// (kernel construction, fixed dt, tick fan-out) lives here.
pub async fn control_task<I: RobotIo>(mut io: I) {
    let mut control = ControlTask::new_with_profile(io.profile(), &io.calibration());
    if let Some(gains) = io.initial_balance_gains() {
        control.kernel_mut().set_balance_gains(gains);
    }
    if let Some(gait) = io.initial_uvc_gait() {
        control.kernel_mut().set_uvc_gait_config(gait);
    }
    if let Some(impact) = io.initial_impact_config() {
        control.kernel_mut().set_impact_wire_config(impact);
    }
    loop {
        io.wait_tick().await;
        step_control_tick(&mut io, &mut control);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rk_control::JointCalibration;
    use rk_proto::ControlMode;

    struct MockIo {
        now_ms: u32,
        cmd: Option<HostToMcu>,
        fb: RawServoFeedback,
        last_cmd: Option<RawServoCommand>,
        last_telem: Option<McuToHost>,
        recorded: Vec<ControlTick>,
        post_tick_ms: Option<u32>,
    }

    impl RobotIo for MockIo {
        async fn wait_tick(&mut self) {}
        fn now_ms(&self) -> u32 {
            self.now_ms
        }
        fn take_cmd(&mut self) -> Option<HostToMcu> {
            self.cmd.take()
        }
        fn wire_error_counts(&self) -> (u16, u16) {
            (2, 3)
        }
        fn read_feedback(&mut self, _dt_sec: f32) -> RawServoFeedback {
            self.fb
        }
        fn write_command(&mut self, cmd: &RawServoCommand) {
            self.last_cmd = Some(*cmd);
        }
        fn publish_telemetry(&mut self, telem: McuToHost) {
            self.last_telem = Some(telem);
        }
        fn record_tick(&mut self, tick: &ControlTick) {
            self.recorded.push(*tick);
        }
        fn post_tick(&mut self, now_ms: u32) {
            self.post_tick_ms = Some(now_ms);
        }
    }

    #[test]
    fn step_control_tick_fans_out_one_recorded_tick() {
        let cal = [JointCalibration::default(); MAX_JOINTS];
        let mut control = ControlTask::new(1, &cal);
        let cmd = HostToMcu {
            mode: ControlMode::Teach,
            ..Default::default()
        };
        let mut io = MockIo {
            now_ms: 125,
            cmd: Some(cmd),
            fb: RawServoFeedback::default(),
            last_cmd: None,
            last_telem: None,
            recorded: Vec::new(),
            post_tick_ms: None,
        };

        step_control_tick(&mut io, &mut control);

        assert_eq!(control.tick_index(), 1);
        assert_eq!(io.last_cmd, Some(io.recorded[0].tx_cmd));
        assert_eq!(io.last_telem, Some(io.recorded[0].tx_telem));
        assert_eq!(io.recorded[0].now_ms, 125);
        assert_eq!(io.recorded[0].dt_sec, CONTROL_DT_SEC);
        assert_eq!(io.recorded[0].wire_errors, (2, 3));
        assert_eq!(io.post_tick_ms, Some(125));
    }
}
