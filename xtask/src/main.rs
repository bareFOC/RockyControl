//! Workspace verification gate runner powered by `craft-xtask`.
//!
//! Usage:
//!   cargo run -p xtask -- check           # full gate (identical to `sim-verify`)
//!   cargo run -p xtask -- sim-verify      # simulation acceptance gate
//!   cargo run -p xtask -- no-std          # no_std core crates
//!   cargo run -p xtask -- test            # host tests
//!   cargo run -p xtask -- wasm            # WASM cross-check
//!   cargo run -p xtask -- firmware        # firmware-core tests + RP2350 matrix
//!   cargo run -p xtask -- experiments     # committed MCP headless experiments

mod codegen;

use std::path::Path;

use craft_xtask::runner::{self, RunOptions, Step};

const RP2350_MANIFEST: &str = "firmware/rk-rp2350/Cargo.toml";
const THUMB_TARGET: &str = "thumbv8m.main-none-eabihf";

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let cmd = args.first().map(String::as_str).unwrap_or("check");

    if let Err(err) = run_gate(cmd) {
        eprintln!("xtask '{cmd}' failed: {err:?}");
        std::process::exit(1);
    }
}

fn run_gate(cmd: &str) -> Result<(), runner::Error> {
    let root = runner::root(Path::new(env!("CARGO_MANIFEST_DIR")))?;
    if cmd == "codegen-robots" {
        return codegen::run_codegen(&root);
    }
    let full_gate = matches!(cmd, "check" | "sim-verify");

    if !full_gate
        && !matches!(cmd, "no-std" | "test" | "wasm" | "firmware" | "experiments")
    {
        return Err(runner::Error::Config(format!(
            "unknown gate '{cmd}': expected check, sim-verify, no-std, test, wasm, firmware or experiments"
        )));
    }

    let mut steps: Vec<Step> = Vec::new();

    if full_gate || cmd == "no-std" {
        steps.push(Step::new("no-std-check", |log| {
            let mut c = runner::cargo(&root);
            c.args([
                "check",
                "-p",
                "rk-proto",
                "-p",
                "rk-control",
                "-p",
                "rk-multibus",
                "--no-default-features",
            ]);
            runner::run(
                c,
                log.command("cargo check -p rk-proto -p rk-control -p rk-multibus --no-default-features"),
            )
        }));
    }

    if full_gate || cmd == "test" {
        steps.push(Step::new("workspace-tests", |log| {
            let mut c = runner::cargo(&root);
            c.args(["test", "--workspace"]);
            runner::run(c, log.command("cargo test --workspace"))
        }));
    }

    if full_gate || cmd == "wasm" {
        steps.push(Step::new("wasm32-check", |log| {
            let mut c = runner::cargo(&root);
            c.args(["check", "-p", "rk-ui", "--target", "wasm32-unknown-unknown"]);
            runner::run(
                c,
                log.command("cargo check -p rk-ui --target wasm32-unknown-unknown"),
            )
        }));
    }

    if full_gate || cmd == "firmware" {
        steps.push(Step::new("fw-core-tests", |log| {
            let mut c = runner::cargo(&root);
            c.args([
                "test",
                "--manifest-path",
                "firmware/Cargo.toml",
                "-p",
                "rk-fw-core",
            ]);
            runner::run(
                c,
                log.command("cargo test --manifest-path firmware/Cargo.toml -p rk-fw-core"),
            )
        }));

        // Default bench bring-up build (USB CDC-ACM + loopback servo plant).
        steps.push(Step::new("rp2350-default-check", |log| {
            let mut c = runner::cargo(&root);
            c.args([
                "check",
                "--manifest-path",
                RP2350_MANIFEST,
                "--target",
                THUMB_TARGET,
            ]);
            runner::run(
                c,
                log.command(&format!(
                    "cargo check --manifest-path {RP2350_MANIFEST} --target {THUMB_TARGET}"
                )),
            )
        }));

        // USB-Net + real servo bus + LSM6DSV IMU: the feature set closest to the
        // intended v1 hardware. The crate spells the IMU feature `imu-lsm6dsv`.
        steps.push(Step::new("rp2350-hardware-features-check", |log| {
            let mut c = runner::cargo(&root);
            c.args([
                "check",
                "--manifest-path",
                RP2350_MANIFEST,
                "--target",
                THUMB_TARGET,
                "--no-default-features",
                "--features",
                "usb-net,i3c-servo-bus,imu-lsm6dsv,robot-humanoid",
            ]);
            runner::run(
                c,
                log.command(&format!(
                    "cargo check --manifest-path {RP2350_MANIFEST} --target {THUMB_TARGET} --no-default-features --features usb-net,i3c-servo-bus,imu-lsm6dsv,robot-humanoid"
                )),
            )
        }));

        // Keep the legacy UDP-bench cross-check used by README and CI.
        steps.push(Step::new("rp2350-usb-net-check", |log| {
            let mut c = runner::cargo(&root);
            c.args([
                "check",
                "--manifest-path",
                RP2350_MANIFEST,
                "--target",
                THUMB_TARGET,
                "--no-default-features",
                "--features",
                "usb-net,bench-loopback,robot-humanoid",
            ]);
            runner::run(
                c,
                log.command(&format!(
                    "cargo check --manifest-path {RP2350_MANIFEST} --target {THUMB_TARGET} --no-default-features --features usb-net,bench-loopback,robot-humanoid"
                )),
            )
        }));
    }

    if full_gate || cmd == "experiments" {
        // Committed experiment scripts are loaded through the MCP headless host's
        // confined-file `run_experiment` path. The same tests re-run each script
        // through `CopperSimRunner` to assert the public walk-quality bounds and
        // verify the committed golden `.copper` log bit-exactly.
        steps.push(Step::new("mcp-headless-experiments", |log| {
            let mut c = runner::cargo(&root);
            c.args([
                "test",
                "-p",
                "rk-mcp",
                "--lib",
                "acceptance_experiments",
                "--",
                "--nocapture",
            ]);
            runner::run(
                c,
                log.command("cargo test -p rk-mcp --lib acceptance_experiments -- --nocapture"),
            )
        }));
    }

    runner::run_steps(&mut steps, &RunOptions::default()).map(|_| ())
}
