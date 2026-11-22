//! Workspace verification gate runner powered by `craft-xtask`.
//!
//! Usage: `cargo run -p xtask -- check`

use std::path::Path;

use craft_xtask::runner::{self, RunOptions, Step};

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

    let mut steps: Vec<Step> = Vec::new();

    if cmd == "check" || cmd == "no-std" {
        steps.push(Step::new("no_std-check", |log| {
            let mut c = runner::cargo(&root);
            c.args([
                "check",
                "-p",
                "humanoid-proto",
                "-p",
                "humanoid-control",
                "-p",
                "humanoid-cu-nodes",
                "--no-default-features",
            ]);
            runner::run(
                c,
                log.command("cargo check -p humanoid-proto -p humanoid-control -p humanoid-cu-nodes --no-default-features"),
            )
        }));
    }

    if cmd == "check" || cmd == "test" {
        steps.push(Step::new("workspace-tests", |log| {
            let mut c = runner::cargo(&root);
            c.args([
                "test",
                "-p",
                "humanoid-proto",
                "-p",
                "humanoid-control",
                "-p",
                "humanoid-cu-nodes",
                "-p",
                "humanoid-sim",
                "-p",
                "humanoid-model",
                "-p",
                "humanoid-link",
            ]);
            runner::run(
                c,
                log.command("cargo test -p humanoid-proto -p humanoid-control -p humanoid-cu-nodes -p humanoid-sim -p humanoid-model -p humanoid-link"),
            )
        }));
    }

    if cmd == "check" || cmd == "wasm" {
        steps.push(Step::new("wasm32-check", |log| {
            let mut c = runner::cargo(&root);
            c.args([
                "check",
                "-p",
                "humanoid-web",
                "--target",
                "wasm32-unknown-unknown",
            ]);
            runner::run(
                c,
                log.command("cargo check -p humanoid-web --target wasm32-unknown-unknown"),
            )
        }));
    }

    if cmd == "check" || cmd == "firmware" {
        steps.push(Step::new("fw-core-tests", |log| {
            let mut c = runner::cargo(&root);
            c.args([
                "test",
                "--manifest-path",
                "firmware/humanoid-fw-core/Cargo.toml",
            ]);
            runner::run(
                c,
                log.command("cargo test --manifest-path firmware/humanoid-fw-core/Cargo.toml"),
            )
        }));

        steps.push(Step::new("rp2350-usb-cdc-check", |log| {
            let mut c = runner::cargo(&root);
            c.args([
                "check",
                "--manifest-path",
                "firmware/humanoid-rp2350/Cargo.toml",
                "--target",
                "thumbv8m.main-none-eabihf",
            ]);
            runner::run(
                c,
                log.command("cargo check --manifest-path firmware/humanoid-rp2350/Cargo.toml --target thumbv8m.main-none-eabihf"),
            )
        }));

        steps.push(Step::new("rp2350-usb-net-check", |log| {
            let mut c = runner::cargo(&root);
            c.args([
                "check",
                "--manifest-path",
                "firmware/humanoid-rp2350/Cargo.toml",
                "--target",
                "thumbv8m.main-none-eabihf",
                "--no-default-features",
                "--features",
                "usb-net,bench-loopback",
            ]);
            runner::run(
                c,
                log.command("cargo check --manifest-path firmware/humanoid-rp2350/Cargo.toml --target thumbv8m.main-none-eabihf --no-default-features --features usb-net,bench-loopback"),
            )
        }));
    }

    runner::run_steps(&mut steps, &RunOptions::default()).map(|_| ())
}
