# Repository Guidance for Agents

## Decision-making principles

- **Long-term code maintainability is the priority.** When choosing between options, prefer the design that is easiest to understand, modify, and verify over months of development — even if it requires a larger one-time refactor.
- **No backward compatibility is required.** APIs, wire formats, module layouts, crate boundaries, and artifact names may all be changed freely. Do not preserve deprecated interfaces, aliases, or compatibility shims solely for compatibility's sake. Remove dead code rather than keeping it "just in case".
- **Prefer a single source of truth.** Eliminate duplicated logic between the firmware and simulator. Code that computes the same result in two places should be unified so that simulation validates the exact code path that runs on hardware.

## Architecture context

- This repository is a pure-Rust humanoid robot control workstation + firmware monorepo ("Rocky Studio") for a 1/4-scale Dummy 13-derived humanoid ("Rocky Mk1").
- The real-time control kernel (`crates/rk-control::DirectServoKernel`) is `no_std`, `f32`-only, deterministic, and is the single most important piece of code. Keep it hardware-agnostic.
- Host crates: `rk-proto` (wire), `rk-control` (kernel), `rk-model` (URDF/action studio), `rk-sim` (software simulation), `rk-link` (transport), `rk-viewport` (3D), `rk-ui` (egui app), `rk-mcp` (MCP server).
- Firmware lives in a separate Cargo workspace under `firmware/`: `rk-fw-core` (shared control application), `rk-rp2350` (v1 board), `rk-esp32p4` (v2 scaffold).
- The MCU firmware is a pure Embassy application (no copper-rs). The simulator is a software-only twin: a fixed-dt synchronous harness that reuses the shared control code, with a physics plant and in-memory transport as I/O backends.
- Determinism, fixed 200 Hz ticks, `f32` numeric parity, and replay verification are core requirements. Avoid introducing nondeterministic runtime timing into the replay-verified control path.
- No camera at runtime. Available sensing: joint position + velocity + current (SimpleFOC over I3C on v1, custom current loop on N20) and a 3-axis IMU (LSM6DSV16X primary on v1). Contact is inferred from motor current + IMU, not dedicated foot sensors.
- No runtime host PC and no vision. The PC is used **offline only** for training, optimization, and tooling.

## Conventions

- Use OpenSpec for planning non-trivial changes (propose → apply → sync/archive).
- Delegate independent work to subagents with clear file ownership; no concurrent edits to the same file.
- Verify with `cargo test` across host crates, `cargo test --manifest-path firmware/Cargo.toml -p rk-fw-core`, and thumb-target `cargo check` for RP2350 feature sets before declaring work complete.
- Keep changes focused and avoid adding dependencies unless clearly justified.
