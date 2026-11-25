# Proposal

## Why

RockyControl's servo communication uses a bespoke 16-byte command / 12-byte feedback frame over I3C with a per-call codec (`i3c_bus.rs`), which cannot be validated end-to-end in simulation and has no shared vocabulary with the BareFOC servo firmware. `barefoc-master` (a regular, dependency-light member of the sibling bareFOC repo) already provides the exact register-pointer master client (`Transport`, `I2cTransport`, `ControlTableClient`, canonical register constants) matching BareFOC's multibus register server. Adopting it gives us one servo protocol shared by firmware, simulator, and the real servo firmware — and lets simulation exercise the actual wire encoding/decoding.

## What Changes

- **BREAKING (servo wire format):** Replace the custom 16B/12B I3C frames with BareFOC register-pointer transactions (2-byte BE pointer + LE payload; burst reads) using `barefoc-master` as a read-only path dependency. No upstream bareFOC code is modified.
- Add a new `rk-multibus` workspace crate: `no_std` translation layer mapping `RawServoCommand`/`RawServoFeedback` ↔ BareFOC control-table registers (`0x120` torque, `0x130` goal position, `0x134` goal velocity, `0x138` goal current, `0x144/0x148/0x14C` present position/velocity/current, plus an agreed temperature offset), per-joint node addressing (`0x20 + joint`), and bus-health bookkeeping (ported timeout/degraded logic from `i3c_bus.rs`).
- Provide an `embedded_hal::i2c::I2c` adapter over the external RP2350 PIO I3C driver boundary so `barefoc-master::I2cTransport` works unchanged; DAA/CCC stays in the external driver.
- Replace `i3c_simplefoc.rs`'s transaction loop with `ControlTableClient` register reads/writes; delete the old codec (`pack_i3c_joint_cmd`, `unpack_i3c_joint_reply`, `simple_crc8`) from `rk-fw-core/i3c_bus.rs`; keep bus-health/timeout semantics.
- Route the simulator's `ServoBusPort` through the same `rk-multibus` client against an in-memory `RegMap`/`RegServer` (or mock `I2c`) so sim and firmware exercise identical encoding, and the plant publishes/consumes the same registers.
- Assume servo nodes will be flashed with matching bareFOC multibus firmware (deployment step outside RockyControl); this change does not modify upstream servo firmware.

## Capabilities

### New Capabilities
- `multibus-servo-transport`: Servo command/feedback exchange via BareFOC register-pointer transactions, shared verbatim between firmware and simulation.

### Modified Capabilities
- `unified-control-application`: the servo I/O backend changes from a bespoke codec to the shared multibus client.
- `persistent-simulation-executor`, `embassy-sim-runtime`: simulator bus path now uses real wire encoding.

## Impact

- New crate `crates/rk-multibus`; path dependency `barefoc-master = { path = "../bareFOC/barefoc-master" }` (root bareFOC repo, not a submodule, zero path deps).
- Changes: `rk-fw-core` (i3c_bus), `rk-rp2350` (i3c_simplefoc, servo_uart), `rk-sim` (plant bus), root workspace manifests, firmware README, openspec archives referencing the old codec.
- Risks: servo-side register profile/offsets must match (documented assumption); external PIO I3C adapter must implement `embedded-hal::i2c::I2c` correctly; defmt versions differ (keep barefoc `defmt` feature off); 21 joints vs bareFOC 4-axes/node → one node per joint (`0x20+joint`).
