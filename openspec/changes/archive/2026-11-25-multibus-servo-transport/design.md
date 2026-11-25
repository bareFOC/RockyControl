# Design

## Context

`barefoc-master` is a regular member of the bareFOC root repo (not a submodule), has zero path dependencies, depends only on `embedded-hal 1.0`/`embedded-io`/`heapless`, and provides `Transport`, `I2cTransport` (2-byte BE pointer + LE burst), `ControlTableClient<T>` (`read_f32`/`write_f32`), and canonical register constants. RockyControl currently uses a bespoke 16B/12B I3C frame with CRC8 in `rk-fw-core/i3c_bus.rs` and a per-joint transaction loop in `rk-rp2350/i3c_simplefoc.rs`; the simulator bypasses wire encoding entirely. `defmt` major versions differ (bareFOC 0.3 vs Rocky 1.0), so bareFOC defmt must stay off.

## Goals / Non-Goals

**Goals:** one servo wire format shared firmware/sim/bareFOC servos; simulator validates real wire encoding; preserve bus-health isolation; read-only upstream deps.

**Non-Goals:** modifying bareFOC source; flashing physical servos; I3C DAA/IBI (stays in external PIO driver); multi-axis-per-node (use one node per joint).

## Decisions

1. New `crates/rk-multibus` (`no_std`, workspace member): depends on `barefoc-master` via path `../bareFOC/barefoc-master` (no submodule risk), plus `rk-proto`/`rk-control`, `embedded-hal 1.0`. Do NOT enable barefoc `defmt` or profile features.
2. `rk-multibus/src/`: `lib.rs` re-exports; `joint_map.rs` (node addr `0x20+joint`, register↔`RawServoCommand/Feedback` mapping, unit conversions); `transport.rs` (`embedded_hal::i2c::I2c` adapter over the external PIO transaction boundary, error mapping); `health.rs` (port timeout/degraded logic from `i3c_bus.rs:107-209`, drop CRC/pack/unpack).
3. Firmware: `rk-rp2350/i3c_simplefoc.rs` keeps the 3-phase critical-section discipline and per-joint degraded accounting, but the loop body becomes `ControlTableClient::write_f32/read_f32` register accesses through the I2C adapter. Delete old codec from `i3c_bus.rs`; keep bus-health.
4. Simulator: `ServoBusPort` routes through `rk-multibus` against an in-memory `RegMap`/`RegServer` (or `embedded-hal-mock` I2C) so sim exercises the same register transactions; the plant publishes/consumes the same registers instead of raw struct fields.
5. Register profile assumption: canonical bareFOC offsets `0x120` torque-enable, `0x130` goal position, `0x134` goal velocity, `0x138` goal current, `0x144/0x148/0x14C` present pos/vel/current. Temperature is NOT in `barefoc-master` registers — define locally in `joint_map.rs` at canonical firmware offset `0x142` (u16 °C) / `0x16A`/`0x16E` (f32) and document for servo firmware. Use one node per joint (`0x20+joint`, range 0x20..=0x34) with REG_GOAL_POSITION directly — do NOT use `FishyServoNode` joint slots at `0x16C+i*4` (they collide with canonical thermal registers 0x16E/0x172/0x176). Ignore `sync.rs`/`scanner.rs` (16-node caps); `ControlTableClient` has no node cap. Servo nodes must run the canonical 512-byte `control_table_profile()` — the fishyservo example currently exposes the 384-byte SimpleFOC `RegisterId` map (offsets 0x00..0xBF), so reflashing with a canonical server is mandatory (deployment step, not this change). `REG_GOAL_CURRENT`/`REG_PRESENT_CURRENT` are f32 amperes; convert to/from Rocky's N·m and u16 mA via a documented torque-constant config (no KV register in the master crate).

## Risks / Trade-offs

- Servo firmware offsets must match — documented assumption; hardware validation later.
- External PIO I3C adapter must implement `embedded-hal::i2c::I2c` correctly (write/read); `I2cTransport::read` does two separate HAL calls (pointer latch then burst) so pointer state must survive across transactions; DAA/CCC and `send_packet` (broadcast 0xFE, invalid 7-bit I2C) are out of scope.
- `rk-multibus` path dep requires sibling bareFOC checkout — document in README; CI must have it present.
- Wire-format change breaks old servos until reflashed (accepted; no back-compat needed).
