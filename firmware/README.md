# Rocky Humanoid MCU Firmware (`embassy-rs`)

Real-time `#![no_std]` humanoid control firmware built on **`embassy-rs`**, targeting **RP2350** first and architected for **ESP32-P4** expansion with zero control-logic duplication. A single Embassy scheduler owns all MCU work; `rk-fw-core` supplies a plain `ControlTask` that calls `DirectServoKernel::step()` directly — there is no copper-rs (`cu29`) runtime on either MCU.

> [!WARNING]
> **BREAKING telemetry wire-format change (fall-recovery):** `rk_proto::ImuState` now carries full orientation (`yaw_rad`, `yaw_rate_rad_s`, `gravity`) in every `McuToHost` frame. The COBS telemetry protocol has no version/CRC handshake, so the firmware image and the host workstation must be **rebuilt and flashed in lockstep**. A host and MCU running different `ImuState` layouts cannot communicate — update (or roll back) all sides together.

---

## 1. Layered Crate Architecture

```text
crates/rk-proto          (#![no_std])  HostToMcu / McuToHost, postcard COBS, f32
crates/rk-control        (#![no_std])  DirectServoKernel (100 ms watchdog, soft stops, slew clamp)
  ▲
firmware/rk-fw-core      (#![no_std])  Board-agnostic MCU core:
  │                                          - ControlTask: plain DirectServoKernel::step() wrapper
  │                                          - Zero-alloc CobsStreamFramer (USB CDC-ACM)
  │                                          - UDP COBS datagram codec (USB CDC-NCM networking)
  │                                          - BenchLoopbackBus (Day-1 USB-only testing) & ServoBusPort
  │                                          - Deterministic StatusLedMode (Teach / Drive / Fault)
  ├── firmware/rk-rp2350  (Primary)    RP2350 Cortex-M33F (thumbv8m.main-none-eabihf)
  │                                          embassy-rp 0.10 + embassy-usb 0.6 + embassy-net 0.9
  └── firmware/rk-esp32p4 (Next MCU)   ESP32-P4 RV32IMAFC (riscv32imafc-unknown-none-elf)
                                             esp-hal + esp-hal-embassy + USB-OTG HS 480 Mbps
```

### Embassy-Only Runtime (identical shape across both MCUs)

```text
embassy_time::Ticker (200 Hz)
        │
        ▼
  [ControlTask::step]  ──▶ servo command  ──▶ SERVO_BUS_SLOT / hardware bus task
        ▲                   └──▶ McuToHost  ──▶ HOST_LINK_MAILBOX / USB task
        │
   HOST_LINK_MAILBOX.take_rx_cmd()   (HostToMcu, latest-wins)
   SERVO_BUS_SLOT.read_feedback(dt)  (RawServoFeedback snapshot)
```

- **Loop Rate:** Deterministic `200 Hz` (`5 ms` `embassy_time::Ticker`) driving `ControlTask::step()` directly.
- **Numeric Parity:** Strictly single-precision `f32` everywhere (RP2350 Cortex-M33F & ESP32-P4 RV32IMAFC both have hardware single-precision FPUs).
- **Safety by Construction:**
  - Always boots in `Teach` mode (`torque_mask = 0`).
  - `100 ms` host deadman watchdog inside `DirectServoKernel` latches `EStop` and forces torque OFF on cable pull or GUI stall.
  - `250 ms` RP2350 hardware `Watchdog` resets the MCU if the control loop itself ever wedges.
  - Onboard LED (`GPIO25` on Pico 2): **1 Hz slow heartbeat** = `Teach`, **Solid ON** = `Drive`, **8 Hz rapid strobe** = `EStop` / Watchdog / Fault.
- **Deterministic replay** remains a host-side feature (`rk-cu-nodes::DirectServoTask`, `rk-sim`, UI, MCP). It is independent of the MCU runtime: no copper-rs code or `.copper` logging runs on the firmware.

---

## 2. Wired Bring-Up Modes (Phase 1 — Wired Only)

Both wired transports carry the **exact same `postcard` COBS-encoded `HostToMcu` / `McuToHost` frames**:

| Feature Flag | Transport | Host Setup | Best For |
| :--- | :--- | :--- | :--- |
| `--features usb-cdc,bench-loopback` *(default)* | **USB CDC-ACM Virtual Serial** (`VID 0x1209 / PID 0x524B`) | Plug in USB -> `/dev/ttyACM0` (Linux), `/dev/cu.usbmodem*` (macOS), `COMx` (Windows) | Fastest Day-1 plug-and-play bench test + browser WebSerial |
| `--no-default-features --features usb-net,bench-loopback` | **USB Networking (CDC-NCM Ethernet-over-USB)** (`VID 0x1209 / PID 0x524E`) | Assign host USB Ethernet adapter `10.42.0.1/24`; MCU listens on UDP `10.42.0.2:7777` (ICMP ping enabled) | High-rate telemetry, Wireshark packet capture, zero serial framing jitter |

### Bench Loopback vs. Real Servo Bus

- **`bench-loopback`** *(default)*: Runs `BenchLoopbackBus` directly inside the MCU so you can flash RP2350, plug in a **single USB cable**, and test live 200 Hz Teach/Drive mirroring, bumpless transfer, E-Stop, and cable-pull watchdog trip with zero servos powered.
- **`uart-servo-bus`** *(legacy, unused by the control application)*: Initializes RP2350 `UART0` (`GPIO0` TX / `GPIO1` RX @ `1 Mbps`) wired into `SERVO_BUS_SLOT` for physical Feetech STS3215 / Dynamixel Protocol 2.0 smart bus servos. `rk-multibus` is now the single servo wire path; this remains only as a bring-up placeholder.

> [!IMPORTANT]
> **`bench-loopback` hooks are overridden whenever a hardware servo bus is enabled.** Both `uart-servo-bus` and `i3c-servo-bus` register their own `ServoBusHooks` in `SERVO_BUS_SLOT`, so the `bench-loopback` mirroring plant is not active in those builds even if the `bench-loopback` feature is still present. The two hardware bus features are mutually exclusive and enabling both is a compile error.

### Opt-in High-Rate IMU & I3C Servo Bus Features

These are **off by default** — the default `bench-loopback` build is unchanged. Enable them explicitly for real-hardware bring-up:

| Feature | Hardware | What it wires |
| :--- | :--- | :--- |
| `imu-lsm6dsv` | ST LSM6DSV16X on dedicated **SPI1** (SCK `GPIO10`, MOSI `GPIO11`, MISO `GPIO8`, CS `GPIO9`) | 960 Hz hardware FIFO burst-read at 200 Hz, complementary filter publishes `ImuState` to `IMU_SLOT` tagged `LSM6DSV_OK`. **Mutually exclusive with `imu-bno055`** — the BNO055 task is not spawned when this is enabled (motor currents disturb its magnetometer). |
| `i3c-servo-bus` | RP2350 PIO I3C master (external driver, *not* implemented here) | Registers `ServoBusHooks` in `SERVO_BUS_SLOT` via `rk-multibus` / `barefoc-master`: one BareFOC node per joint (`0x20 + joint`), register-pointer writes to the canonical control table (`0x120` torque enable, `0xE0` operating mode, `0x130/0x134/0x138` goal position/velocity/current) and one 14-byte burst read at `0x142` for temperature/position/velocity/current. Per-joint timeout accounting and an edge-triggered `defmt::warn!` are preserved; the external PIO driver plugs in via `set_i3c_transport()` and a safe stub records timeouts until then. |

> [!IMPORTANT]
> **Servo firmware must be reflashed for `i3c-servo-bus`.** Each servo node must
> expose the canonical 512-byte BareFOC multibus control table at node
> `0x20 + joint` — the old 16-byte command / 12-byte feedback I3C frame codec is
> gone. Flashing the servos is a deployment step outside this repository. The
> register client is a read-only path dependency on the sibling
> `../bareFOC/barefoc-master` checkout, so that checkout must be present to
> build this feature.

Example — LSM6DSV + I3C hardware with USB CDC transport:

```bash
cargo build --release --manifest-path firmware/rk-rp2350/Cargo.toml \
  --features usb-cdc,imu-lsm6dsv,i3c-servo-bus
```

These features also work alongside `usb-net`. `i3c-servo-bus` and `uart-servo-bus` are mutually exclusive (enabling both is a compile error), as are `imu-bno055` and `imu-lsm6dsv`.

---

## 3. Building & Flashing RP2350

```bash
# 1. Install Cortex-M33F target once
rustup target add thumbv8m.main-none-eabihf

# 2. Run board-agnostic firmware core unit tests on host
cargo test --manifest-path firmware/rk-fw-core/Cargo.toml

# 3. Build RP2350 firmware — Mode A: Wired USB CDC-ACM (default)
cargo build --release --manifest-path firmware/rk-rp2350/Cargo.toml

# 4. Build RP2350 firmware — Mode B: Wired USB Networking (CDC-NCM Ethernet + UDP 7777)
cargo build --release --manifest-path firmware/rk-rp2350/Cargo.toml \
  --no-default-features --features usb-net,bench-loopback

# 5. Flash & view defmt RTT logs via probe-rs (SWD debug probe)
cargo run --release --manifest-path firmware/rk-rp2350/Cargo.toml
```

### USB Networking Quick Test (`usb-net`)

```bash
# On Linux host after plugging RP2350 USB cable:
sudo ip addr add 10.42.0.1/24 dev usb0
sudo ip link set usb0 up
ping -c 3 10.42.0.2
# Rocky Studio desktop host connects to UDP 10.42.0.2:7777
```

---

## 4. Expanding to ESP32-P4

`firmware/rk-esp32p4` reuses the exact same `rk-fw-core` crate and Embassy-only 200 Hz `ControlTask` loop. Only board peripherals (`esp-hal` USB-OTG High-Speed 480 Mbps + `UART1`/`TWAI0` servo bus + `TIMG0` hardware watchdog) differ—keeping safety invariants and wire framing 100% identical across both MCUs. The ESP32 scaffold currently runs only the control loop; USB transport, TIMG0 watchdog arming, and LED GPIO remain bring-up TODOs.
