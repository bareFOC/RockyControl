# Rocky Humanoid MCU Firmware (`copper-rs` + `embassy-rs`)

Real-time `#![no_std]` humanoid control firmware built on **`copper-rs` (`cu29 1.3`)** and **`embassy-rs`**, targeting **RP2350** first and architected for **ESP32-P4** expansion with zero control-logic duplication.

---

## 1. Layered Crate Architecture

```text
crates/rk-proto          (#![no_std])  HostToMcu / McuToHost, postcard COBS, f32
crates/rk-control        (#![no_std])  DirectServoKernel (100 ms watchdog, soft stops, slew clamp)
  ▲
firmware/rk-fw-core      (#![no_std])  Board-agnostic MCU core:
  │                                          - Real cu29 DirectServoCuTask, HostLinkBridge, ServoBusBridge
  │                                          - Zero-alloc CobsStreamFramer (USB CDC-ACM)
  │                                          - UDP COBS datagram codec (USB CDC-NCM networking)
  │                                          - BenchLoopbackBus (Day-1 USB-only testing) & ServoBusPort
  │                                          - Deterministic StatusLedMode (Teach / Drive / Fault)
  ├── firmware/rk-rp2350  (Primary)    RP2350 Cortex-M33F (thumbv8m.main-none-eabihf)
  │                                          embassy-rp 0.10 + embassy-usb 0.6 + embassy-net 0.9
  └── firmware/rk-esp32p4 (Next MCU)   ESP32-P4 RV32IMAFC (riscv32imafc-unknown-none-elf)
                                             esp-hal + esp-hal-embassy + USB-OTG HS 480 Mbps
```

### Copper-rs Task Graph (`copperconfig.ron`, identical across MCUs)

```text
host_link/rx_cmd   ──(HostCmdMsg)───────┐
                                        ▼
                                 [servo_ctrl] ──(ControlStepMsg)──┬──▶ servo_bus/tx_cmd
                                        ▲                         └──▶ host_link/tx_telem
servo_bus/rx_state ─(ServoFeedbackMsg)──┘
```

- **Loop Rate:** Deterministic `200 Hz` (`5 ms` `embassy_time::Ticker`) driving `app.run_one_iteration()`.
- **Numeric Parity:** Strictly single-precision `f32` everywhere (RP2350 Cortex-M33F & ESP32-P4 RV32IMAFC both have hardware single-precision FPUs).
- **Safety by Construction:**
  - Always boots in `Teach` mode (`torque_mask = 0`).
  - `100 ms` host deadman watchdog inside `DirectServoKernel` latches `EStop` and forces torque OFF on cable pull or GUI stall.
  - `250 ms` RP2350 hardware `Watchdog` resets the MCU if the `copper-rs` loop itself ever wedges.
  - Onboard LED (`GPIO25` on Pico 2): **1 Hz slow heartbeat** = `Teach`, **Solid ON** = `Drive`, **8 Hz rapid strobe** = `EStop` / Watchdog / Fault.

---

## 2. Wired Bring-Up Modes (Phase 1 — Wired Only)

Both wired transports carry the **exact same `postcard` COBS-encoded `HostToMcu` / `McuToHost` frames**:

| Feature Flag | Transport | Host Setup | Best For |
| :--- | :--- | :--- | :--- |
| `--features usb-cdc,bench-loopback` *(default)* | **USB CDC-ACM Virtual Serial** (`VID 0x1209 / PID 0x524B`) | Plug in USB -> `/dev/ttyACM0` (Linux), `/dev/cu.usbmodem*` (macOS), `COMx` (Windows) | Fastest Day-1 plug-and-play bench test + browser WebSerial |
| `--no-default-features --features usb-net,bench-loopback` | **USB Networking (CDC-NCM Ethernet-over-USB)** (`VID 0x1209 / PID 0x524E`) | Assign host USB Ethernet adapter `10.42.0.1/24`; MCU listens on UDP `10.42.0.2:7777` (ICMP ping enabled) | High-rate telemetry, Wireshark packet capture, zero serial framing jitter |

### Bench Loopback vs. Real Servo Bus

- **`bench-loopback`** *(default)*: Runs `BenchLoopbackBus` directly inside the MCU so you can flash RP2350, plug in a **single USB cable**, and test live 200 Hz Teach/Drive mirroring, bumpless transfer, E-Stop, and cable-pull watchdog trip with zero servos powered.
- **`uart-servo-bus`**: Initializes RP2350 `UART0` (`GPIO0` TX / `GPIO1` RX @ `1 Mbps`) wired into `SERVO_BUS_SLOT` for physical Feetech STS3215 / Dynamixel Protocol 2.0 smart bus servos.

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

`firmware/rk-esp32p4` reuses the exact same `rk-fw-core` crate and `copperconfig.ron` graph. Only board peripherals (`esp-hal` USB-OTG High-Speed 480 Mbps + `UART1`/`TWAI0` servo bus + `TIMG0` hardware watchdog) differ—keeping safety invariants and wire framing 100% identical across both MCUs.
