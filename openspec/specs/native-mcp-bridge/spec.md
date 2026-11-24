# native-mcp-bridge Specification

## Purpose

External MCP clients can drive the running native desktop app over loopback HTTP, with requests serialized onto the egui thread so the live 200 Hz loop, recovery watcher, and 3D state are reachable.

## Requirements

### Requirement: Loopback MCP listener in desktop app
The native desktop app SHALL start a Streamable HTTP MCP listener bound to 127.0.0.1 on a configurable port (default 7788), and SHALL NOT bind external interfaces. Port conflicts SHALL be surfaced in the UI without preventing the app from launching.

#### Scenario: Listener available
- **WHEN** the desktop app starts with the default configuration
- **THEN** `GET /health` on 127.0.0.1:7788 returns ok and the drawer shows the endpoint as live

#### Scenario: Port conflict is non-fatal
- **WHEN** the configured port is already in use
- **THEN** the GUI still starts and the drawer reports the listener as unavailable

### Requirement: egui-thread request bridge
MCP requests SHALL be forwarded from the HTTP thread to the egui thread via a bounded queue and dispatched against the live application; replies SHALL be returned to the originating HTTP request. Requests SHALL still be serviced when the window is minimized.

#### Scenario: External tool call drives the app
- **WHEN** an HTTP client calls an MCP tool
- **THEN** the tool executes against the running app's model/transport and the reply is returned to that client

#### Scenario: Queue saturation and timeouts
- **WHEN** the request queue is full or the egui thread does not reply in time
- **THEN** the HTTP request receives an explicit error rather than hanging

### Requirement: Honest UI status
The MCP drawer SHALL report the actual listener status (live/unavailable/stopped) and its quick-test shove preset SHALL use the tool's real argument names.

#### Scenario: Drawer reflects listener state
- **WHEN** no listener is running on native
- **THEN** the drawer does not claim the endpoint is live

#### Scenario: Shove quick test works
- **WHEN** the operator presses the shove quick-test button
- **THEN** the request uses `pitch_impulse_rad_s`/`roll_impulse_rad_s`/`step_ticks` and succeeds
