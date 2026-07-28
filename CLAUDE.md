# Project Instructions

## Project

This is an ESP32-S3 three-wheel chassis prototype derived from the ESP-IDF
Blink example. Read `CODEBASE_GUIDE.md` before changing control behavior.

## Tech Stack

- Firmware: C, ESP-IDF 6.0.2, FreeRTOS
- Motor I/O: LEDC PWM and PCNT quadrature encoders
- Debug transports: UART0 and SoftAP/APSTA TCP
- PC tools: Python, pyserial, PySide6, pyqtgraph
- Target: ESP32-S3

## Build and Run

```powershell
idf.py set-target esp32s3
idf.py -B build-local build
idf.py -B build-local -p COM3 flash monitor
python tools/debug_panel_gui.py
python tools/debug_panel.py --port COM3
```

Use a fresh build directory: the committed/local `build/` cache references an
old absolute path. Do not edit `build/` or `managed_components/`.

## Structure

- `main/blink_example_main.c`: startup, tasks, debug command hooks
- `components/chassis/`: Vx/Vy/Vw state and three-wheel kinematics
- `components/motor/`: three motor instances and closed-loop orchestration
- `components/pid/`: position and incremental PID implementation
- `components/tb6612/`: reusable TB6612, LEDC, and encoder module
- `components/debug_comm/`: UART frame transport
- `components/wifi_debug/`: Wi-Fi/TCP frame transport
- `components/config/`: default PID values
- `tools/esp_link.py`: shared GUI serial/TCP client
- `tools/debug_panel*.py`: PC control UIs

## Control Invariants

- `tb6612_set_speed()` accepts signed permille in `[-1000, 1000]`.
- Active motors need distinct LEDC channels. A timer may be shared only when
  frequency and resolution match.
- Encoder CPR means four-edge counts per output-shaft revolution.
- Protocol frames are little-endian:
  `[0xAA][CMD][LEN][PAYLOAD][checksum]`.
- Keep firmware and Python command IDs and payload layouts synchronized.
- Motor power, encoder, TB6612, and ESP32 must share ground; STBY is assumed
  tied high externally.

## Current Caveats

- `chassis_task` currently forces `ROBOT_CHASSI.Vx = 100` every 100 ms.
- UART, Wi-Fi, and periodic state reporting are disabled in `app_main()`.
- Motor pins/CPR are hardcoded in `components/motor/motor.c`; Kconfig motor
  values are not the source of truth.
- Shared chassis, motor, and PID state has no synchronization.
- `rgb_led.c` is not registered in `main/CMakeLists.txt`.
- Some Chinese comments are mojibake; preserve UTF-8 and repair only when the
  intended wording is clear.

## Code Style

- Follow the ESP-IDF component layout: public headers in `include/`.
- Use `snake_case` for C functions/files and Python functions.
- Keep hardware behavior behind small module interfaces.
- Prefer `esp_err_t` returns in reusable firmware modules.
- Do not add a new seam until there are at least two real adapters.
- Avoid direct access to module runtime state in new code; add focused
  interface functions instead.
- Keep UART/TCP transport concerns separate from protocol parsing.

## Testing

```powershell
python -m py_compile tools/esp_link.py tools/debug_panel.py tools/debug_panel_gui.py pytest_blink.py
pytest pytest_blink.py
```

The existing pytest only checks the firmware binary size. Add host-side tests
for pure kinematics, PID behavior, frame encoding/decoding, and emergency-stop
state before relying on automated coverage.

## Git

History is too shallow (one `first commit`) to infer branch or commit-message
conventions. Preserve unrelated user changes and keep commits narrowly scoped.
