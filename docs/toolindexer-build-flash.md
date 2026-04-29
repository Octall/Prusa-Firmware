# ToolIndexer Firmware — Build & Flash

## Prerequisites

- `avr-gcc` (bundled at `.dependencies/avr-gcc-7.3.0/`)
- `cmake` ≥ 3.22
- `avrdude` (`sudo apt install avrdude` or equivalent)
- USB connection to the Einsy board (appears as `/dev/ttyACM0` or similar)

## Build

```bash
export PATH="$PWD/.dependencies/avr-gcc-7.3.0/bin:$PATH"

cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/AvrGcc.cmake
cmake --build build --target ToolIndexer_ENGLISH
```

Output: `build/ToolIndexer_ENGLISH.hex` (~54 KB, fits well within the 256 KB ATmega2560 flash)

## Flash

Find the serial port:
```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

Flash via avrdude (replace `/dev/ttyACM0` as needed):
```bash
avrdude -p atmega2560 -c wiring -P /dev/ttyACM0 -b 115200 \
  -U flash:w:build/ToolIndexer_ENGLISH.hex:i
```

## Verify

Open a serial terminal at **115200 baud**:
```bash
screen /dev/ttyACM0 115200
# or: minicom -D /dev/ttyACM0 -b 115200
```

On boot you should see:
```
READY
```

Quick smoke test:
```
STATUS          → POS X 0.00 Y 0.00 Z 0.00 E 0.00 + sensor states
HOME ALL        → axes move to endstops, reply OK
MOVE X50 F1000  → X axis moves 50 mm, reply OK
READ IR         → VALUE 0 or VALUE 1
ESTOP           → motors stop immediately
ENABLE          → re-enables motors
```

## Serial Command Reference

| Command | Description |
|---|---|
| `HOME [X\|Y\|Z\|ALL]` | Home specified axes (default: all) |
| `MOVE [X<mm>] [Y<mm>] [Z<mm>] [E<mm>] [F<mm/min>]` | Queue a move |
| `WAIT` | Block until motion buffer drains |
| `ESTOP` | Disable all stepper drivers immediately |
| `ENABLE` | Re-enable stepper drivers |
| `READ IR` | IR filament sensor (`VALUE 0/1`) |
| `READ PINDA` | SuperPINDA probe (`VALUE 0/1`) |
| `READ ENDSTOP [X\|Y\|Z\|ALL]` | Endstop state (`VALUE 0/1` or bitmask) |
| `STATUS` | Position (mm) + sensor states |
| `LCD <text>` | Display up to 20 chars on LCD row 3 |

All commands reply `OK` on success or `ERROR <reason>` on failure.
