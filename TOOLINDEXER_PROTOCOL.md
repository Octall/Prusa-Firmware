# ToolIndexer Serial Command Protocol

## Overview

The ToolIndexer firmware runs on a Prusa MK3S Einsy Rambo board repurposed as a
multi-tool carousel controller for a Kinova robotic arm. It exposes a simple
line-oriented ASCII command protocol over USB/UART at **115200 baud, 8N1**.

Every command is a single line. Every command produces exactly one reply line
(except `READ` and `STATUS`, which produce data lines *followed* by a final `OK`).
The protocol is **synchronous**: the firmware does not send `OK` until the
requested action (motion, homing, tool selection) is physically complete.

---

## Connection

| Parameter | Value |
|-----------|-------|
| Baud rate | 115200 |
| Data bits | 8 |
| Parity    | None |
| Stop bits | 1 |
| Line ending sent by host | `\n` or `\r\n` |
| Line ending sent by firmware | `\r\n` |

The firmware echoes each character as it is typed, and echoes a blank line
(`\r\n`) when it receives the line terminator. Hosts that don't want echoed
characters should either open the port with local echo disabled or simply ignore
characters that are not reply lines (i.e. lines that do not start with `OK`,
`ERROR`, `VALUE`, `POS`, `SENSOR`, `CONFIG`, or `TOOL`).

---

## Reply Format

### Success

```
OK\r\n
```

When a sequence number was provided (see below):

```
OK N<seq>\r\n
```

### Failure

```
ERROR <reason>\r\n
ERROR <reason> N<seq>\r\n
```

`<reason>` is a short human-readable string with no spaces internal to it
(e.g. `PINDA verify failed`, `no axis specified`).

---

## Sequence Numbers (Acknowledgement Correlation)

Any command may be prefixed with `N<number>` (an unsigned 16-bit integer). The
firmware strips the prefix, executes the command, and echoes the same number in
the reply.

```
Host  →  N1 TOOL A\n
Device →  OK N1\r\n

Host  →  N2 HOME\n
Device →  ERROR PINDA home failed N2\r\n
```

**Why use sequence numbers?**

- The firmware is synchronous: it only processes one command at a time and
  sends exactly one reply per command. But if your host sends a command and the
  reply is lost (e.g. a read timeout followed by a retry), you cannot tell
  whether the original command succeeded. With a sequence number you can retry
  with a *new* number and know immediately whether you are seeing a reply to the
  retry or a delayed reply to the original.
- For logging and debugging: every log entry can record the sequence number, so
  it is trivial to correlate host-side log lines with firmware-side events.

Sequence numbers are optional. Commands without `N` work exactly as before and
receive bare `OK` / `ERROR` replies. If you are writing a new host integration,
using sequence numbers is strongly recommended.

**Practical host implementation (pseudocode):**

```python
seq = 0

def send_command(cmd):
    global seq
    seq = (seq + 1) % 65536
    tag = f"N{seq}"
    port.write(f"{tag} {cmd}\n".encode())

    # Read lines until we get one that contains our tag
    while True:
        line = port.readline().decode().strip()
        if tag in line:
            if line.startswith("OK"):
                return True, line
            elif line.startswith("ERROR"):
                return False, line
            # data line (VALUE/POS/etc.) with our tag — shouldn't happen,
            # but safe to ignore and keep reading
```

---

## Command Reference

All commands are case-insensitive. Arguments follow the command word separated
by spaces. Unrecognised commands reply `ERROR unknown command`.

---

### `HOME [X] [Y] [Z] [ALL]`

Homes one or more axes by driving toward the min endstop, then backing off 2 mm.
After homing, the axis position is set to 0. If no axis is specified, defaults
to homing all axes.

```
HOME           → homes X, Y, Z
HOME X         → homes X only
HOME ALL       → homes X, Y, Z
```

Reply: `OK` after all requested axes have finished homing.
Failure: `ERROR PINDA home failed` if the PINDA sensor did not trigger during
the X-axis center-finding scan.

---

### `MOVE [X<mm>] [Y<mm>] [Z<mm>] [E<mm>] [F<mm/min>]`

Queues a linear move to the specified absolute position(s) and blocks until the
move completes. At least one axis must be specified. Omitted axes hold their
current position. `F` sets the feedrate in mm/min; if omitted, the default
feedrate (3000 mm/min) is used.

```
MOVE X45.0 F1500
MOVE X10 Y20 Z5
```

Reply: `OK` after the move is complete.

---

### `ROTATE D<degrees> [F<mm/min>]`

Rotates the X axis (the carousel) by a **relative** angle in degrees. Positive
values rotate away from home; negative values rotate toward home. Blocks until
the rotation is complete.

The conversion from degrees to steps uses `axis_steps_per_mm[X]`
(`DEFAULT_AXIS_STEPS_PER_UNIT` in `ToolIndexer.h`, default 143 steps/degree).

```
ROTATE D120        → rotate 120° at default speed
ROTATE D-90 F500   → rotate −90° at 500 mm/min equivalent
```

Reply: `OK` after the motion is complete.

---

### `TOOL <A|B|C>`

Selects a tool by rotating the carousel to its preset angular position:

| Tool | Angle from PINDA home |
|------|-----------------------|
| A    | 0°                    |
| B    | 120°                  |
| C    | −120°                 |

The firmware re-homes X (PINDA center-finding) before rotating to the target
position to eliminate accumulated positioning error.

```
TOOL A
TOOL B
TOOL C
```

Reply: `OK` after the carousel is confirmed at the correct position.
Failure: `ERROR PINDA verify failed` if the PINDA sensor did not trigger during
homing.

This is the most important command for autonomous tool switching. Always wait for
`OK` (or check for `ERROR`) before commanding the robot arm to engage the tool.

---

### `WAIT`

Blocks until the motion planner buffer is empty. Useful as a synchronisation
barrier if you have queued multiple moves and need to confirm all have finished.

```
WAIT
```

Reply: `OK`.

---

### `ESTOP`

Immediately disables the stepper ISR and all motor driver enable pins. All
in-progress motion stops instantly. Positional state is lost.

After an ESTOP, you **must** call `ENABLE` and re-home before issuing any
further motion commands.

```
ESTOP
```

Reply: `OK`.

---

### `ENABLE`

Re-enables stepper drivers and the stepper ISR after an ESTOP. Positional state
is preserved in memory but may no longer reflect physical reality — re-home
before relying on it.

```
ENABLE
```

Reply: `OK`.

---

### `READ <sensor>`

Reads a sensor and returns its value, followed by `OK`.

| Sensor         | Returns                                      |
|----------------|----------------------------------------------|
| `IR`           | `VALUE 0` or `VALUE 1`                       |
| `PINDA`        | `VALUE 0` or `VALUE 1`                       |
| `ENDSTOP X`    | `VALUE 0` or `VALUE 1`                       |
| `ENDSTOP Y`    | `VALUE 0` or `VALUE 1`                       |
| `ENDSTOP Z`    | `VALUE 0` or `VALUE 1`                       |
| `ENDSTOP`      | `VALUE <bitmask>` (all endstops combined)     |

```
READ PINDA
→ VALUE 1\r\n
→ OK\r\n
```

---

### `STATUS`

Returns a multi-line snapshot of the current state, followed by `OK`:

```
POS X<mm> Y<mm> Z<mm> E<mm>\r\n
SENSOR IR=<0|1> PINDA=<0|1> ENDSTOP_X=<0|1> ENDSTOP_Y=<0|1> ENDSTOP_Z=<0|1>\r\n
CONFIG X_STEPS_PER_DEG=<float>\r\n
TOOL <A|B|C|NONE>\r\n
OK\r\n
```

When reading `STATUS`, collect lines until you receive one starting with `OK` or
`ERROR`. All preceding lines are data.

---

### `SET_STEPS_PER_UNIT X<steps>`

Sets the steps-per-unit value for the X axis at runtime. Persists across moves
within a session but is not saved to EEPROM. Useful for calibration.

```
SET_STEPS_PER_UNIT X143.0
```

Reply: `OK` on success. `ERROR invalid X steps per unit` if the value is
missing, zero, or negative.

---

### `LCD <text>`

Writes a string to the user row of the LCD display. Maximum ~20 characters.

```
LCD Tool A ready
```

Reply: `OK`.

---

## Typical Session

```
Host  →  N1 HOME X\n
Device →  OK N1\r\n

Host  →  N2 STATUS\n
Device →  POS X0.00 Y0.00 Z0.00 E0.00\r\n
          SENSOR IR=0 PINDA=0 ENDSTOP_X=0 ENDSTOP_Y=0 ENDSTOP_Z=0\r\n
          CONFIG X_STEPS_PER_DEG=143.00\r\n
          TOOL NONE\r\n
          OK N2\r\n

Host  →  N3 TOOL B\n
Device →  OK N3\r\n          ← carousel is now at 120°, safe to engage arm

Host  →  N4 TOOL A\n
Device →  OK N4\r\n

Host  →  N5 ESTOP\n
Device →  OK N5\r\n

Host  →  N6 ENABLE\n
Device →  OK N6\r\n

Host  →  N7 HOME X\n
Device →  OK N7\r\n
```

---

## Error Handling

| Error string              | Cause                                              | Recovery                       |
|---------------------------|----------------------------------------------------|--------------------------------|
| `PINDA home failed`       | X endstop / PINDA did not trigger during homing    | Check wiring; retry `HOME X`   |
| `PINDA verify failed`     | PINDA center-find failed during `TOOL` selection   | Retry `TOOL <x>` or `HOME X`  |
| `no axis specified`       | `MOVE` called with no axis arguments               | Fix command syntax             |
| `missing D<degrees>`      | `ROTATE` called without `D` argument               | Fix command syntax             |
| `tool must be A, B, or C` | `TOOL` called with invalid argument                | Fix command syntax             |
| `unknown sensor`          | `READ` called with unrecognised sensor name        | Fix command syntax             |
| `unknown command`         | Unrecognised command word                          | Fix command syntax             |
| `line too long`           | Input line exceeded 79 characters                  | Shorten the command            |
| `invalid X steps per unit`| `SET_STEPS_PER_UNIT` value missing or ≤ 0         | Fix command syntax             |

If the firmware sends `ERROR` for a motion command (`HOME`, `MOVE`, `ROTATE`,
`TOOL`), the carousel position is **unknown**. Always re-home before issuing
further motion commands after any motion error.

---

## Firmware Source Files

| File | Purpose |
|------|---------|
| `Firmware/custom_command.cpp` | Command parser and dispatcher |
| `Firmware/custom_motion.cpp`  | Motion API (homing, moves, tool positioning) |
| `Firmware/custom_sensor.cpp`  | Sensor read wrappers |
| `Firmware/custom_display.cpp` | LCD helpers |
| `Firmware/variants/ToolIndexer.h` | Hardware constants (steps/deg, tool angles, feedrates) |
