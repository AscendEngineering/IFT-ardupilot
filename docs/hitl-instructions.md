# HITL / Sim-on-Hardware Instructions

This document covers building and running the **Sim-on-Hardware (SIH)** firmware for
the IF800 (CubeOrangePlus), used for hardware-in-the-loop style testing. The board
runs ArduPilot's native SITL physics model *on the real flight controller*, with
simulated IMU/GPS/baro/compass, while real peripherals (e.g. the Gremsy gimbal and
its MAVLink camera) stay connected and active.

> §2 covers external-physics HITL over PPP/USB; §3 covers desktop SITL.

> **Warning:** Firmware built from this `hitl` branch lineage embeds bench-only
> parameter defaults (Remote ID disabled, CAN drivers off, battery monitors off, and
> now `NET_ENABLE 1` / `SERIAL6_PROTOCOL 48`, which also displaces OTG2's normal SLCAN
> role) and must **never** be flashed on a production aircraft.

---

## 1. Sim-on-Hardware (SIH)

### 1.1 What gets built

The SIH firmware is the **production `IF800-CubeOrangePlus` board** firmware, plus a
build-time overlay (`sih-extra-hwdef.dat`) that:

- turns on the on-board simulator (`env SIM_ENABLED 1`),
- caps compass instances and trims unused features for flash,
- forces the simulated vehicle to a quad (`AP_SIM_FRAME_CLASS MultiCopter`,
  `AP_MOTORS_FRAME_QUAD_ENABLED`),
- keeps the gimbal + MAVLink camera backends compiled in.

Files live in `libraries/AP_HAL_ChibiOS/hwdef/IF800-CubeOrangePlus/`:

| File | Role |
|------|------|
| `hwdef.dat` | Production board definition (includes `../CubeOrangePlus/hwdef.dat`). |
| `defaults.parm` | **The embedded default parameters** (see §1.4). |
| `sih-extra-hwdef.dat` | SIH build overlay (`env SIM_ENABLED 1`, frame, feature trims). |

### 1.2 Prerequisites

**ARM cross-compiler** — `gcc-arm-none-eabi-10-2020-q4-major` (the version this
ArduPilot 4.5.x tree expects). Install to your home dir (no sudo):

```bash
cd ~
wget https://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
tar xjf gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
export PATH="$HOME/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"   # add to ~/.bashrc to persist
```

**Python ≤ 3.11.** This tree bundles an old `waf` (`modules/waf`) that still does
`import imp`, which was **removed in Python 3.12**. Building under a 3.12+ interpreter
fails with `ModuleNotFoundError: No module named 'imp'`. Use a 3.10/3.11 venv:

```bash
python3.11 -m venv ~/venv-ap311
source ~/venv-ap311/bin/activate
pip install --upgrade pip
pip install "empy==3.3.4" pexpect future pymavlink intelhex dronecan
```

> `empy` **must** be 3.3.x — empy 4.x breaks ArduPilot's code generation.

### 1.3 Build

From the repo root, with the toolchain on `PATH` and the Python ≤3.11 venv active:

```bash
./waf configure --board IF800-CubeOrangePlus \
  --extra-hwdef=libraries/AP_HAL_ChibiOS/hwdef/IF800-CubeOrangePlus/sih-extra-hwdef.dat

./waf copter            # build only
./waf copter --upload   # build and flash the connected board
```

A successful configure prints:

```
Default parameters path from hwdef: .../IF800-CubeOrangePlus/defaults.parm
```

Artifacts land in `build/IF800-CubeOrangePlus/bin/` (`arducopter.apj`,
`arducopter_with_bl.hex`, etc.).

> **Note:** do **not** pass `--default-param=...`. It is silently ignored here (see
> §1.4) and is not needed — the board's `defaults.parm` is picked up automatically.

### 1.4 Where the parameters come from

This is the part that trips people up, so it is spelled out in full.

**The embedded defaults are `defaults.parm` in the board's hwdef directory** —
`libraries/AP_HAL_ChibiOS/hwdef/IF800-CubeOrangePlus/defaults.parm`.

During `./waf configure`, `chibios_hwdef.py:write_default_parameters()` looks for a
`defaults.parm` next to the board's `hwdef.dat` and, because this board does **not**
set `FORCE_APJ_DEFAULT_PARAMETERS`, embeds it into the firmware via **ROMFS** (as the
file `defaults.parm`). At boot `AP_Param` loads it through
`load_embedded_param_defaults()`.

The processed copy that actually gets embedded is written to
`build/IF800-CubeOrangePlus/processed_defaults.parm` — inspect it to confirm what
shipped.

**Why `--default-param=<file>` does nothing here.** `chibios_hwdef.py` resolves the
CLI path as `os.path.join(dirname(hwdef), args.params)`. Passing a full relative path
(e.g. `libraries/.../sih-defaults.parm`) produces a non-existent joined path, so the
check fails and the code falls back to the board's `defaults.parm`. The CLI file is
ignored without warning. For that reason all SIH defaults are kept **directly in
`defaults.parm`** rather than in a separate `--default-param` file.

> This board variant is built **SIH-only**, so `defaults.parm` carries the simulator
> configuration (e.g. `GPS_TYPE 100`, `AHRS_EKF_TYPE 10`, `SIM_*`). Do not flash a
> build of this board from this branch onto a real aircraft.

**Parameter precedence (low → high):** compiled-in defaults → embedded `defaults.parm`
→ values saved in the board's flash/FRAM → live `param set` from a GCS. A value
previously written to the board's storage **overrides** the embedded default. After
flashing, refresh params and confirm the key values, or `param set` them once if the
board still holds stale values from a prior session.

#### Reset parameters on first SIH boot (required)

The IF800 is a production board repurposed for SIH, so its flash/FRAM still holds
the **real** sensor calibration from production firmware. Parameter space is shared
between the real and simulated aircraft, and the sensor suites differ
([ArduPilot SIH docs](https://ardupilot.org/dev/docs/sim-on-hardware.html)). The
simulated IMUs/compasses register fewer instances than production, so the sim never
overwrites the stale extra device IDs, and embedded `defaults.parm` **cannot** clear a
value already saved to storage. Symptoms of skipping this step:

- The board demands an **accelerometer calibration** even though `INS_ACC*` cal offsets
  are in `defaults.parm`. A stale `INS_ACC3_ID` (the disabled aux IMU) left in storage
  trips the "missing accel" branch of `accel_calibrated_ok_all()`.
- The board demands a **compass calibration** ("Compass not calibrated"). `COMPASS_OFS`
  reads back as zero (the embedded non-zero offsets are shadowed by saved zeros) and/or
  a stale `COMPASS_DEV_ID3`/`COMPASS_PRIO3_ID` from production lingers.

Neither can be fixed by `param set` of individual IDs — do a full storage wipe so the
embedded SIH defaults apply and the simulated sensors re-register cleanly:

1. (Optional) Save the board's current parameters first if you will reflash production
   firmware later.
2. Set `FORMAT_VERSION 0` and **reboot**. On the next boot the param header no longer
   matches, storage is erased (`AP_Param::erase_all()`), and the embedded SIH defaults
   are re-applied. (Equivalent: Mission Planner *Reset to Default*, or
   `MAV_CMD_PREFLIGHT_STORAGE` param1=2.)
3. Reboot once more and confirm `COMPASS_OFS_X` reads `5` (not `0`), `COMPASS_DEV_ID`
   and `INS_ACC_ID` are populated by the sim backend, and no accel/compass cal is
   demanded.

### 1.5 What the SIH defaults configure

Beyond the standard simulated-sensor setup, `defaults.parm` configures the gimbal and
camera signal chain:

| Area | Params | Purpose |
|------|--------|---------|
| Sim sensors | `AHRS_EKF_TYPE 10`, `GPS_TYPE 100`, `SIM_MAG1_DEVID`, INS cal offsets, `SIM_RATE_HZ 400`, `SCHED_LOOP_RATE 400` | Run the native SITL model on the board. |
| Sim GPS port | `SERIAL3_PROTOCOL 5` | The SITL GPS backend (type 100) only starts if a serial port is set to GPS protocol. |
| Gimbal/camera link | `SERIAL4_PROTOCOL 2`, `SERIAL4_BAUD 115`, `SERIAL4_OPTIONS 0` | UART8 is MAVLink2 so the Gremsy gimbal (`MAV_TYPE_GIMBAL`) and MAVLink camera (`MAV_TYPE_CAMERA`) heartbeats are routed and the devices are discovered. |
| Gimbal | `MNT1_TYPE 6` (Gremsy), `MNT1_DEFLT_MODE 3` (RC_TARGETING), `MNT1_*` ranges | Gremsy MAVLink gimbal. |
| Camera | `CAM2_TYPE 6` (MAVLink CamV2), `CAM2_MNT_INST 0` | CamV2 is the backend that requests + relays `CAMERA_INFORMATION`. |
| Gimbal RC control | `RC15_OPTION 214` (Mount1 yaw), `RC9_OPTION 163` (yaw lock/follow) | RC stick/wheel control in RC_TARGETING mode. GCS/mission `DO_MOUNT_CONTROL` works without these. |

> The Gremsy gimbal and the MAVLink camera are physically wired to **UART8 / SERIAL4**.
> If that wiring changes, update the `SERIAL4_*` (or relevant `SERIALx_*`) lines so the
> device's port is MAVLink2 — otherwise the gimbal will not move and
> `CAMERA_INFORMATION` reports `UNSUPPORTED`.

### 1.6 Verifying on the board

After flashing, connect a GCS and check:

- A component with `MAV_TYPE_GIMBAL` and one with `MAV_TYPE_CAMERA` appear on the link.
- The gimbal responds to a commanded angle (GCS/mission) and to RC yaw input.
- `CAMERA_INFORMATION` returns vendor/model instead of `UNSUPPORTED`.
- `SERIAL4_PROTOCOL` reads `2` and `GPS_TYPE` reads `100` (if a stale stored value
  shadows the default, `param set` it once).

---

## 2. External-physics HITL (JSON over PPP/USB)

This section covers **external-physics HITL**: the board runs the real flight-code
firmware and a physics model on the laptop supplies the simulated state, connected
over PPP on a second USB interface. It sits between §1 (physics on the board) and §3
(the whole autopilot on the desktop).

### 2.1 Concept

The board runs the real `arducopter` flight-code firmware — the same binary you would
fly with — built with a `JSON` physics-frame overlay (§2.2) instead of the on-board SIH
model of §1. The physics simulation itself runs on the laptop as a separate process
(§2.5 / §2.6) and exchanges state with the firmware using ArduPilot's JSON-over-UDP
protocol (port 9002), carried over a PPP link established on the board's **second**
USB CDC interface (OTG2 / SERIAL6) — the same physical USB cable, a second logical
port. SERIAL0 (the first CDC interface, OTG1) is untouched and keeps carrying MAVLink
to your GCS exactly as it does in §1. Because the board itself is the flight
controller running the real firmware, `sim_vehicle.py` — which launches and manages a
**desktop** SITL process — plays no part in this bench test; it only reappears in §2.7
for a desktop-only regression check that doesn't involve the board at all.

### 2.2 Build

Same toolchain/venv prerequisites as SIH (§1.2) — nothing additional. The build
differs only in which extra-hwdef overlay is passed; `json-extra-hwdef.dat` `include`s
`sih-extra-hwdef.dat` and then swaps the simulated-frame class from the on-board
`MultiCopter` model to the external `JSON` backend:

```bash
./waf configure --board IF800-CubeOrangePlus \
  --extra-hwdef=libraries/AP_HAL_ChibiOS/hwdef/IF800-CubeOrangePlus/json-extra-hwdef.dat

./waf copter
```

This is a **build-only** step — do not add `--upload`; flash the board with your usual
tool once the build completes. The artifact is
`build/IF800-CubeOrangePlus/bin/arducopter.apj`, with 193,292 bytes (~189 KiB) of
flash free.

> As with SIH, do not pass `--default-param=...` — it is silently ignored for this
> board (§1.3). All defaults, including the two new networking lines (§2.3), live in
> `defaults.parm`.

### 2.3 Parameters: precedence and required read-back (do this every flash)

The same precedence rule as §1.4 applies: compiled-in defaults → embedded
`defaults.parm` → values saved in the board's flash/FRAM → live `param set` from a
GCS, with a **stored value always winning over the embedded default**. This build's
`defaults.parm` only adds two lines on top of §1's (`NET_ENABLE 1`,
`SERIAL6_PROTOCOL 48`) — if this board has ever been flashed before (SIH, production,
or an earlier JSON build) with those params at a different value, the stored copy wins
and networking silently does not start. Do not assume the embedded defaults took
effect; read them back.

After flashing, connect a GCS on SERIAL0 (unchanged — §2.1) and confirm:

| Param | Required value | Why |
|-------|-----------------|-----|
| `NET_ENABLE` | `1` | starts `AP_Networking`. |
| `SERIAL6_PROTOCOL` | `48` | SERIAL6 (OTG2, the second CDC interface) carries PPP. |
| `SERVO_32_ENABLE` | `0` | keeps the 16-channel servo-output packet (`uint16 pwm[16]`, magic `18458`) that both baseline backends (§2.5, §2.6) parse. |

`SERVO_32_ENABLE 1` switches the firmware to a 32-channel packet (`uint16 pwm[32]`,
magic `29569`) that neither backend understands — the board will look silent to the
backend even though it is sending valid packets. If any of the three read back wrong,
`param set` the correct value and reboot; if the board is also showing the
accel/compass calibration symptoms from §1.4's storage-reset section, do the full
`FORMAT_VERSION 0` wipe described there instead of patching params one at a time.

Also confirm the GCS link shows two status texts on reboot, in order:

```
PPP: started
NET: Initialized
```

Both come from the firmware, not from `pppd` — seeing them confirms the board-side
network/PPP stack came up; it says nothing about the laptop side yet (§2.4).

### 2.4 Laptop PPP bring-up

The board's single USB cable exposes two CDC-ACM ports: the first (OTG1) is unchanged
SERIAL0 MAVLink; the second (OTG2 = SERIAL6, per §2.3) is the raw PPP byte stream.
Linux does not guarantee which one enumerates as `/dev/ttyACM0` vs `/dev/ttyACM1` — it
depends on boot order and whatever else is plugged in — so don't hard-code
`/dev/ttyACM1` without checking; identify it first:

```bash
ls -l /dev/serial/by-id/            # stable, udev-assigned names
# or, once you have a candidate node:
udevadm info -q property -n /dev/ttyACM1 | grep ID_
```

Once you've confirmed the PPP node, bring it up:

```bash
sudo pppd /dev/ttyACM1 115200 192.168.144.1:192.168.144.14 \
     noauth local nodetach persist maxfail 0
# verify: ping 192.168.144.14
```

`/dev/ttyACM1` above is only the typical case on an otherwise-quiet USB bus — use
whatever `/dev/serial/by-id/` (or `udevadm`) actually gave you.

> The `115200` is a **host-side TTY line setting only** — it configures `pppd`'s end of
> the `/dev/ttyACM1` character device. CDC-ACM is a USB transport, not a physical UART,
> so there is no baud rate on the wire and nothing on the board needs to agree with
> this number.

`persist maxfail 0` is required, not cosmetic — see the boot-order caveat in §2.8: the
link is expected to drop and reconnect around the board's power-on.

### 2.5 Required baseline backend: the JSON C++ `minimal` example

The board's lockstep loop (§2.8) will not run at all without *something* answering
JSON on the laptop, so bring this up before touching the board. Build and run it:

```bash
cmake -S libraries/SITL/examples/JSON/C++ -B /tmp/ardupilot-json-example
cmake --build /tmp/ardupilot-json-example --target minimal
/tmp/ardupilot-json-example/minimal
```

`minimal` listens on UDP `9002` and does nothing until it receives the board's
servo-output packet (magic `18458`, 16 channels); it then starts replying to whichever
IP:port that packet came from — no separate target configuration is needed. It sends
back a fixed, non-moving state (zero position/velocity/attitude, `-9.81 m/s²` on Z for
gravity) plus a few optional fields (airspeed, windvane, rangefinder), so it satisfies
the protocol and unblocks the board's loop but does **not** simulate real flight
dynamics — it is a connectivity/protocol check, not a dynamics backend (§2.6 for that).

Don't wait for a documented ten-second grace period if nothing connects. The JSON
backend's own `readme.md` describes a 10-second resend, but in this tree
`SIM_JSON.cpp` actually resends whenever `wait_ms > 1000`, i.e. roughly **one
second**. If `minimal` hasn't reported a connection within a couple of seconds of the
board being up, treat it as a PPP/routing problem (§2.4), not something to wait out.

### 2.6 Optional: PyBullet dynamics backend

For real flight dynamics instead of `minimal`'s static state,
`libraries/SITL/examples/JSON/pybullet/robot.py` drives a PyBullet quadcopter model.
It needs **both** PyBullet and [PyRoboLearn](https://github.com/robotlearn/pyrobolearn)
— `robot.py` imports `pyrobolearn` directly (`from pyrobolearn.simulators import
Bullet`, `import pyrobolearn as prl`), so a plain `pip install pybullet` is **not**
sufficient; install PyRoboLearn as well, per its own instructions.

Run it with the `iris` vehicle:

```bash
python3 libraries/SITL/examples/JSON/pybullet/robot.py --vehicle iris
```

`--vehicle quad` is listed as a valid `argparse` choice but is **not implemented** —
none of the script's `elif` branches handle it, so it falls through to `else: raise
Exception("Bad vehicle")` and the script dies immediately. Use `--vehicle iris` for a
quadcopter model (`pyrobolearn.robots.Quadcopter`, `models/iris/iris.urdf`); it maps
`pwm[0:4]` to the four rotor speeds and speaks the same UDP:9002 JSON protocol as
§2.5.

### 2.7 Desktop-only regression check (no hardware)

To sanity-check a JSON-parser or backend change without touching the board at all, run
desktop SITL against the same backend:

```bash
Tools/autotest/sim_vehicle.py -v ArduCopter -f json:127.0.0.1 --console
```

The vehicle **must be `ArduCopter`**. This is a separate desktop SITL process built
from `-v ArduCopter`, not the IF800 board firmware from §2.2, and it connects to a
backend on `127.0.0.1` rather than over PPP. Use this to isolate whether a problem is
in the JSON protocol/parser path (reproducible here) or specific to the board/PPP/
hardware setup (only reproducible earlier in this section).

### 2.8 Boot order: bring up PPP and the backend before the board

> **The board's main loop blocks on physics.** With `AP_SIM_FRAME_CLASS JSON`,
> ArduCopter runs in lockstep with the JSON backend: each scheduler tick it sends a
> servo packet and waits for the matching state reply before continuing. If nothing is
> listening when the board starts, the loop simply does not advance.

Order matters:

1. Start `pppd` on the laptop, persistent (§2.4).
2. Start the backend — `minimal` (§2.5) or the PyBullet script (§2.6).
3. Only then power on or reset the board.

> The board's first USB enumeration after power-on can briefly drop the CDC-ACM nodes
> and knock the not-yet-established PPP link down. With `persist maxfail 0` (§2.4)
> `pppd` is expected to reconnect once the interface reappears — give it a few seconds
> before concluding the link is stuck.

### 2.9 Bench verification checklist

Work through these in order; each depends on the one before it:

1. Parameters read back correctly (§2.3): `NET_ENABLE 1`, `SERIAL6_PROTOCOL 48`,
   `SERVO_32_ENABLE 0`.
2. GCS on SERIAL0 shows `PPP: started` then `NET: Initialized` after boot.
3. `ping 192.168.144.14` succeeds from the laptop.
4. The backend (§2.5 or §2.6) reports it has received the board's servo packets
   (magic `18458` — `minimal` prints `magic: 18458` and `Connected to ArduPilot
   controller @ ...`; `robot.py` prints `Connected to ...`).
5. GCS on SERIAL0 shows the vehicle's JSON-driven state (attitude/position/EKF
   tracking whatever the backend is sending — static for `minimal`, dynamic for
   PyBullet).
6. The board's `JSON received:` field list (printed once, the first time the
   mandatory fields are all satisfied — see `SIM_JSON.cpp`) matches what the backend
   is actually sending: `timestamp`, `gyro`, `accel_body`, `position`, `attitude` (or
   `quaternion`), `velocity`, plus whichever optional fields that backend adds
   (`minimal` also sends `airspeed`, `windvane`, `rng_1`–`rng_6`).

Then check memory separately — a low-memory failure can look exactly like a healthy
boot until something later allocates and fails:

- Note the **initial `Free RAM`** the board reports in its boot banner. This happens
  early, **before** the JSON backend connects and before its dynamic allocations, so
  it is not a valid post-start reading on its own.
- Once PPP and the JSON link are both up (step 4 above), pull `@SYS/memory.txt` and
  `@SYS/threads.txt` through MAVFTP (e.g. MAVProxy `ftp get @SYS/memory.txt`, or your
  GCS's MAVFTP browser) and compare against the boot-time figure.

### 2.10 Failure/recovery test

Don't assume recovery works — exercise it and record what actually happens:

1. **Physics backend drop.** With the board running, stop the backend process (§2.5 /
   §2.6) for long enough to be sure the board's lockstep wait is engaged, then restart
   it. Note whether the board's loop resumes from where it blocked, or whether the
   watchdog reboots the board first — both are plausible outcomes, and the answer
   determines how much of a bench window you actually have.
2. **PPP drop.** Kill and restart `pppd` (or unplug/replug the PPP CDC node) while the
   board keeps running, and confirm the reconnect logic re-establishes the link
   without a board-side reboot. This reconnect logic is pinned in this tree from
   ArduPilot 4.6's networking sync (not the surrounding 4.5.x codebase) — see
   `AP_Networking_PPP.cpp` — so verify it on the bench rather than assuming it behaves
   like the rest of this 4.5.x-based firmware.

Record the observed behavior in both cases rather than assuming either one "just
works" — this is a bench characterization, not a guarantee.

---

## 3. SITL (desktop simulation)

This section covers running **desktop SITL** via `sim_vehicle.py` while a **real**
Gremsy gimbal + MAVLink camera are connected to the host over USB/serial. Unlike SIH
(§1), nothing runs on the flight controller — the autopilot is a process on your PC,
and a physical UART is bridged into one of its simulated serial ports.

### 3.1 The key gotcha: SITL serial ports are not real UARTs

In desktop SITL the **byte transport** for each serial port is hard-wired in code and
is independent of the `SERIALx_*` params. See
`libraries/AP_HAL_SITL/SITL_State.h` (`_serial_path[]`):

| Port | Default transport | Notes |
|------|-------------------|-------|
| SERIAL0 | `tcp:0:wait` | console (TCP 5760) |
| SERIAL1 | `tcp:2` | MAVLink (TCP 5762) |
| SERIAL2 | `tcp:3` | MAVLink (TCP 5763) |
| SERIAL3 | `GPS1` | **simulated** GPS |
| SERIAL4 | `GPS2` | **simulated** 2nd GPS |
| SERIAL5–8 | `tcp:5`…`tcp:8` | |

So by default **SERIAL4 is wired to a fake `GPS2` simulator**, not to a physical UART.
Setting `SERIAL4_PROTOCOL 2` alone does nothing — your gimbal's MAVLink bytes never
reach the autopilot, the gimbal won't move, and `CAMERA_INFORMATION` reports
`UNSUPPORTED`. You must point SERIAL4 at the real device on the command line.

> This is the desktop-SITL equivalent of the SIH wiring note in §1.5: the device has to
> be on a MAVLink2 port **and** that port has to actually carry the device's bytes.

### 3.2 Bridging a real serial device into SITL

`sim_vehicle.py` forwards `-A` args to the SITL binary, which understands a
`uart:<path>:<baud>` device string (see `libraries/AP_HAL_SITL/UARTDriver.cpp`,
`_parse_args` / `uart:` case):

```bash
-A "--serial4=uart:/dev/ttyUSB0:115200"
```

Confirm the device path (the gimbal usually enumerates as `/dev/ttyUSB0`; check
`dmesg | tail` after plugging it in). For a name that is stable across replug/reboot,
use `/dev/serial/by-id/<id>` from `ls -l /dev/serial/by-id/` instead.

Make sure your user can open it (add to the `dialout` group once, then re-login):

```bash
sudo usermod -aG dialout "$USER"
```

The `:115200` in the string sets the host UART baud directly; if SITL prints
`Failed to open (...)` at startup it is a wrong path or a permissions problem, and if
it prints `Opened /dev/...` the bridge is up.

### 3.3 Parameters: the board defaults do NOT apply here

The board's `defaults.parm` / SIH config from §1 are embedded in **board firmware** and
are **not** loaded when you run desktop SITL. You must supply the gimbal/camera params
explicitly. Keep them in a small file (repo root `gimbal-sitl.parm`):

```
# --- serial link to the real Gremsy gimbal/camera on SERIAL4 ---
SERIAL4_PROTOCOL 2
SERIAL4_BAUD 115
SERIAL4_OPTIONS 0

# --- Gremsy MAVLink mount ---
MNT1_TYPE 6
MNT1_DEFLT_MODE 3
MNT1_OPTIONS 1
MNT1_RC_RATE 90
MNT1_PITCH_MIN -90
MNT1_PITCH_MAX 90
MNT1_ROLL_MIN -45
MNT1_ROLL_MAX 45
MNT1_YAW_MIN -319
MNT1_YAW_MAX 319

# --- MAVLink camera tied to mount instance 0 ---
CAM2_TYPE 6
CAM2_MNT_INST 0

# --- RC stick control of the gimbal (MNT1_DEFLT_MODE 3 = RC_TARGETING) ---
# RC9  = 163 MOUNT_LOCK  (yaw lock vs follow toggle)
# RC15 = 214 MOUNT1_YAW  (yaw input -- this is the stick that moves yaw)
RC9_OPTION 163
RC15_OPTION 214
```

> Do **not** add the sim-sensor params (`GPS_TYPE 100`, `AHRS_EKF_TYPE 10`, `SERIAL3_*`).
> Desktop SITL already simulates its own sensors and runs the sim GPS on SERIAL3.

### 3.4 Run it

```bash
cd <repo root>
Tools/autotest/sim_vehicle.py -v ArduCopter \
  -A "--serial4=uart:/dev/ttyUSB0:115200" \
  --add-param-file=gimbal-sitl.parm \
  --console --map
```

- `--add-param-file=` applies the params right after defaults load.
- Add `-w` for a clean param/EEPROM wipe, `-N` to skip the rebuild if already built.

To load params into an **already-running** SITL instead, use the MAVProxy console:

```
param load gimbal-sitl.parm
reboot
```

> `MNT1_TYPE` only instantiates the mount backend at boot, so a `reboot` is required
> after changing it (MAVProxy reconnects automatically).

### 3.5 Driving the gimbal in SITL

`MNT1_DEFLT_MODE 3` (RC_TARGETING) means the gimbal moves from RC input. **Commanded
control and RC control are separate code paths** — `MAV_CMD_DO_MOUNT_CONTROL` goes
straight to the mount target, while RC control reads `get_radio_in()` of the channel
mapped to `MOUNT1_YAW` and only acts when that raw PWM is `> 0`
(`AP_Mount_Backend::get_rc_input`). So commanded control can work while RC does
nothing — that just means no RC is being injected.

With no physical transmitter in SITL, override the channels from MAVProxy
(**ch15 = yaw input**, ch9 = lock/follow). Values must be **off-center** — 1500 is the
trim/deadzone and produces no movement:

```
rc 15 1900      # MOUNT1_YAW input -- gimbal yaw
rc 9  1900      # MOUNT_LOCK -- follow vs lock (frame only)
```

Verify the override actually reaches the FC (the key diagnostic if RC seems dead):

```
status RC_CHANNELS      # chan15_raw should now read 1900, not 0 / 65535
```

With `MNT1_RC_RATE 90` (rate mode, as on the production board) ch15 controls yaw
*rate*: `1900` slews continuously, `1500` holds. With `MNT1_RC_RATE 0` (angle mode) ch15
maps stick *position* to absolute yaw angle, so `1500` snaps back to center.

GCS/mission `MAV_CMD_DO_MOUNT_CONTROL` works regardless of RC mapping, e.g.:

```
mount 1 pitch -45 0 0
```

### 3.6 Verifying

- SITL printed `Opened /dev/...` (not `Failed to open`) at startup.
- A `MAV_TYPE_GIMBAL` and a `MAV_TYPE_CAMERA` component appear on the link.
- The gimbal responds to a commanded angle and to ch15 yaw override.
- `CAMERA_INFORMATION` returns vendor/model instead of `UNSUPPORTED`; camera trigger
  (e.g. `module load camera` / GCS shutter) takes a picture.
