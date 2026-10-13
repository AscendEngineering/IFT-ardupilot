# External-Physics HITL (JSON over PPP/USB) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let the IF800-CubeOrangePlus run its real firmware while an external physics simulator on the laptop feeds it sensor state over ArduPilot's JSON protocol, carried as UDP over a PPP link on the second USB endpoint (OTG2) — true HITL, replacing the onboard SIH physics.

**Architecture:** Three layers: (1) sync `AP_Networking` + Socket utility files wholesale to the reviewed ArduPilot-4.6 commit `12a6efea39425b084234551355470341f71f79d4` (field-tested PPP), (2) hand-adapt one small master-only patch (`bfafc2d16b`) so `SIM_JSON` uses the lwIP `SocketAPM` on hardware, (3) a new `--extra-hwdef` overlay that layers JSON frame class + networking on top of the existing SIH overlay. The laptop side is `pppd` + a JSON physics backend; the dependency-light C++ `minimal` example is the required transport/parser smoke test and the PyBullet example is an optional dynamics test.

**Tech Stack:** ArduPilot 4.5.6 (`hitl` base), ChibiOS/CubeOrangePlus, lwIP (submodule pin `143a6a5c`, identical to 4.6 — no bump), gcc-arm-none-eabi-10-2020-q4-major, Python ≤3.11 venv (empy 3.3.4).

## Global Constraints

- Branch: `hitl-network` (created off `hitl`).
- **NEVER run `./waf ... --upload`.** Build `.apj`/`.hex` only; the user flashes boards manually.
- Build environment for ChibiOS targets: `source ~/venv-ap311/bin/activate` and `export PATH="$HOME/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"`.
- Configure command for the HITL build (once Task 3 exists):
  `./waf configure --board IF800-CubeOrangePlus --extra-hwdef=libraries/AP_HAL_ChibiOS/hwdef/IF800-CubeOrangePlus/json-extra-hwdef.dat`
- `libraries/AP_HAL_ChibiOS/hwdef/IF800-CubeOrangePlus/defaults.parm` has **uncommitted bench edits** in the working tree. Do not commit or revert them as a side effect; Task 4 appends new lines and commits only after the user has decided what to do with the existing diff.
- Keep the guard name `HAL_SIM_JSON_ENABLED` (this tree's name). Do NOT port master's `AP_SIM_JSON_ENABLED` rename commit (`2c0e25cb5d`).
- Upstream refs available locally: `upstream/ArduPilot-4.6`, `upstream/master`.
- Pin the networking import to `12a6efea39425b084234551355470341f71f79d4`; do not substitute the moving `upstream/ArduPilot-4.6` ref without re-reviewing its diff.
- Every desktop JSON/SITL launch in this plan must explicitly use `-v ArduCopter`. Do not rely on `sim_vehicle.py` vehicle inference and do not substitute Rover.

## Reference: verified facts (2026-07-23 investigation)

- JSON-on-hardware never shipped in a stable release; it exists only on master (`bfafc2d16b`, Sept 2025). 4.6 stable's `SIM_JSON` is still SITL-only.
- `hitl` already contains: lwIP module, `AP_Networking` (incl. `AP_Networking_PPP.cpp`), `Socket.h`/`Socket_native.h` split, `SerialProtocol_PPP = 48` (`libraries/AP_SerialManager/AP_SerialManager.h:84`).
- Networking delta from `hitl` to pinned commit `12a6efea39425b084234551355470341f71f79d4`: 28 path-touching commits, 776 insertions and 76 deletions across 14 files under `AP_Networking` and the Socket utilities. Socket API change is additive only (`sendto(uint32_t)`, `last_recv_address()`).
- `NEW_NOTHROW` prerequisite: **DONE** — commit `8539da62d5` on `hitl-network` adds the macro + `#include <new>` to `libraries/AP_Common/AP_Common.h`, matching 4.6.
- 4.6 PPP: backend auto-selected when a serial port has protocol 48 (`AP_Networking.cpp:159`). Board default IP `192.168.144.14`, gateway/laptop `192.168.144.1` (`AP_Networking_Config.h:78-93`).
- CubeOrange serial order (`libraries/AP_HAL_ChibiOS/hwdef/CubeOrange/hwdef.inc:31`): `OTG1 USART2 USART3 UART4 UART8 UART7 OTG2` → **SERIAL6 = OTG2** (second USB CDC endpoint). PPP there keeps SERIAL0 MAVLink on the same cable.
- Sim aircraft instantiation on hardware: `libraries/AP_HAL/SIMState.cpp:75` — `SITL::AP_SIM_FRAME_CLASS::create(AP_SIM_FRAME_STRING)`; frame class/string come from hwdef defines (current SIH overlay sets `MultiCopter`/`"quad"`).
- `JSON::JSON()` parses the target IP from the frame string after `:` (`SIM_JSON.cpp:66-69`), ctor init `sock(true)` works for both `SocketAPM_native` and `SocketAPM`.
- Boot caveat (upstream wiki): with JSON physics the board blocks in `recv_fdm()` until the physics backend is reachable — PPP link + physics server must be up before/at board boot.

---

### Task 1: Sync networking stack to pinned ArduPilot-4.6 state

**Files:**
- Modify (wholesale replace from commit `12a6efea39425b084234551355470341f71f79d4`): `libraries/AP_Networking/` (entire directory), `libraries/AP_HAL/utility/Socket.h`, `Socket.cpp`, `Socket.hpp`, `Socket_native.h`, `Socket_native.cpp`

**Interfaces:**
- Produces: 4.6 `SocketAPM` (lwIP) class used by Task 2's `SIM_JSON`; PPP backend auto-started when a serial port has protocol 48.

- [ ] **Step 1: Preflight the exact paths before overwriting anything**

```bash
git status --short -- \
  libraries/AP_Networking \
  libraries/AP_HAL/utility/Socket.h libraries/AP_HAL/utility/Socket.cpp \
  libraries/AP_HAL/utility/Socket.hpp \
  libraries/AP_HAL/utility/Socket_native.h libraries/AP_HAL/utility/Socket_native.cpp
```

Expected: no output. If any selected path is modified or untracked, stop and get the user's decision before checkout. Also run `git status --short` and confirm the known `defaults.parm` bench edits remain outside this task.

- [ ] **Step 2: Checkout the pinned 4.6 file state**

```bash
git checkout 12a6efea39425b084234551355470341f71f79d4 -- \
  libraries/AP_Networking \
  libraries/AP_HAL/utility/Socket.h libraries/AP_HAL/utility/Socket.cpp \
  libraries/AP_HAL/utility/Socket.hpp \
  libraries/AP_HAL/utility/Socket_native.h libraries/AP_HAL/utility/Socket_native.cpp
git status --short
```

Expected: only the known `defaults.parm`/untracked bench files plus M/A entries under the selected networking paths.

- [ ] **Step 3: Check for files deleted in the pinned tree that remain locally**

```bash
diff <(git ls-tree -r --name-only HEAD libraries/AP_Networking) \
     <(git ls-tree -r --name-only 12a6efea39425b084234551355470341f71f79d4 libraries/AP_Networking)
```
Expected: any line prefixed `<` only (local-only file) must be `git rm`-ed. If none, continue.

- [ ] **Step 4: Regression-build desktop SITL (compiles AP_Networking natively)**

```bash
source ~/venv-ap311/bin/activate
./waf configure --board sitl
./waf copter
```
Expected: `'copter' finished successfully`. If it fails, identify the exact missing dependency in pinned commit `12a6efea39425b084234551355470341f71f79d4` and review that dependency before importing it. Do not pull an arbitrary helper from the moving branch or broaden the backport without recording the reason and adding a focused build check.

- [ ] **Step 5: Regression-build the existing SIH board variant (networking compiled out)**

```bash
export PATH="$HOME/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
./waf configure --board IF800-CubeOrangePlus \
  --extra-hwdef=libraries/AP_HAL_ChibiOS/hwdef/IF800-CubeOrangePlus/sih-extra-hwdef.dat
./waf copter
```
Expected: `'copter' finished successfully` — proves the sync is inert when networking is disabled.

- [ ] **Step 6: Audit and commit only the networking sync**

```bash
git add libraries/AP_Networking libraries/AP_HAL/utility/Socket*
git diff --cached --check
git diff --cached --stat
git diff --cached --name-only
git commit -m "AP_Networking, AP_HAL: sync networking stack to ArduPilot-4.6

Wholesale sync of AP_Networking and the Socket utility classes to the
released 4.6 state: robust PPP, timeout options, additive socket API
(sendto by u32 addr, last_recv_address). lwIP submodule pin unchanged.
Groundwork for external-physics HITL over PPP/USB."
```

---

### Task 2: SIM_JSON on hardware (adapt master commit bfafc2d16b)

**Files:**
- Modify: `libraries/SITL/SIM_JSON.h:25` (include), `:67` (socket member)
- Modify: `libraries/SITL/SIM_JSON.cpp:~146-280` (parse_array + three parse cases)

**Interfaces:**
- Consumes: `SocketAPM` from Task 1.
- Produces: `SITL::JSON` compilable under ChibiOS when hwdef defines `HAL_SIM_JSON_ENABLED 1` (verified by Task 3).

- [ ] **Step 1: SIM_JSON.h — socket include + conditional member**

Replace line 25 `#include <AP_HAL/utility/Socket_native.h>` with:

```cpp
#include <AP_HAL/utility/Socket.h>
#include <AP_HAL/utility/Socket_native.h>
```

Replace line 67 `    SocketAPM_native sock;` with:

```cpp
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SocketAPM_native sock;
#else
    // sim-on-hardware: lwIP socket over AP_Networking (PPP/Ethernet)
    SocketAPM sock;
#endif
```

- [ ] **Step 2: SIM_JSON.cpp — replace sscanf float parsing (unavailable on ChibiOS newlib) with parse_array**

Insert immediately before `uint32_t JSON::parse_sensors(const char *json)` (~line 146, after the "not at all general purpose" comment block):

```cpp
template <typename T>
bool parse_array(const char *str, T &arr, int count) {
    const char *p = str;

    for (int i = 0; i < count; i++) {
        // Skip to start of number
        while (*p && *p != '-' && (*p < '0' || *p > '9'))
            p++;

        if (!*p) {
            // End of string before we got all numbers
            return false;
        }

        // Match the current upstream follow-up to bfafc2d16b: preserve double
        // precision for Vector3d and avoid an implicit float narrowing warning.
        arr[i] = strtod(p, nullptr);

        // Move past the number
        while (*p && *p != ',' && *p != ']')
            p++;

        if (i < count - 1) { // expect comma between numbers
            if (*p != ',') return false;
            p++; // skip comma
        }
    }

    return true;
}
```

In `parse_sensors()`, replace the three sscanf cases:

```cpp
            case DATA_VECTOR3F: {
                Vector3<float> v;
                if (!parse_array(p, v, ARRAY_SIZE(v))) {
                    printf("Failed to parse Vector3f for %s/%s\n", key.section, key.key);
                    return received_bitmask;
                }
                Vector3f *tv = (Vector3<float> *)key.ptr;
                *tv = v;
                break;
            }

            case DATA_VECTOR3D: {
                Vector3<double> v;
                if (!parse_array(p, v, ARRAY_SIZE(v))) {
                    printf("Failed to parse Vector3d for %s/%s\n", key.section, key.key);
                    return received_bitmask;
                }
                Vector3d *tv = (Vector3d *)key.ptr;
                *tv = v;
                break;
            }

            case QUATERNION: {
                VectorN<float, 4> v;
                if (!parse_array(p, v, ARRAY_SIZE(v))) {
                    printf("Failed to parse Vector4f for %s/%s\n", key.section, key.key);
                    return received_bitmask;
                }
                Quaternion *tv = static_cast<Quaternion*>(key.ptr);
                tv->q1 = v[0];
                tv->q2 = v[1];
                tv->q3 = v[2];
                tv->q4 = v[3];
                break;
            }
```

If `VectorN` is unknown, add `#include <AP_Math/vectorN.h>` to SIM_JSON.cpp's include block. Note: `ARRAY_SIZE(v)` on a `Vector3<T>`/`VectorN` works here because upstream's patch uses exactly this form. The original patch used `atof`; use the later upstream correction, `strtod(p, nullptr)`, shown above.

- [ ] **Step 3: Build desktop SITL (JSON backend compiles there today)**

```bash
./waf configure --board sitl && ./waf copter
```
Expected: `'copter' finished successfully`.

- [ ] **Step 4: Functional test — desktop ArduCopter SITL against the dependency-light C++ example**

```bash
cd /home/ak/git/IFT/IFT-ardupilot
# one-time build (kept outside the repository):
cmake -S libraries/SITL/examples/JSON/C++ -B /tmp/ardupilot-json-example
cmake --build /tmp/ardupilot-json-example --target minimal

# terminal A (JSON state backend):
/tmp/ardupilot-json-example/minimal

# terminal B (flight code; vehicle must be ArduCopter):
Tools/autotest/sim_vehicle.py -v ArduCopter -f json:127.0.0.1 --console
```
Expected: the C++ example reports a connection, the SITL console prints `JSON received: timestamp gyro accel_body position ...`, and EKF converges without repeated "JSON sensor data not received" messages. This validates the servo packet, reply-address discovery, and rewritten parser before any hardware involvement.

Optional dynamics test, only after its additional dependencies are installed and verified:

```bash
# robot.py imports both pybullet and pyrobolearn; installing pybullet alone is
# insufficient. Its --vehicle argument is required, and "iris" is the working
# ArduCopter model (the script's advertised "quad" choice is not implemented).
python3 libraries/SITL/examples/JSON/pybullet/robot.py --vehicle iris
Tools/autotest/sim_vehicle.py -v ArduCopter -f json:127.0.0.1 --console
```

Do not make the optional PyBullet environment a prerequisite for the C++ smoke test.

- [ ] **Step 5: Commit**

```bash
git add libraries/SITL/SIM_JSON.h libraries/SITL/SIM_JSON.cpp
git diff --cached --check
git diff --cached --stat
git diff --cached --name-only
git commit -m "SITL: allow JSON physics backend on hardware builds

Adapted from upstream master bfafc2d16b: use lwIP SocketAPM when not
building for desktop SITL, and replace sscanf float parsing (not
supported by the ChibiOS newlib config) with a hand-rolled parser.
Guard name kept as HAL_SIM_JSON_ENABLED (pre-rename tree)."
```

---

### Task 3: Instantiate JSON model and compile the hardware JSON path

**Files:**
- Modify: `libraries/AP_HAL/SIMState.cpp:22` (include block)
- Create: `libraries/AP_HAL_ChibiOS/hwdef/IF800-CubeOrangePlus/json-extra-hwdef.dat`

**Interfaces:**
- Consumes: `SITL::JSON::create()` (Task 2) and PPP networking (Task 1).
- Produces: a hardware build that actually compiles the ChibiOS `SocketAPM` JSON path, rather than merely compiling the header with JSON disabled.

- [ ] **Step 1: Add the include**

After `#include <SITL/SIM_Blimp.h>` (line 22) add:

```cpp
#include <SITL/SIM_JSON.h>
```

(`SIM_JSON.h` self-guards on `HAL_SIM_JSON_ENABLED`, so this is inert for all other builds.)

- [ ] **Step 2: Create the external-physics overlay**

`json-extra-hwdef.dat`:

```
# External-physics HITL overlay for IF800-CubeOrangePlus.
# Layers on top of the SIH overlay: same bench-only flight firmware, but the
# physics runs on the laptop and reaches the board as JSON-over-UDP via PPP on
# the second CDC interface (OTG2 / SERIAL6).
# Pass via: ./waf configure --board IF800-CubeOrangePlus --extra-hwdef=<this file>

include sih-extra-hwdef.dat

# physics comes from the JSON backend instead of the onboard MultiCopter model
undef AP_SIM_FRAME_CLASS
undef AP_SIM_FRAME_STRING
define AP_SIM_FRAME_CLASS JSON
define AP_SIM_FRAME_STRING "json:192.168.144.1"

# compile the JSON backend on this hardware build
define HAL_SIM_JSON_ENABLED 1

# IP networking over PPP (laptop = 192.168.144.1, board = 192.168.144.14)
define AP_NETWORKING_ENABLED 1
define AP_NETWORKING_BACKEND_PPP 1
```

- [ ] **Step 3: Compile the real ChibiOS JSON path before committing**

```bash
./waf configure --board IF800-CubeOrangePlus \
  --extra-hwdef=libraries/AP_HAL_ChibiOS/hwdef/IF800-CubeOrangePlus/json-extra-hwdef.dat
./waf copter
```
Expected: `'copter' finished successfully`. Confirm the generated build configuration contains all of:

```bash
rg -n 'HAL_SIM_JSON_ENABLED|AP_NETWORKING_ENABLED|AP_NETWORKING_BACKEND_PPP|AP_SIM_FRAME_CLASS JSON' \
  build/IF800-CubeOrangePlus/hwdef.h
```

This is the acceptance test for Task 2's ChibiOS compatibility claim; a normal SIH build would compile the modified JSON code out and is not sufficient.

- [ ] **Step 4: Commit the wiring and overlay together**

```bash
git add libraries/AP_HAL/SIMState.cpp \
        libraries/AP_HAL_ChibiOS/hwdef/IF800-CubeOrangePlus/json-extra-hwdef.dat
git diff --cached --check
git diff --cached --stat
git diff --cached --name-only
git commit -m "AP_HAL: wire external JSON physics into IF800 HITL build"
```

---

### Task 4: Add the PPP parameter defaults without capturing bench edits

**Files:**
- Modify: `libraries/AP_HAL_ChibiOS/hwdef/IF800-CubeOrangePlus/defaults.parm` (append 2 lines — see uncommitted-edits constraint)

**Interfaces:**
- Consumes: the JSON/PPP overlay from Task 3.
- Produces: embedded defaults that request networking and select PPP on SERIAL6.

- [ ] **Step 1: Resolve the pre-existing dirty-file decision before editing**

Show the user the complete pre-existing diff:

```bash
git diff -- libraries/AP_HAL_ChibiOS/hwdef/IF800-CubeOrangePlus/defaults.parm
```

The user must choose one of these paths before proceeding:

1. Commit the bench fixes separately, leaving `defaults.parm` clean; or
2. Keep the bench fixes uncommitted and use interactive staging in Step 3 so only the two new lines enter this commit.

Do not use a full-file `git add` while the bench edits remain unstaged.

- [ ] **Step 2: Append the parameter defaults**

Append:

```
NET_ENABLE 1
SERIAL6_PROTOCOL 48
```

(SERIAL6 = OTG2, the second USB CDC endpoint; protocol 48 = PPP, which also auto-selects the PPP networking backend.)

- [ ] **Step 3: Stage and prove that only the two new lines are selected**

```bash
# If defaults.parm was made clean in Step 1, a normal git add is safe.
# Otherwise use interactive staging and select only the EOF hunk containing
# NET_ENABLE and SERIAL6_PROTOCOL:
git add -p libraries/AP_HAL_ChibiOS/hwdef/IF800-CubeOrangePlus/defaults.parm

git diff --cached --check
git diff --cached --name-only
git diff --cached -- libraries/AP_HAL_ChibiOS/hwdef/IF800-CubeOrangePlus/defaults.parm
```

Expected cached diff: exactly the two appended parameter lines and no bench-fix hunks. If the index contains anything else, stop and correct the staging selection before committing.

- [ ] **Step 4: Commit the two defaults**

```bash
git commit -m "IF800-CubeOrangePlus: enable PPP defaults for external HITL"
```

---

### Task 5: HITL firmware build + embedded-default, flash, and RAM checks

**Files:**
- Normally none modified (build task). If size trims are required, modify only `json-extra-hwdef.dat` and commit them separately. Artifacts: `build/IF800-CubeOrangePlus/bin/arducopter.apj` etc.

- [ ] **Step 1: Configure + build**

```bash
./waf configure --board IF800-CubeOrangePlus \
  --extra-hwdef=libraries/AP_HAL_ChibiOS/hwdef/IF800-CubeOrangePlus/json-extra-hwdef.dat
./waf copter
```
Expected: configure prints `Default parameters path from hwdef: .../IF800-CubeOrangePlus/defaults.parm`; build ends `'copter' finished successfully` with a `Free flash` figure in the link summary.

- [ ] **Step 2: Prove that the intended defaults and features entered the artifact**

```bash
rg -n '^(NET_ENABLE 1|SERIAL6_PROTOCOL 48)$' \
  build/IF800-CubeOrangePlus/processed_defaults.parm
rg -n 'HAL_SIM_JSON_ENABLED|AP_NETWORKING_ENABLED|AP_NETWORKING_BACKEND_PPP|AP_SIM_FRAME_CLASS JSON' \
  build/IF800-CubeOrangePlus/hwdef.h
arm-none-eabi-size build/IF800-CubeOrangePlus/bin/arducopter
```

Expected: both parameter lines occur exactly once, all four compile-time features are present, and `arm-none-eabi-size` succeeds. Record the text/data/bss figures and the linker's free-flash figure. The JSON model owns a 65 kB receive buffer and PPP/lwIP adds buffers and threads, so a flash-only check is not sufficient.

- [ ] **Step 3: If flash overflows** — add trims to `json-extra-hwdef.dat` one at a time, rebuilding after each, drawn from this candidate list (safe for a bench HITL image): `define HAL_OSD_ENABLED 0`, `define AP_OPTICALFLOW_ENABLED 0`, `define HAL_PARACHUTE_ENABLED 0`, `define AP_GRIPPER_ENABLED 0`, `define AP_LANDINGGEAR_ENABLED 0`, `define AP_WINCH_ENABLED 0`. Do NOT trim mount, camera, or scripting. Keep DroneCAN compiled unless the user explicitly confirms it is unnecessary for this bench image. Commit any required trims separately with their measured size impact.

- [ ] **Step 4: Do NOT flash.** Report the artifact path, SHA-256, free-flash number, and size output; the user uploads manually.

```bash
sha256sum build/IF800-CubeOrangePlus/bin/arducopter.apj
```

---

### Task 6: Docs — extend and renumber hitl-instructions.md

**Files:**
- Modify: `docs/hitl-instructions.md` (insert a new top-level section "2. External-physics HITL (JSON over PPP/USB)" and renumber the existing desktop-SITL section from 2/2.x to 3/3.x)

- [ ] **Step 1: Write the section** covering, in this order:
  1. Concept diagram (one paragraph): board runs ArduCopter flight code, laptop runs physics; JSON/UDP over PPP on OTG2; SERIAL0 MAVLink remains on the first CDC interface of the same USB cable. Clarify that `sim_vehicle.py` is not run during the hardware test because the board is the flight controller.
  2. Build: the Task 5 configure/build commands (no `--upload`; user flashes).
  3. Post-flash parameter precedence warning: saved FRAM values override embedded defaults. Require the operator to read back `NET_ENABLE == 1`, `SERIAL6_PROTOCOL == 48`, and `SERVO_32_ENABLE == 0`, set any stale value, and reboot. The bundled baseline backends expect the 16-channel packet with magic 18458; `SERVO_32_ENABLE == 1` changes the packet and magic. Also require confirmation of `NET: Initialized` and `PPP: started`. Cross-reference the existing storage-reset discussion rather than silently assuming defaults won.
  4. Laptop PPP bring-up. Explain how to identify the second CDC interface using `/dev/serial/by-id/` or `udevadm`; `/dev/ttyACM1` is only a typical example and enumeration order is not guaranteed:

     ```bash
     sudo pppd /dev/ttyACM1 115200 192.168.144.1:192.168.144.14 \
          noauth local nodetach persist maxfail 0
     # verify: ping 192.168.144.14
     ```
     Note that 115200 is the host TTY line setting; the CDC transport itself is USB and is not physically baud-limited like a UART.
  5. Required baseline backend: build and run `/tmp/ardupilot-json-example/minimal` using Task 2's commands. It listens on UDP 9002 and discovers the board from periodic servo packets. Do not promise a ten-second interval; this tree currently retries at roughly one second.
  6. Optional PyBullet dynamics backend: document that `robot.py` needs both PyBullet and PyRoboLearn, then run `python3 libraries/SITL/examples/JSON/pybullet/robot.py --vehicle iris`. Do not document `pip install pybullet` as sufficient and do not use the script's broken `quad` choice.
  7. Desktop-only parser regression command, clearly separated from hardware HITL: `Tools/autotest/sim_vehicle.py -v ArduCopter -f json:127.0.0.1 --console`. State explicitly that the vehicle must be `ArduCopter`.
  8. **Boot order caveat** (warning box): the board's lockstep main loop blocks waiting for physics. Start persistent `pppd` and the backend, then power/reset the board. A first USB enumeration may temporarily disconnect PPP; `persist` should reconnect it.
  9. Bench verification checklist: parameters read back correctly → `NET: Initialized`/`PPP: started` → `ping 192.168.144.14` succeeds → backend receives magic 18458 servo packets → GCS on SERIAL0 shows the JSON state → `JSON received: ...` fields match the backend. Record the boot banner's initial `Free RAM`, then retrieve `@SYS/memory.txt` and `@SYS/threads.txt` through MAVFTP after JSON and PPP are active; the boot banner occurs before the JSON model's dynamic allocation and is not a sufficient post-start memory check.
  10. Failure/recovery test: stop the physics backend long enough to exercise lockstep blocking, restart it, and verify whether the board resumes or watchdog-reboots; then interrupt/restart `pppd` and confirm the 4.6 PPP reconnect logic restores the path. Record the observed behavior rather than assuming recovery.

- [ ] **Step 2: Renumber the existing desktop-SITL section**

Change existing heading `## 2. SITL (desktop simulation)` to `## 3...`, all `### 2.x` headings to `### 3.x`, and textual `§2` cross-references accordingly. Verify there are no duplicate section numbers:

```bash
rg -n '^## |^### ' docs/hitl-instructions.md
```

- [ ] **Step 3: Commit**

```bash
git add docs/hitl-instructions.md
git commit -m "docs/hitl: add external-physics HITL (JSON over PPP/USB) instructions"
```

---

### Deferred (explicitly out of scope for this plan)

- **Non-lockstep mode** (`759e03f1aa`): depends on master-only SIM_JSON refactors (64-bit sensor bitmask `b7ae62c57e`, RC keys). Revisit only if lockstep blocking proves unworkable on the bench.
- Global-coordinate JSON inputs (`1f7b57a770`), Simulink glue in `sitl-on-hw.py`, and the 4.6→master PPP deltas.

## Verification (end-to-end)

1. Task 1 builds both desktop SITL and the unchanged SIH board after the pinned networking sync.
2. Task 2 Step 4 proves the JSON packet and parser path with desktop `-v ArduCopter` before hardware.
3. Task 3 compiles `HAL_SIM_JSON_ENABLED=1`, lwIP `SocketAPM`, PPP, and `AP_SIM_FRAME_CLASS JSON` together for the real ChibiOS target.
4. Task 5 proves the intended parameters entered `processed_defaults.parm`, records text/data/bss, free flash, artifact hash, and the exact output path.
5. Bench (user-driven, after manual flash): read-back parameters → network/PPP startup messages → PPP ping → physics handshake → JSON state in GCS → free-RAM observation.
6. Bench recovery: restart the physics backend and PPP independently and record resume versus watchdog-reset behavior.
