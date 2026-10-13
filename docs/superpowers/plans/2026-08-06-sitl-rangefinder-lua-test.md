# SITL Rangefinder Test Harness for limit-descent-speed.lua — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Exercise every branch of `~/Documents/IFT/rangefinder/limit-descent-speed.lua` against a simulated downward rangefinder in desktop ArduCopter SITL, with an automated pytest harness that asserts on statustexts and the parameters the script overrides.

**Architecture:** Desktop SITL only — no hardware, no sim-on-hardware. A pytest fixture launches `build/sitl/bin/arducopter` directly (no `sim_vehicle.py`, no MAVProxy) with a `--defaults` param file, in a dedicated working directory whose `scripts/` subdir holds a symlink to the canonical Lua file. Tests connect over `tcp:127.0.0.1:5760` with pymavlink, drive the aircraft with MAVLink RC overrides, and read back `LAND_SPEED` / `LOIT_SPEED` / `PILOT_SPEED_DN` to confirm the script's effect.

**Tech Stack:** ArduCopter 4.5.6 SITL (`--board sitl`), pymavlink 2.4.49, pytest 9.0.2, Python 3.10.

## Global Constraints

- Desktop SITL only. Nothing in this plan touches the IF800 board, `sih-extra-hwdef.dat`, or `defaults.parm`.
- The canonical Lua file is `~/Documents/IFT/rangefinder/limit-descent-speed.lua`. It is **symlinked**, never copied — edits to the original must take effect on the next SITL start.
- The Lua script is **not modified** by this plan. If a test fails, the finding is reported; fixing the script is separate work.
- All SITL artifacts (`eeprom.bin`, `logs/`, `terrain/`) live in `~/Documents/IFT/rangefinder/sitl/` and never in the ardupilot git repo.
- `RNGFND1_TYPE` is `100` (`Type::SIM`). Verified: `AP_RangeFinder_config.h:137` enables `AP_RANGEFINDER_SIM_ENABLED` whenever `CONFIG_HAL_BOARD == HAL_BOARD_SITL`, so a stock SITL build already has the backend.
- Params are supplied via the binary's `--defaults` flag so they are in place **before** `RangeFinder::init()` runs. Never set `RNGFND1_TYPE` over MAVLink at startup — it needs a reboot to instantiate a backend.
- MAVLink stream rates are **off by default** and this harness has no MAVProxy to turn them on, so `rangefinder.parm` sets `SR0_*` explicitly. Without that, `GLOBAL_POSITION_INT` and `RANGEFINDER` never arrive and every altitude-driven helper hangs.
- `--speedup 10` for all tests.

## Known Firmware Behaviour This Plan Relies On

Each verified in the 4.5.6 tree in this repo:

| Fact | Location |
|---|---|
| `Type::SIM = 100` | `AP_RangeFinder.h:178` |
| SIM backend reads `AP::sitl()->get_rangefinder(instance)` | `AP_RangeFinder_SITL.cpp:35` |
| Sim fills all 10 rangefinder slots with `rangefinder_range()` each step | `SIM_Aircraft.cpp:988-992` |
| `rangefinder_range()` returns `state.height_agl`, attitude- and beam-corrected, plus `SIM_SONAR_RND` noise | `SIM_Aircraft.cpp:520-578` |
| Setting `RNGFND1_TYPE 0` **at runtime** gives `Status::NotConnected` with no reboot, and the driver stays allocated so restoring the type resumes it | `AP_RangeFinder.cpp:236-241` |
| `update_status()` compares against `RNGFND1_MIN_CM`/`MAX_CM` live, so shrinking them mid-flight forces OutOfRangeLow/High | `AP_RangeFinder_Backend.cpp` |
| SITL Lua script directory is `./scripts`, relative to the SITL process CWD | `lua_common_defs.h:17` |
| SITL binary flags: `--model`, `--home`, `--defaults`, `--speedup`, `--instance`, `--wipe` | `SITL_cmdline.cpp:291-330` |
| SITL's `serial0` is `tcp:0:wait`, so the binary **blocks at boot until a client connects** — the `started` statustext can never be missed by connecting late | `SITL_State.h:33` |
| Every `SR0_*` stream rate defaults to **0**. Nothing streams until a GCS asks. `sim_vehicle.py` only works because MAVProxy calls `request_data_stream_send` | `ArduCopter/GCS_Mavlink.cpp:401-491` |
| Statustexts longer than 50 bytes are split into chunks sharing a nonzero `id` and numbered by `chunk_seq`; the loop breaks once a chunk is shorter than 50, so the final chunk is always `< 50` | `GCS_Common.cpp:2251-2283` |
| SITL's statustext queue holds **7** entries, not 30 — a 76-char warning burns two slots | `GCS.h:1293` |
| `LAND_SPEED_HIGH` defaults to 0, meaning "use `WPNAV_SPEED_DN`" = 150 cm/s | `Parameters.cpp:207`, `AC_WPNav.cpp:12` |
| SITL seeds RC input at boot (ch1/2/4 = 1500, ch3/6/7 = 1000, ch5/8 = 1800), so there is **no** RC failsafe and the vehicle arms without overrides | `SITL_State.cpp:563-565` |
| The SIM backend writes raw `distance_m` with no MIN/MAX clamp, so shrinking those params at runtime genuinely forces OutOfRange | `AP_RangeFinder_SITL.cpp:43` |

## Script Behaviour Under Test

Parameters the script creates at boot (`param:add_table` key 72, prefix `RFL_`): `ENABLE 1`, `ALT_HI 5.0`, `SPD_HI 0.5`, `ALT_LO 1.0`, `SPD_LO 0.25`, `DSC_TH 0.2`.

Expected statustexts — **exact** text, copied from the script's format strings at lines 314, 324-325, 282, 367-368 and 383. Note `limiting to **no more than** X m/s`; there is no shorter form:

- `RangefinderLimiter: started`
- `RangefinderLimiter: limiting to no more than 0.50 m/s`
- `RangefinderLimiter: limiting to no more than 0.25 m/s`
- `RangefinderLimiter: limits released`
- `RangefinderLimiter: rangefinder unhealthy, normal flight restored`
- `RangefinderLimiter: rangefinder healthy`

**MAVLink chunks these at 50 bytes, and pymavlink does not reassemble them.** `msg.text` is one raw chunk; pymavlink 2.4.49's `mavutil.py` contains no reference to `chunk_seq` at all. With the 20-character `RangefinderLimiter: ` prefix, these split:

| Message body | Total with prefix | Chunk 0 ends at |
|---|---|---|
| `limiting to no more than 0.50 m/s` | 53 | `…to no more than 0.50 ` |
| `rangefinder unhealthy, normal flight restored` | 65 | `…unhealthy, normal ` |
| `no downward rangefinder, speed limiting inactive` | 68 | `…rangefinder, speed` |
| `rangefinder max range below RFL_ALT_HI, upper band unused` | 77 | `…max range below RF` |
| `LAND_SPEED is 100 cm/s, looks like a leftover override` | 74 | `…100 cm/s, looks ` |

Single-chunk and therefore safe to match raw: `started` (27), `limits released` (35), `rangefinder healthy` (39), `inactive, reboot to clear` (45).

Each row above was checked by replaying ArduPilot's chunking loop against the reassembler; all nine messages come back byte-identical with no orphaned partial chunks.

`SitlCopter` reassembles chunks by `id` before recording a statustext, so tests assert on the whole string. Skipping that breaks assertions two ways: a positive assertion never matches, and — more dangerously — an *absence* assertion like `"max range below RFL_ALT_HI" not in ...` passes even when the warning fired, because the substring straddles the chunk boundary.

Levers per mode, from `apply_limit` (script lines 296-311):

| Mode | Parameters written |
|---|---|
| LOITER | `PILOT_SPEED_DN` **and** `LOIT_SPEED` |
| LAND | `LAND_SPEED` **and** `LAND_SPEED_HIGH` |

Both modes write two parameters. `LAND_SPEED_HIGH` defaults to 0 (`Parameters.cpp:207`), which means "fall back to `WPNAV_SPEED_DN`" = 150 cm/s (`AC_WPNav.cpp:12`); the script resolves that into `baseline_land_speed_high` at boot (lines 155-158), clamps against it, and `restore_originals` writes the literal 0 back (line 277). Tests must assert **both** levers in both modes, or the LAND write path is only half covered.

**Baseline collision to design around.** Stock Copter defaults are `LAND_SPEED 50` (`config.h:348`) and `RFL_SPD_HI` is `0.5 m/s` = 50 cm/s — identical, so the 5 m band in LAND would emit a statustext with no rate change. The script's own comment at lines 152-154 anticipates this. **This plan sets `LAND_SPEED 100` in the param file** so all three LAND rates (1.00 / 0.50 / 0.25 m/s) are distinct. 100 is safely above the script's 25 cm/s "leftover override" threshold, so `check_baseline` still passes.

Other stock defaults that matter: `LOIT_SPEED 1250` (`AC_Loiter.cpp:7`), `WPNAV_SPEED_DN 150` (`AC_WPNav.cpp:12`), `PILOT_SPEED_DN 0` (`Parameters.cpp:887`) which means "use `PILOT_SPEED_UP`" = `250` cm/s (`config.h:506`). `PILOT_SPEED_DN 0` is legal and the script skips its baseline check for it (`if original_pilot_speed_dn > 0`), so the limiter stays usable.

**The descent latch is sticky, and the tests depend on that.** `update_descent_latch` (lines 404-417) engages above `RFL_DSC_TH` and only releases on a climb faster than `climb_release_rate` = -0.1 m/s. Once a test has descended into a band it can return the throttle to neutral and the limits stay applied. Every test does this, so that assertions are not racing the aircraft toward the ground — see the note on `descend_to` in Task 2.

## File Structure

All under `~/Documents/IFT/rangefinder/sitl/` unless noted.

| File | Responsibility |
|---|---|
| `scripts/limit-descent-speed.lua` | Symlink → `../../limit-descent-speed.lua`. The only thing SITL loads. |
| `rangefinder.parm` | Rangefinder + scripting + baseline speed params, fed to `--defaults`. |
| `sitl_harness.py` | `SitlCopter` class: launch/teardown SITL, connect, RC override thread, arm/takeoff/mode helpers, statustext capture, param read/write. No test logic. |
| `conftest.py` | pytest fixtures wrapping `SitlCopter` — one fresh SITL per test. |
| `pytest.ini` | Registers the `speedup` marker used to slow one LAND test down. |
| `test_loiter_bands.py` | LOITER branch: upper band, lower band, latch release on climb, live-tuning floor. |
| `test_land_bands.py` | LAND branch: all three descent rates. |
| `test_sensor_faults.py` | Fault injection and out-of-range paths. |
| `README.md` | How to run, what each test covers, what is not covered. |

Split by *behaviour under test*, not by technical layer — `sitl_harness.py` holds everything mechanical so the three test files read as behaviour specs.

---

### Task 1: SITL build, working directory, and a smoke test

**Files:**
- Create: `~/Documents/IFT/rangefinder/sitl/rangefinder.parm`
- Create: `~/Documents/IFT/rangefinder/sitl/scripts/limit-descent-speed.lua` (symlink)
- Create: `~/Documents/IFT/rangefinder/sitl/.gitignore`

**Interfaces:**
- Consumes: nothing.
- Produces: a runnable SITL working directory, and `rangefinder.parm` which every later task passes to `--defaults`.

- [ ] **Step 1: Build SITL**

This reconfigures waf away from the IF800 board. That is expected and reversible — see Step 7.

```bash
cd /home/ak/git/IFT/IFT-ardupilot
./waf configure --board sitl
./waf copter
```

Expected: `build/sitl/bin/arducopter` exists and is newer than the source tree.

- [ ] **Step 2: Create the working directory and symlink the script**

`~/Documents/IFT/rangefinder` is **not currently a git repo** — it holds only `docs/`, `limit-descent-speed.lua` and `test_program.lua`. Initialise one so the commit steps in this plan are real:

```bash
mkdir -p ~/Documents/IFT/rangefinder/sitl/scripts
cd ~/Documents/IFT/rangefinder
git rev-parse --git-dir 2>/dev/null || git init
ln -sf ../../limit-descent-speed.lua \
       ~/Documents/IFT/rangefinder/sitl/scripts/limit-descent-speed.lua
ls -lL ~/Documents/IFT/rangefinder/sitl/scripts/
```

Expected: the symlink resolves (no "No such file or directory" on `ls -lL`).

- [ ] **Step 3: Write the param file**

Create `~/Documents/IFT/rangefinder/sitl/rangefinder.parm`:

```
# Simulated downward rangefinder for the RFL Lua limiter.
# Loaded via the SITL binary's --defaults flag, so these are in place before
# RangeFinder::init() runs and no reboot is needed to instantiate the backend.
RNGFND1_TYPE 100
RNGFND1_ORIENT 25
RNGFND1_MIN_CM 20
RNGFND1_MAX_CM 5000
RNGFND1_GNDCLEAR 10
SIM_SONAR_RND 0
SIM_SONAR_GLITCH 0

# Every SR0_* stream rate defaults to 0 (ArduCopter/GCS_Mavlink.cpp:401-491) and
# this harness has no MAVProxy to request streams. Without these, no
# GLOBAL_POSITION_INT and no RANGEFINDER ever arrive and every altitude helper
# times out. POSITION carries GLOBAL_POSITION_INT; EXTRA3 carries RANGEFINDER.
SR0_POSITION 10
SR0_EXTRA1 10
SR0_EXTRA2 5
SR0_EXTRA3 5
SR0_EXT_STAT 5

# Lua scripting
SCR_ENABLE 1
SCR_HEAP_SIZE 102400

# Copter's own surface tracking would otherwise consume this rangefinder in
# LOITER, so a sensor dropout would change two things at once. Off, to leave
# the Lua limiter as the only consumer.
SURFTRAK_MODE 0

# LAND_SPEED defaults to 50 cm/s, which is exactly RFL_SPD_HI * 100 -- the 5 m
# band in LAND would then change nothing observable. 100 makes all three LAND
# rates distinct and still clears the script's 25 cm/s baseline check.
LAND_SPEED 100
```

- [ ] **Step 4: Keep SITL droppings out of git**

Create `~/Documents/IFT/rangefinder/sitl/.gitignore`:

```
eeprom.bin
logs/
terrain/
__pycache__/
*.BIN
```

- [ ] **Step 5: Smoke test — does the script load and the rangefinder read AGL?**

```bash
cd ~/Documents/IFT/rangefinder/sitl
/home/ak/git/IFT/IFT-ardupilot/build/sitl/bin/arducopter \
    --model quad \
    --home -35.363261,149.165230,584,353 \
    --defaults /home/ak/git/IFT/IFT-ardupilot/Tools/autotest/default_params/copter.parm,./rangefinder.parm \
    --speedup 1 --wipe
```

In a second terminal:

```bash
mavproxy.py --master tcp:127.0.0.1:5760 --console
```

- [ ] **Step 6: Confirm the smoke test**

In MAVProxy:

```
param show RNGFND1_TYPE
param show RFL_ALT_HI
status RANGEFINDER
```

Expected:
- `RNGFND1_TYPE` reads `100.0`
- `RFL_ALT_HI` reads `5.0` — proves the script ran `param:add_table` and is alive
- Startup statustext `RangefinderLimiter: started`
- **No** `no downward rangefinder` warning, and **no** `rangefinder max range below RFL_ALT_HI` warning
- `RANGEFINDER` reports ~0 m sitting on the ground

MAVProxy reassembles chunked statustexts, so the two warnings appear in full here even though the pytest harness needs its own reassembler to see them. This step is the one place the long messages can be read without extra machinery — use it to confirm the exact wording before trusting any assertion.

If `RFL_*` params are missing, the script did not load — check the symlink resolves and that SITL's CWD is the `sitl/` directory.

Stop both processes.

- [ ] **Step 7: Note how to return to the IF800 build**

No commit for this step — it is a fact to carry forward. Returning to the hardware build needs only a reconfigure (separate build directories, so no `clean`):

```bash
./waf configure --board IF800-CubeOrangePlus \
  --extra-hwdef=libraries/AP_HAL_ChibiOS/hwdef/IF800-CubeOrangePlus/sih-extra-hwdef.dat
```

- [ ] **Step 8: Commit**

```bash
cd ~/Documents/IFT/rangefinder
git add sitl/rangefinder.parm sitl/.gitignore sitl/scripts/limit-descent-speed.lua
git commit -m "sitl: working dir, rangefinder params, script symlink"
```

If `~/Documents/IFT/rangefinder` is not a git repo, skip the commit and note it — the plan does not require one.

---

### Task 2: Harness and the first LOITER band test

**Files:**
- Create: `~/Documents/IFT/rangefinder/sitl/sitl_harness.py`
- Create: `~/Documents/IFT/rangefinder/sitl/conftest.py`
- Test: `~/Documents/IFT/rangefinder/sitl/test_loiter_bands.py`

**Interfaces:**
- Consumes: `rangefinder.parm` from Task 1.
- Produces:
  - `SitlCopter` class with: `start()`, `stop()`, `wait_ready()`, `param_get(name) -> float`, `param_set(name, value)`, `set_mode(name)`, `arm()`, `takeoff(alt_m)`, `descend_to(alt_m)`, `climb()`, `hold()`, `rc(channel, pwm)`, `rc_neutral()`, `altitude() -> float`, `rangefinder_m() -> float`, `latest(msg_type)`, `statustexts() -> list[str]`, `saw_statustext(substring) -> bool`, `wait_statustext(substring, timeout) -> str | None`, `clear_statustexts()`.
  - pytest fixture `copter` yielding a started, armed-ready `SitlCopter`.

Three properties of this harness the tests depend on, all of them fixes for ways the first draft could not have worked:

1. **One rx thread owns the socket.** Every other method reads latest-value slots. A typed `recv_match` discards non-matching messages, so the old `descend_to` — which polled `altitude()` in a tight loop — would have thrown away the `limiting to…` statustext it was about to assert on.
2. **Statustexts are reassembled** from `id`/`chunk_seq` before being recorded, so tests match on whole strings.
3. **`descend_to` returns to neutral throttle.** The descent latch is sticky, so limits persist; without this every assertion after a descent races the aircraft toward the ground.

- [ ] **Step 1: Write the harness**

Create `~/Documents/IFT/rangefinder/sitl/sitl_harness.py`:

```python
"""Launch and drive an ArduCopter SITL instance for rangefinder limiter tests.

One background thread owns the MAVLink socket and dispatches every message into
latest-value slots.  Nothing else may call recv_match: a *typed* recv_match
silently discards every message that does not match its filter, so polling
altitude in a loop would eat the one-shot statustexts the tests wait for.
"""
import os
import signal
import subprocess
import threading
import time

from pymavlink import mavutil

REPO = "/home/ak/git/IFT/IFT-ardupilot"
BINARY = os.path.join(REPO, "build/sitl/bin/arducopter")
COPTER_DEFAULTS = os.path.join(REPO, "Tools/autotest/default_params/copter.parm")
WORKDIR = os.path.dirname(os.path.abspath(__file__))
HOME = "-35.363261,149.165230,584,353"

RC_NEUTRAL = {1: 1500, 2: 1500, 3: 1500, 4: 1500}
RC_THROTTLE_DOWN = 1150
RC_THROTTLE_UP = 1850

# STATUSTEXT carries char text[50].  Anything longer is split by
# GCS_Common.cpp:2251-2283 into chunks sharing a nonzero `id` and numbered by
# `chunk_seq`; the loop breaks once a chunk is short, so the last chunk of a
# split message is always < 50 characters.  pymavlink does not reassemble.
STATUSTEXT_CHUNK_LEN = 50


class SitlCopter:
    def __init__(self, speedup=10, instance=0):
        self.speedup = speedup
        self.instance = instance
        self.port = 5760 + 10 * instance
        self.proc = None
        self.mav = None
        self._texts = []
        self._chunks = {}
        self._latest = {}
        self._params = {}
        self._rc = dict(RC_NEUTRAL)
        self._stop = threading.Event()
        self._rc_thread = None
        self._rx_thread = None
        self._state_lock = threading.Lock()
        self._send_lock = threading.Lock()

    # ---- lifecycle ----------------------------------------------------
    def start(self):
        cmd = [
            BINARY,
            "--model", "quad",
            "--home", HOME,
            "--defaults", f"{COPTER_DEFAULTS},./rangefinder.parm",
            "--speedup", str(self.speedup),
            "--instance", str(self.instance),
            "--wipe",
        ]
        self.proc = subprocess.Popen(
            cmd, cwd=WORKDIR,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        # SITL's serial0 is "tcp:0:wait" (SITL_State.h:33): it listens at once
        # but blocks in accept() until a client arrives, so nothing -- not even
        # the script's startup statustext -- happens before we connect.
        deadline = time.time() + 30
        while time.time() < deadline:
            try:
                self.mav = mavutil.mavlink_connection(
                    f"tcp:127.0.0.1:{self.port}", source_system=255)
                break
            except Exception:
                time.sleep(0.5)
        if self.mav is None:
            raise RuntimeError("could not connect to SITL")

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        if not self._wait_for(lambda: self.latest("HEARTBEAT") is not None,
                              timeout=30):
            raise RuntimeError("no heartbeat from SITL")

        self._rc_thread = threading.Thread(target=self._rc_loop, daemon=True)
        self._rc_thread.start()

    def stop(self):
        self._stop.set()
        for t in (self._rc_thread, self._rx_thread):
            if t:
                t.join(timeout=3)
        if self.mav:
            self.mav.close()
        if self.proc:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)

    # ---- plumbing -----------------------------------------------------
    def _send(self, fn):
        """Serialise sends.  The RC thread and the test thread share one link
        and MAVLink.send() is not reentrant -- it bumps a shared sequence
        counter and writes the socket."""
        with self._send_lock:
            fn()

    def _wait_for(self, predicate, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if predicate():
                return True
            time.sleep(0.02)
        return False

    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                msg = self.mav.recv_match(blocking=True, timeout=0.5)
            except Exception:
                break
            if msg is None or msg.get_type() == "BAD_DATA":
                continue
            with self._state_lock:
                self._latest[msg.get_type()] = msg
                if msg.get_type() == "STATUSTEXT":
                    self._record_statustext(msg)
                elif msg.get_type() == "PARAM_VALUE":
                    self._params[msg.param_id.rstrip("\x00")] = msg.param_value

    def _record_statustext(self, msg):
        """Reassemble chunked statustexts.  Caller holds _state_lock."""
        text = msg.text
        chunk_id = getattr(msg, "id", 0) or 0
        if chunk_id == 0:  # short enough to have been sent whole
            self._texts.append(text)
            return
        parts = self._chunks.setdefault(chunk_id, {})
        parts[getattr(msg, "chunk_seq", 0) or 0] = text
        if len(text) < STATUSTEXT_CHUNK_LEN:  # final chunk, always short
            del self._chunks[chunk_id]
            self._texts.append("".join(parts[i] for i in sorted(parts)))

    def latest(self, msg_type):
        with self._state_lock:
            return self._latest.get(msg_type)

    # ---- RC -----------------------------------------------------------
    def _rc_loop(self):
        """Continuous RC overrides.

        These are not needed to dodge RC failsafe -- SITL seeds its own RC
        input at boot (SITL_State.cpp:563-565), so the vehicle arms without
        them.  They exist only to command throttle, i.e. to drive climbs and
        descents.  Channels 5-8 stay at 0, which MAVLink defines as "release
        this channel", leaving the sim's own switch positions alone.
        """
        while not self._stop.is_set():
            with self._state_lock:
                chans = [self._rc.get(i, 0) for i in range(1, 9)]
            try:
                self._send(lambda: self.mav.mav.rc_channels_override_send(
                    self.mav.target_system, self.mav.target_component, *chans))
            except Exception:
                pass
            time.sleep(0.1)

    def rc(self, channel, pwm):
        with self._state_lock:
            self._rc[channel] = pwm

    def rc_neutral(self):
        with self._state_lock:
            self._rc = dict(RC_NEUTRAL)

    def hold(self):
        """Neutral throttle: stop climbing or descending, hold altitude."""
        self.rc(3, RC_NEUTRAL[3])

    # ---- statustexts --------------------------------------------------
    def statustexts(self):
        with self._state_lock:
            return list(self._texts)

    def clear_statustexts(self):
        with self._state_lock:
            self._texts = []
            self._chunks = {}

    def wait_statustext(self, substring, timeout=30):
        """Return the first reassembled statustext containing `substring`."""
        hit = []

        def seen():
            for t in self.statustexts():
                if substring in t:
                    hit.append(t)
                    return True
            return False

        return hit[0] if self._wait_for(seen, timeout) else None

    def saw_statustext(self, substring):
        return any(substring in t for t in self.statustexts())

    # ---- params -------------------------------------------------------
    def param_get(self, name, timeout=10):
        with self._state_lock:
            self._params.pop(name, None)
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._send(lambda: self.mav.mav.param_request_read_send(
                self.mav.target_system, self.mav.target_component,
                name.encode(), -1))
            if self._wait_for(lambda: name in self._params, timeout=2):
                with self._state_lock:
                    return self._params[name]
        raise TimeoutError(f"no PARAM_VALUE for {name}")

    def param_set(self, name, value, timeout=10):
        """Set, then confirm by read-back.  ArduPilot echoes a PARAM_VALUE on
        success, so callers never need a blind sleep."""
        self._send(lambda: self.mav.mav.param_set_send(
            self.mav.target_system, self.mav.target_component,
            name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32))
        deadline = time.time() + timeout
        while time.time() < deadline:
            if abs(self.param_get(name, timeout=3) - float(value)) < 1e-4:
                return
        raise TimeoutError(f"{name} never took the value {value}")

    # ---- state --------------------------------------------------------
    # Non-blocking reads of the freshest message the rx thread has seen.
    # SR0_POSITION 10 / SR0_EXTRA3 5 are sim-rate, so at --speedup 10 these
    # land at 100 Hz / 50 Hz of wall clock.
    def altitude(self):
        msg = self.latest("GLOBAL_POSITION_INT")
        return msg.relative_alt / 1000.0 if msg else None

    def rangefinder_m(self):
        msg = self.latest("RANGEFINDER")
        return msg.distance if msg else None

    # ---- commands -----------------------------------------------------
    def wait_ready(self, timeout=180):
        """Wait for the Lua script to register its params, then for a fix.

        Raises loudly if RFL_ALT_HI never appears: a Lua load failure must not
        surface later as a confusing arming or statustext timeout.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                if self.param_get("RFL_ALT_HI", timeout=3) == 5.0:
                    break
            except TimeoutError:
                pass
        else:
            raise RuntimeError(
                "RFL_ALT_HI never appeared: the Lua script did not load. "
                "Check that scripts/limit-descent-speed.lua resolves and that "
                "SCR_ENABLE is 1.")

        gps_ok = lambda: (self.latest("GPS_RAW_INT") is not None
                          and self.latest("GPS_RAW_INT").fix_type >= 3
                          and self.altitude() is not None)
        if not self._wait_for(gps_ok, timeout=max(10, deadline - time.time())):
            raise RuntimeError("no 3D fix / no position stream -- check SR0_*")

    def set_mode(self, name, timeout=15):
        mode_id = self.mav.mode_mapping()[name]
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._send(lambda: self.mav.set_mode_apm(mode_id))
            if self._wait_for(lambda: self.mav.flightmode == name, timeout=2):
                return True
        raise RuntimeError(
            f"mode {name} not reached (still {self.mav.flightmode})")

    def arm(self, timeout=90):
        self.rc(3, 1000)
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._send(lambda: self.mav.mav.command_long_send(
                self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0))
            if self._wait_for(self.mav.motors_armed, timeout=3):
                return True
        raise RuntimeError(
            f"failed to arm; recent statustexts: {self.statustexts()[-5:]}")

    def takeoff(self, alt_m, timeout=120):
        """GUIDED takeoff, then neutral throttle and hand back to the caller."""
        self.set_mode("GUIDED")
        self.arm()
        self._send(lambda: self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt_m))
        reached = self._wait_for(
            lambda: (self.altitude() or 0.0) >= alt_m * 0.95, timeout=timeout)
        self.hold()
        if not reached:
            raise RuntimeError(
                f"takeoff to {alt_m} m timed out at {self.altitude()}")
        return True

    def descend_to(self, alt_m, timeout=180):
        """Descend in the current mode until below alt_m, then hold altitude.

        Returning to neutral throttle is deliberate and load-bearing.  The
        script's descent latch only releases on a climb faster than -0.1 m/s
        (script line 412), so the limits stay applied while the aircraft holds
        station -- and the caller's assertions stop racing the aircraft toward
        the ground.  At --speedup 10 the fall from 4 m to 1 m at the clamped
        0.5 m/s is 0.6 s of wall clock, less than two param round trips.
        """
        self.rc(3, RC_THROTTLE_DOWN)
        reached = self._wait_for(
            lambda: self.altitude() is not None and self.altitude() <= alt_m,
            timeout=timeout)
        self.hold()
        if not reached:
            raise RuntimeError(
                f"descent to {alt_m} m timed out at {self.altitude()}")
        return True

    def climb(self):
        """Full-up throttle.  Releases the script's descent latch."""
        self.rc(3, RC_THROTTLE_UP)
```

- [ ] **Step 2: Write the fixture**

Create `~/Documents/IFT/rangefinder/sitl/conftest.py`:

```python
import pytest

from sitl_harness import SitlCopter


@pytest.fixture
def copter(request):
    """One fresh SITL per test, always --wipe, so no parameter state leaks.

    Mark a test with @pytest.mark.speedup(2) if it needs more wall clock per
    simulated second -- the LAND band window is only ~0.8 s at speedup 10.
    """
    marker = request.node.get_closest_marker("speedup")
    c = SitlCopter(speedup=marker.args[0] if marker else 10)
    c.start()
    c.wait_ready()
    yield c
    c.stop()
```

and `~/Documents/IFT/rangefinder/sitl/pytest.ini` to register the marker:

```ini
[pytest]
markers =
    speedup(factor): run this test's SITL at a non-default --speedup
```

- [ ] **Step 3: Write the failing test — LOITER upper band**

Create `~/Documents/IFT/rangefinder/sitl/test_loiter_bands.py`:

```python
"""LOITER branch of the RFL limiter. apply_limit() sets BOTH PILOT_SPEED_DN
and LOIT_SPEED in this mode.

Statustexts are asserted in full. SitlCopter reassembles MAVLink's 50-byte
chunks, so "limiting to no more than 0.50 m/s" (53 chars with the script's
prefix) matches; without reassembly it never would, and the absence assertions
below would pass vacuously.
"""


def test_script_starts_and_binds_rangefinder(copter):
    assert copter.wait_statustext("RangefinderLimiter: started", timeout=30)
    for warning in ("no downward rangefinder",
                    "max range below RFL_ALT_HI",
                    "looks like a leftover override",
                    "inactive, reboot to clear"):
        assert not copter.saw_statustext(warning), \
            f"unexpected warning: {warning}"


def test_loiter_upper_band_clamps_to_half_metre(copter):
    copter.takeoff(15)
    copter.set_mode("LOITER")
    copter.clear_statustexts()

    # baseline: PILOT_SPEED_DN 0 means "use PILOT_SPEED_UP" = 250 cm/s
    assert copter.param_get("PILOT_SPEED_DN") == 0.0

    copter.descend_to(4.0)  # returns at neutral throttle, limits latched

    assert copter.wait_statustext("limiting to no more than 0.50 m/s")
    assert copter.param_get("PILOT_SPEED_DN") == 50.0
    assert copter.param_get("LOIT_SPEED") == 50.0
    # LOITER must not touch the landing levers
    assert copter.param_get("LAND_SPEED") == 100.0
    assert copter.param_get("LAND_SPEED_HIGH") == 0.0
```

- [ ] **Step 4: Run it and watch it fail for the right reason**

```bash
cd ~/Documents/IFT/rangefinder/sitl
python3 -m pytest test_loiter_bands.py -v -x
```

Expected on first run: failures come from harness bugs (connection, arming, mode changes), **not** from the assertions. Iterate on `sitl_harness.py` until `test_script_starts_and_binds_rangefinder` passes. If `test_loiter_upper_band_clamps_to_half_metre` fails on its assertions once the harness works, that is a real finding about the script — record it, do not patch the Lua.

- [ ] **Step 5: Run until both pass**

```bash
python3 -m pytest test_loiter_bands.py -v
```

Expected: 2 passed.

- [ ] **Step 6: Commit**

```bash
cd ~/Documents/IFT/rangefinder
git add sitl/sitl_harness.py sitl/conftest.py sitl/pytest.ini sitl/test_loiter_bands.py
git commit -m "sitl: harness + LOITER upper band test"
```

---

### Task 3: LOITER lower band, latch release, live-tuning floor

**Files:**
- Modify: `~/Documents/IFT/rangefinder/sitl/test_loiter_bands.py`

**Interfaces:**
- Consumes: `SitlCopter` from Task 2 — `takeoff`, `set_mode`, `descend_to`, `climb`, `hold`, `param_get`, `param_set`, `wait_statustext`, `clear_statustexts`.
- Produces: nothing new.

- [ ] **Step 1: Write the failing tests**

Append to `test_loiter_bands.py`:

```python
def test_loiter_lower_band_clamps_to_quarter_metre(copter):
    copter.takeoff(15)
    copter.set_mode("LOITER")
    copter.descend_to(4.0)
    copter.clear_statustexts()

    copter.descend_to(0.8)

    assert copter.wait_statustext("limiting to no more than 0.25 m/s")
    assert copter.param_get("PILOT_SPEED_DN") == 25.0
    assert copter.param_get("LOIT_SPEED") == 25.0


def test_climbing_releases_the_latch(copter):
    copter.takeoff(15)
    copter.set_mode("LOITER")
    copter.descend_to(4.0)
    assert copter.wait_statustext("limiting to no more than")
    copter.clear_statustexts()

    # climb_release_rate is -0.1 m/s in the script
    copter.climb()

    assert copter.wait_statustext("limits released")
    assert copter.param_get("PILOT_SPEED_DN") == 0.0
    assert copter.param_get("LOIT_SPEED") == 1250.0
    copter.hold()


def test_speed_floor_is_enforced_on_live_tuning(copter):
    """band_speed_cm_s (script line 424) re-applies the 20 cm/s AC_Loiter floor
    to the live RFL_ parameters, so a customer cannot tune below it."""
    copter.takeoff(15)
    copter.set_mode("LOITER")
    copter.descend_to(0.8)
    assert copter.wait_statustext("limiting to no more than 0.25 m/s")
    copter.clear_statustexts()

    copter.param_set("RFL_SPD_LO", 0.10)

    assert copter.wait_statustext("limiting to no more than 0.20 m/s")
    assert copter.param_get("PILOT_SPEED_DN") == 20.0
    assert copter.param_get("LOIT_SPEED") == 20.0
```

- [ ] **Step 2: Run to verify they fail**

```bash
cd ~/Documents/IFT/rangefinder/sitl
python3 -m pytest test_loiter_bands.py -v
```

Expected: the three new tests fail; the two from Task 2 still pass.

- [ ] **Step 3: Fix harness issues only**

The Lua script already implements all three behaviours. Any failure is either a harness timing problem (raise the timeout) or a genuine script finding. Do not edit the Lua.

There is no `pump()` any more — the rx thread drains the link continuously, so "wait longer" means a longer `timeout` argument, never an extra pump call.

- [ ] **Step 4: Run to verify they pass**

```bash
python3 -m pytest test_loiter_bands.py -v
```

Expected: 5 passed.

- [ ] **Step 5: Commit**

```bash
cd ~/Documents/IFT/rangefinder
git add sitl/test_loiter_bands.py
git commit -m "sitl: LOITER lower band, latch release, live-tuning floor"
```

---

### Task 4: LAND branch

**Files:**
- Test: `~/Documents/IFT/rangefinder/sitl/test_land_bands.py`

**Interfaces:**
- Consumes: `SitlCopter` from Task 2.
- Produces: nothing new.

- [ ] **Step 1: Write the failing tests**

Create `~/Documents/IFT/rangefinder/sitl/test_land_bands.py`:

```python
"""LAND branch. apply_limit() writes TWO parameters here: LAND_SPEED and
LAND_SPEED_HIGH (script lines 307-310). It does not touch the loiter levers --
LAND's horizontal control is position hold capped by WPNAV_ACCEL (mode.cpp:719),
not LOIT_SPEED.

LAND_SPEED_HIGH defaults to 0 = "use WPNAV_SPEED_DN" (Parameters.cpp:207,
AC_WPNav.cpp:12), so the script's effective baseline for it is 150 cm/s and
restore_originals writes the literal 0 back, not 150.

rangefinder.parm sets LAND_SPEED 100 so all three rates are distinct:
  above 5 m  -> 1.00 m/s (baseline)
  5 m .. 1 m -> 0.50 m/s (RFL_SPD_HI)
  below 1 m  -> 0.25 m/s (RFL_SPD_LO)
"""


def test_land_upper_band_clamps_both_landing_stages(copter):
    copter.takeoff(15)
    assert copter.param_get("LAND_SPEED") == 100.0
    assert copter.param_get("LAND_SPEED_HIGH") == 0.0
    copter.clear_statustexts()
    copter.set_mode("LAND")

    assert copter.wait_statustext("limiting to no more than 0.50 m/s",
                                  timeout=60)
    assert copter.param_get("LAND_SPEED") == 50.0
    assert copter.param_get("LAND_SPEED_HIGH") == 50.0
    # LAND must not touch the loiter levers
    assert copter.param_get("LOIT_SPEED") == 1250.0
    assert copter.param_get("PILOT_SPEED_DN") == 0.0


def test_land_lower_band_clamps_to_quarter_metre(copter):
    copter.takeoff(15)
    copter.set_mode("LAND")
    assert copter.wait_statustext("limiting to no more than 0.50 m/s",
                                  timeout=60)
    copter.clear_statustexts()

    assert copter.wait_statustext("limiting to no more than 0.25 m/s",
                                  timeout=60)
    assert copter.param_get("LAND_SPEED") == 25.0
    assert copter.param_get("LAND_SPEED_HIGH") == 25.0


def test_mode_change_out_of_land_restores_baseline(copter):
    # 20 m, not 15: LAND descends at WPNAV_SPEED_DN 1.5 m/s until LAND_ALT_LOW
    # (10 m, Parameters.cpp:896) and 1.0 m/s below it, so the band_mid window
    # between 5 m and 1 m is only ~8 s of sim time = 0.8 s at --speedup 10.
    # Starting higher does not widen that window, but it does give the mode
    # change room ahead of it.
    copter.takeoff(20)
    copter.set_mode("LAND")
    assert copter.wait_statustext("limiting to no more than", timeout=60)
    copter.clear_statustexts()

    copter.set_mode("LOITER")
    copter.hold()

    # apply_limit sees the mode change while still latched, so it restores the
    # LAND levers (emitting "limits released") before applying the LOITER ones.
    assert copter.wait_statustext("limits released")
    assert copter.param_get("LAND_SPEED") == 100.0
    assert copter.param_get("LAND_SPEED_HIGH") == 0.0
```

- [ ] **Step 2: Run to verify they fail**

```bash
cd ~/Documents/IFT/rangefinder/sitl
python3 -m pytest test_land_bands.py -v
```

- [ ] **Step 3: Fix harness issues only**

`test_mode_change_out_of_land_restores_baseline` is the one genuinely tight test in the suite, and it cannot be fixed by returning to neutral throttle the way the LOITER tests are — LAND ignores the throttle stick and keeps descending. The band_mid window is ~8 s of sim time, 0.8 s of wall clock at `--speedup 10`, and `wait_statustext` plus `set_mode` must both land inside it.

If it proves flaky, the fix is to lower the speedup **for this test only** via the fixture (`SitlCopter(speedup=2)`), not to loosen the assertion. Do not widen the window by raising `RFL_ALT_HI` — that would stop testing the shipped configuration.

- [ ] **Step 4: Run to verify they pass**

```bash
python3 -m pytest test_land_bands.py -v
```

Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
cd ~/Documents/IFT/rangefinder
git add sitl/test_land_bands.py
git commit -m "sitl: LAND branch tests"
```

---

### Task 5: Sensor faults and out-of-range paths

**Files:**
- Test: `~/Documents/IFT/rangefinder/sitl/test_sensor_faults.py`

**Interfaces:**
- Consumes: `SitlCopter` from Task 2 — especially `param_set`.
- Produces: nothing new.

This is the task SIH could not do at all. `AP_RangeFinder.cpp:236` makes runtime fault injection possible with no reboot.

- [ ] **Step 1: Write the failing tests**

Create `~/Documents/IFT/rangefinder/sitl/test_sensor_faults.py`:

```python
"""Off-nominal sensor paths.

RNGFND1_TYPE 0 at runtime -> Status::NotConnected (AP_RangeFinder.cpp:236),
with the driver still allocated so restoring the type resumes it. That reaches
the script's sensor_fault path: 5 samples at 100 ms to condemn, 3 to recover.

MIN_CM / MAX_CM are read live by update_status()
(AP_RangeFinder_Backend.cpp:56-66) and the SIM backend does not pre-clamp
distance_m (AP_RangeFinder_SITL.cpp:43), so shrinking them mid-flight really
does force OutOfRangeLow / OutOfRangeHigh.

Every test here descends to 4 m and then holds. descend_to() leaves the
aircraft at neutral throttle; the descent latch stays engaged because it only
releases on a climb faster than -0.1 m/s. Holding is what makes these tests
deterministic -- once limits release mid-test, PILOT_SPEED_DN returns to 250
cm/s, and a still-descending aircraft would reach the ground in 1.6 s of sim
time (0.16 s of wall clock at speedup 10), well inside the assertions.
"""


def descend_into_upper_band(copter):
    copter.takeoff(15)
    copter.set_mode("LOITER")
    copter.descend_to(4.0)
    assert copter.wait_statustext("limiting to no more than 0.50 m/s")
    copter.clear_statustexts()


def test_sensor_fault_releases_limits_and_recovers(copter):
    descend_into_upper_band(copter)

    copter.param_set("RNGFND1_TYPE", 0)

    assert copter.wait_statustext(
        "rangefinder unhealthy, normal flight restored")
    assert copter.param_get("PILOT_SPEED_DN") == 0.0
    assert copter.param_get("LOIT_SPEED") == 1250.0
    copter.clear_statustexts()

    copter.param_set("RNGFND1_TYPE", 100)

    assert copter.wait_statustext("rangefinder healthy")


def test_out_of_range_high_drops_to_no_band(copter):
    descend_into_upper_band(copter)

    # 3 m ceiling while holding at ~4 m -> OutOfRangeHigh -> band_none
    copter.param_set("RNGFND1_MAX_CM", 300)

    assert copter.wait_statustext("limits released")
    assert copter.param_get("PILOT_SPEED_DN") == 0.0
    assert copter.param_get("LOIT_SPEED") == 1250.0


def test_out_of_range_low_forces_tightest_clamp(copter):
    descend_into_upper_band(copter)

    # 5 m floor while holding at ~4 m -> OutOfRangeLow -> band_low.
    # select_band() short-circuits to band_low on sensor_below_min, bypassing
    # the hysteresis in next_band().
    copter.param_set("RNGFND1_MIN_CM", 500)

    assert copter.wait_statustext("limiting to no more than 0.25 m/s")
    assert copter.param_get("PILOT_SPEED_DN") == 25.0
    assert copter.param_get("LOIT_SPEED") == 25.0


def test_rfl_enable_zero_releases_everything(copter):
    descend_into_upper_band(copter)

    copter.param_set("RFL_ENABLE", 0)

    assert copter.wait_statustext("limits released")
    assert copter.param_get("PILOT_SPEED_DN") == 0.0
    assert copter.param_get("LOIT_SPEED") == 1250.0
```

- [ ] **Step 2: Run to verify they fail**

```bash
cd ~/Documents/IFT/rangefinder/sitl
python3 -m pytest test_sensor_faults.py -v
```

- [ ] **Step 3: Fix harness issues only**

Do **not** hold the throttle down through these assertions. The latch is only *entered* while descending faster than `RFL_DSC_TH` 0.2 m/s, but it is already entered by the time `descend_into_upper_band` returns, and it persists at neutral throttle (script line 412). Continuing to descend would only race the assertions against the ground.

The debounce timings are the thing to watch: 5 samples at 100 ms to condemn the sensor, 3 to recover (script lines 43-44). At `--speedup 10` those are 50 ms and 30 ms of wall clock, so they are never the bottleneck — but they do mean `param_set`'s read-back must not be the first thing to observe the transition. It is not: the rx thread records statustexts continuously.

- [ ] **Step 4: Run to verify they pass**

```bash
python3 -m pytest test_sensor_faults.py -v
```

Expected: 4 passed.

- [ ] **Step 5: Run the whole suite**

```bash
python3 -m pytest -v
```

Expected: 12 passed (5 LOITER + 3 LAND + 4 faults).

- [ ] **Step 6: Commit**

```bash
cd ~/Documents/IFT/rangefinder
git add sitl/test_sensor_faults.py
git commit -m "sitl: sensor fault + out-of-range tests"
```

---

### Task 6: README

**Files:**
- Create: `~/Documents/IFT/rangefinder/sitl/README.md`

**Interfaces:**
- Consumes: everything above.
- Produces: nothing.

- [ ] **Step 1: Write the README**

Create `~/Documents/IFT/rangefinder/sitl/README.md`:

```markdown
# SITL test harness for limit-descent-speed.lua

Exercises the RFL rangefinder descent limiter in desktop ArduCopter SITL.

## Setup (once)

    cd /home/ak/git/IFT/IFT-ardupilot
    ./waf configure --board sitl && ./waf copter

This reconfigures waf away from the IF800 board. To go back:

    ./waf configure --board IF800-CubeOrangePlus \
      --extra-hwdef=libraries/AP_HAL_ChibiOS/hwdef/IF800-CubeOrangePlus/sih-extra-hwdef.dat

## Run

    cd ~/Documents/IFT/rangefinder/sitl
    python3 -m pytest -v

Each test launches its own SITL with `--wipe`, so tests never inherit
parameter state from each other.

## Coverage

| File | Tests | Covers |
|---|---|---|
| `test_loiter_bands.py` | 5 | script startup, LOITER 5 m and 1 m bands, latch release on climb, 20 cm/s floor on live tuning |
| `test_land_bands.py` | 3 | LAND 5 m and 1 m bands on both `LAND_SPEED` and `LAND_SPEED_HIGH`, loiter levers left untouched, mode-change restore |
| `test_sensor_faults.py` | 4 | sensor fault + recovery, OutOfRangeHigh, OutOfRangeLow, `RFL_ENABLE 0` |

## Design notes

- `RNGFND1_TYPE 100` (`Type::SIM`) reads the sim's own `height_agl`. The
  documented SITL rangefinder recipe (`TYPE 1` + `SIM_SONAR_SCALE` +
  `RNGFND1_PIN 0`) is the *analog* path in `AP_HAL_SITL/sitl_rangefinder.cpp`;
  either works on desktop SITL, but only type 100 also works on
  sim-on-hardware, so the same params carry over.
- `LAND_SPEED` is 100, not the stock 50. At 50 it is exactly `RFL_SPD_HI * 100`,
  so the 5 m band in LAND would emit a statustext with no rate change.
- `SURFTRAK_MODE 0` keeps Copter's own surface tracking off the rangefinder so
  a dropout changes only one thing.
- Params come from the binary's `--defaults` flag, which applies them before
  `RangeFinder::init()`. Setting `RNGFND1_TYPE` over MAVLink at startup would
  need a reboot to instantiate a backend.
- `SR0_*` stream rates are set explicitly. They default to 0
  (`ArduCopter/GCS_Mavlink.cpp:401-491`) and nothing here runs MAVProxy to ask
  for them, so without these no position or rangefinder messages ever arrive.
- Statustexts longer than 50 bytes are split into `id`/`chunk_seq` chunks by
  `GCS_Common.cpp:2251-2283`, and pymavlink does not reassemble them.
  `SitlCopter` does, so tests can assert on whole strings. Assert on raw
  `msg.text` and both positive *and negative* assertions quietly stop meaning
  anything.
- One rx thread owns the socket. A typed `recv_match` discards non-matching
  messages, so anything that polls for a specific type in a loop will eat the
  one-shot statustexts under test.
- `descend_to()` returns to neutral throttle. The descent latch only releases
  on a climb faster than -0.1 m/s, so limits persist while holding — which is
  what keeps the assertions from racing the aircraft toward the ground.

## Not covered

- **Band hysteresis** (`band_hysteresis` 1.15, script lines 229-257). It is
  only reachable on a rising rangefinder distance, and any climb fast enough to
  raise the distance also releases the descent latch and drops the limits, so
  the hysteresis and the non-hysteresis path produce identical parameter
  traces. Not observable through this harness's levers. Testing it needs
  `next_band()` extracted into a pure module and driven directly.
- Ground contact below `RNGFND1_MIN_CM` (20 cm) during a real touchdown —
  the tests stop at 0.8 m.
- `parameter write failed` path in `apply_limit` — no way to make
  `Parameter:set()` fail on demand.
- `report_unusable`'s 10 s repeating warning — reaching it needs a boot with a
  poisoned baseline, i.e. a second param file and a second fixture.
```

- [ ] **Step 2: Verify the README's commands actually work**

```bash
cd ~/Documents/IFT/rangefinder/sitl
python3 -m pytest -v
```

Expected: 12 passed, matching the coverage table.

- [ ] **Step 3: Commit**

```bash
cd ~/Documents/IFT/rangefinder
git add sitl/README.md
git commit -m "sitl: README"
```

---

## Self-Review Notes

- **Spec coverage:** every statustext the script can emit has an asserting test except `parameter write failed` (unreachable — noted in the README) and the boot-time `looks like a leftover override` / `inactive, reboot to clear` warnings (asserted *absent* in Task 2, which is the correct polarity for a healthy config). Band hysteresis is the one *behaviour* with no test, and the README says why rather than pretending otherwise.
- **Type consistency:** `SitlCopter` method names used in Tasks 3-5 (`takeoff`, `set_mode`, `descend_to`, `climb`, `hold`, `param_get`, `param_set`, `wait_statustext`, `saw_statustext`, `clear_statustexts`) all appear in the Task 2 class definition. `pump()` is gone entirely, and raw `rc(3, …)` calls no longer appear in any test file — throttle is driven through `climb()` / `hold()` / `descend_to()` so the latch semantics stay in one place.
- **Known risk:** the harness is the fragile part, not the assertions. Tasks 2-5 each say explicitly that assertion failures are findings about the Lua script and must not be fixed by editing it.

### Corrections applied to the first draft

Recorded because each was a silent failure, not a loud one — the kind that produces a green suite that tests nothing:

| Was | Is |
|---|---|
| Asserted `limiting to 0.50 m/s` | Script emits `limiting to no more than 0.50 m/s` (line 325). No assertion could ever have matched. |
| Assumed `msg.text` is the whole message | MAVLink chunks at 50 bytes and pymavlink does not reassemble. `SitlCopter` now does. The *absence* assertions in Task 2 were passing vacuously. |
| No stream rates requested | `SR0_*` all default to 0, so `GLOBAL_POSITION_INT` and `RANGEFINDER` never arrived and every altitude helper would have timed out. Set in `rangefinder.parm`. |
| Typed `recv_match` scattered through the harness | Each one silently discarded the statustexts under test; `descend_to` polled `altitude()` for the whole descent, exactly when `limiting to…` fires. Single rx thread now. |
| "LAND sets only `LAND_SPEED`" | It also sets `LAND_SPEED_HIGH` (lines 307-310). Task 4 now asserts both, in both directions. |
| `descend_to` left the throttle down | Assertions then raced a falling aircraft — 4 m to 1 m is 0.6 s of wall clock at speedup 10. It now returns to neutral; the latch is sticky, so limits persist. |
| `wait_ready` swallowed a Lua load failure for 120 s | Now raises with the symlink/`SCR_ENABLE` hint, so a dead script does not masquerade as an arming timeout. |
| RC overrides described as failsafe avoidance | SITL seeds its own RC input (`SITL_State.cpp:563-565`); the overrides only exist to command throttle. |
| Five commit steps | `~/Documents/IFT/rangefinder` was not a git repo. Task 1 Step 2 now initialises one. |
