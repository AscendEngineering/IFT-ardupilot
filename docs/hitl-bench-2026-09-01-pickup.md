# HITL bench — pickup notes (2026-09-01)

Where the external-physics (Gazebo → JSON/PPP/USB) HITL bench stands after the
2026-09-01 session, and exactly what to do next. Companion to
`docs/hitl-instructions.md` (§2 is the canonical setup doc; §2.4 now has a
boot-loop triage list, §2.6.1 covers free-run).

## State right now

- **Branch** `hitl-network`. Committed this session:
  - `4643928338` — SITL: JSON free-run (`no_lockstep`) mode for hardware HITL
    (upstream 759e03f1aa cherry-pick + reviewer fixes; includes the JSON
    boolean-parser fix, so `no_time_sync` from the SDF now really applies).
  - `370a9b1e2b` — docs: free-run mode + the real ~300 Hz lockstep ceiling
    (firmware polling chain, not USB).
- **Uncommitted** (intentionally, commit when asked):
  - `libraries/SITL/SIM_Aircraft.cpp/.h` — **SIM rate statustext fix** (see below).
    ARM-syntax-checked against the board's real compile command; needs rebuild+flash.
  - `docs/hitl-instructions.md` — statustext semantics + boot-loop triage additions.
  - `defaults.parm` / `sih-extra-hwdef.dat` — user's 08-06 rangefinder-sim edits.
- **Board** (Cube, `…3F0037000A51323031393637`): flashed with githash `370a9b1e`
  (JSON+PPP overlay). Params verified in FRAM: `NET_ENABLE 1`, `SERIAL6_PROTOCOL 48`,
  `SCHED_LOOP_RATE 400`, `SIM_RATE_HZ 400`, `AHRS_EKF_TYPE 10`, `GPS_TYPE 100`,
  `SCR_ENABLE 1`, `RNGFND1_TYPE 1` (analog — NOT the embedded sim type 100).
- **Bench verified running**: PPP up, ~190 lockstep exchanges/s, GPS 3D fix (15 sats)
  at CMAC, EKF flags 895, level attitude, Loiter, `PreArm: Main loop slow (189Hz < 400Hz)`
  (expected: lockstep at 5 ms world step caps the loop at ~200 Hz).

## The boot loop — what it was (all host-side, firmware was fine)

Symptom: USB re-enumeration every ~6 s; `WDG: T-1 … IE34816` (= `0x8800`,
watchdog_reset + main_loop_stuck); console `No JSON sensor message received,
resending servos`. In lockstep the main loop blocks in `recv_fdm()` until physics
answers; the IWDG (armed right after `setup()`) fires ~2 s after `ArduPilot Ready`.

Three causes found in sequence:
1. **No `pppd`.** Start it with `holdoff 1` — the default 30 s holdoff almost never
   lines up with the ~5 s the board stays enumerated while looping.
2. **Gazebo launched on `model.sdf` instead of a world.** `Imu`/`Sensors` systems are
   declared in the *world* SDF only; without them `CreateStateJSON()` bails on
   `!imuMsgValid` and the plugin replies with an **empty datagram** — the board's
   `recv()` returns 0 and it keeps waiting. gz log shows only `Duplicate input frame`.
3. **Sticky plugin lockstep state.** `arduPilotOnline` never clears in `lock_step`
   mode and a rebooted board's `frame_count 0` equals the remembered one → every
   packet is a "Duplicate". **Restart Gazebo after fixing anything upstream of it.**

## Known-good bring-up order

```bash
# 1. Gazebo (world, not model.sdf). GUI+gimbal camera only reached ~70 physics
#    steps/s (world RTF ≈ 0.4) — try headless first:
cd ~/git/ardupilot_gazebo/worlds && gz sim -s -r iris_runway.sdf
#    (iris_runway matches the board's stored SIM_OPOS; if800_survey.sdf is ~40 m off
#     until SIM_OPOS_* is updated. Both worlds: max_step_size 0.005 = 200 Hz.)

# 2. PPP bridge (own terminal, stays foreground):
sudo pppd /dev/serial/by-id/usb-CubePilot_CubeOrange+_3F0037000A51323031393637-if02 \
     115200 192.168.144.1:192.168.144.14 noauth local nodetach persist maxfail 0 holdoff 1

# 3. Power the board (or let it catch on its next resend — it retries every 1 s).
# Success: pppd prints both IP addresses and stays up; ping 192.168.144.14 answers;
# USB stops re-enumerating; gz log: "Connected to ArduPilot controller".

# QGC: AutoConnect→Pixhawk OFF, manual serial link on ttyACM0 (if00) only —
# both CDC ports share VID/PID and QGC will otherwise grab the PPP port.
```

## Next steps, in order

1. **Rebuild + flash** to pick up the SIM-rate fix (waf already configured for
   IF800 + json overlay): `./waf copter` → `build/IF800-CubeOrangePlus/bin/arducopter.apj`
   (flash manually — never `--upload`). Keep pppd+Gazebo up when the board comes back.
   - The bug: `SIM_JSON.h:75` declares a `frame_counter` that shadows
     `Aircraft::frame_counter`, so the report read 0 forever (`SIM rate 0/400Hz RTF 0.00`
     while the link demonstrably ran ~190 Hz). Fixed to count `time_advance()` calls
     and derive RTF from `time_now_us`.
2. **Measure Gazebo's real step rate headless**: `gz topic -e -t /world/iris_runway/stats`
   → want ~200 iterations/s, RTF ≈ 1.0. If the GUI/camera is the drag, keep the bench
   headless or drop the camera sensor rate.
3. **Lockstep at 200**: set `SCHED_LOOP_RATE 200` + `SIM_RATE_HZ 200` (world step
   0.005), reboot → expect `SIM rate 200/200Hz RTF ~1.00` (the verified 07-29 config).
4. **Then free-run at 400** (§2.6.1): `<lock_step>0</lock_step>` in
   `models/iris_with_gimbal/model.sdf`, restore `SCHED_LOOP_RATE`/`SIM_RATE_HZ 400`,
   world `max_step_size 0.0025`. Pass = `SIM rate 400/400Hz RTF 1.00`, console prints
   `Lockstep mode changed: no_lockstep=1` once, no `SIM: no physics data` CRITICALs.
   This exercises the new (still hardware-unverified) free-run path: zero-timeout
   polling, drain-to-newest, 200 ms stale warning.
5. Housekeeping: `ARMING_CRSDP_IGN 1` (acks `PreArm: CrashDump data detected` left by
   the pre-watchdog hardfaults); decide `RNGFND1_TYPE` (FRAM has 1 = analog, embedded
   default wants 100 = sim); commit the SIM-rate fix + docs when satisfied.

## Diagnostic cookbook (all rootless, proven today)

| What | How |
|---|---|
| Reset cadence | `journalctl -k` → `USB disconnect` every ~6.1 s |
| Watchdog cause | pymavlink on the if00 port; **send a GCS heartbeat** or statustexts stay queued; `WDG: … IE<n>` — 0x8800 = watchdog+main_loop_stuck |
| Firmware printf console | rides the MAVLink CDC port as raw bytes → collect `BAD_DATA`, regex printable runs |
| Is state JSON flowing | `/sys/class/net/ppp0/statistics/{rx,tx}_bytes` — state ≈ 1 kB, servo pkt = 68 B |
| Plugin's view | `~/.gz/sim/log/<ts>/server_console.log` (`Connected`, `Duplicate`, `Missed N`) |
| Socket being read? | `ss -lunp | grep 9002` — Recv-Q > 0 means nobody's consuming |
| Sim actually stepping | `gz topic -e -t /world/<name>/stats` (iterations, RTF) |
| Board params mid-bootloop | pymavlink param_request_read inside the ~4 s MAVLink window works fine |

Scratch scripts from the session (rawcap/paramcap/health) live in the session
scratchpad; they're throwaway but trivial to recreate from the table above.
