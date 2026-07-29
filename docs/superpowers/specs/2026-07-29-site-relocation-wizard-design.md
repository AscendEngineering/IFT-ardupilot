# Flight-Site Relocation Wizard — Design

**Date:** 2026-07-29
**Status:** Approved (this session)
**Audience for the tool:** non-technical test operators using the HIL Sim Bench

## Purpose

Let a test operator move the simulated flight site to any real-world location —
for desktop SITL, SIH, or External-Physics SIH (Gazebo HITL) — without editing
SDF files, computing tile math, or typing parameter values by hand. One
interactive terminal wizard does everything; the operator supplies coordinates
pasted from Google Maps.

## Deliverables and layout

```
HIL-Sim-Bench/                       (main repo — becomes bench tooling home)
  tools/
    site_wizard.py                   interactive wizard (entry point)
    fetch_ortho.py                   imagery downloader (moved from IFT-ardupilot)
    sites/                           one folder per site created by the wizard
      <site-name>/
        site.json                    coords, elevation, texture size, mode, spawn
        ortho.jpg                    downloaded imagery (gitignored; re-downloadable)
        launch-sitl.sh               generated launcher (SITL sites)
        launch-gazebo.sh             generated launcher (External-Physics sites)
  .gitignore                         adds tools/sites/**/ortho.jpg

HIL-Sim-Bench wiki:
  new page "Changing the Flight Site" — operator walkthrough + troubleshooting
  link lines added to: HIL-Software-Overview, External-Physics-SIH,
  SITL-Instructions (also fixes SYSID_MY_GCS → SYSID_MYGCS on SIH-Instructions)

IFT-ardupilot (branch hitl-network):
  docs/hitl-instructions.md §2.6.1 — point at HIL-Sim-Bench/tools as the new home
  docs/hitl-tools/fetch_ortho.py — removed (moved to HIL-Sim-Bench)
```

Dependencies: python3 + requests + PIL + pymavlink — all already on the bench
laptop. No new installs. Linux/bench-laptop only.

## Wizard flow

Main menu:
1. **Set up a new site**
2. **Switch to a saved site** — re-applies a `sites/<name>/` folder with no
   re-download; instant switching between the handful of sites testers use.
   Ends with a quick radio prompt (keep current / Herelink / Taisync) so a
   site switch and a radio switch are one pass
3. **Show current site** — reads `site.json` state + board `SIM_OPOS` if reachable
4. **Radio setup (Herelink / Taisync)** — standalone param step, also offered
   during new-site setup

Shared steps for a new site: pick mode (SITL / SIH / External Physics, default
External Physics) → paste coordinates (accepts Google-Maps formats: `lat, lng`
decimal; wizard prints how to copy them) → ground elevation auto-looked-up
(Open-Meteo elevation API), shown for confirm/override → short site name
(default derived from coords).

### Per mode

**SITL** — generates `launch-sitl.sh` wrapping
`Tools/autotest/sim_vehicle.py -v ArduCopter --no-rebuild --console --map
--custom-location=lat,lng,alt,hdg`, with a guard: if the SITL binary is missing,
print "ask a developer to build SITL once" instead of letting waf reconfigure
away the IF800 board build. Offers to launch now.

**SIH** — no Gazebo. Sets `SIM_OPOS_LAT/LNG/ALT/HDG` over USB (below), offers
reboot.

**External Physics** — in order:
1. Area size: ~560 m (default) or ~1100 m. Zoom auto-selected: probe Esri zoom
   20 (corners + center of block), fall back to 19; texture capped at 5120 px
   per side (drop zoom if needed).
2. Spawn point: center (default) or paste a second Google-Maps pin inside the
   area — converted to east/north meters internally; operators never see meters.
   Validated ≥30 m inside the texture edge.
3. Download imagery with progress bar → `sites/<name>/ortho.jpg`; offer
   `xdg-open` so the operator can eyeball the site picture.
4. Apply Gazebo files (backup `.bak` first, edits idempotent):
   - copy jpg → `~/git/ardupilot_gazebo/models/survey_ground/materials/textures/survey_ortho.jpg`
   - `models/survey_ground/model.sdf`: both `<plane><size>` entries ← mosaic
     ground extent
   - `worlds/if800_survey.sdf`: `<spherical_coordinates>` ← mosaic center +
     elevation; iris `<pose>` ← spawn offset
5. Generate `launch-gazebo.sh`: refuses to start if `ppp0` is down (prints how
   to start pppd) or another `gz sim` is already running (both bind UDP 9002);
   then `gz sim -v4 -r if800_survey.sdf`. Offer to launch now.
6. Board params + reboot — sequenced last so physics is already up when the
   Cube comes back (lockstep boot-order rule enforced by sequence, not by
   operator memory).

Geo-registration contract (unchanged from docs §2.6.1): world
`spherical_coordinates` = mosaic center = board `SIM_OPOS_LAT/LNG`;
`SIM_OPOS_HDG 0`; Gazebo `max_step_size` stays `1/SCHED_LOOP_RATE` (wizard does
not touch the physics block).

## Board params over USB

- Find the MAVLink CDC port by ID (`/dev/serial/by-id/*if00*`) — never assume
  ttyACM0.
- IGC stays connected the whole time — it talks over the SERIAL1 radio, not
  USB. No "close IGC" instructions anywhere. (Busy-port handling kept as a
  safety net for benches where a GCS is on USB.)
- `param_set` each value, read back and verify (float32 relative tolerance;
  ~0.2 m coordinate quantization is inherent and acceptable).
- On any failure: print exact IGC steps (search "SIM_OPOS", the four values,
  Tools → Reboot Vehicle) — never a stack trace.
- Reboot via `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`, always confirmed first.

## Radio config step

Values from the SIH-Instructions wiki page (param name corrected to
`SYSID_MYGCS`):

| Radio    | SERIAL1_BAUD | SYSID_MYGCS |
|----------|--------------|-------------|
| Herelink | 57 (57600)   | 255         |
| Taisync  | 115 (115200) | 19          |

Same USB set/verify/fallback mechanics as site params.

## Wiki page content ("Changing the Flight Site")

Operator-voiced: one-paragraph description; how to copy coordinates from Google
Maps; the two commands (`cd HIL-Sim-Bench && python3 tools/site_wizard.py`);
one annotated example session; the radio table; troubleshooting table — drone
on grey void = params/world mismatch → rerun wizard; imagery download fails =
network/VPN (Home-page note); no heartbeat = USB cable / use IGC method.

## Error handling

- Top-level handler: plain-English message + what to try next; `--debug` flag
  for full tracebacks.
- Every automated step has a printed manual path.
- All file edits back up first and are idempotent; re-running on the same site
  is always safe.
- Ctrl+C at any prompt exits cleanly ("Cancelled — nothing was changed" when
  true).

## Testing

- Piped-input scripted runs of all three modes on the bench laptop.
- External-Physics end-to-end at CMAC coordinates (idempotence check: files
  equivalent to current state).
- Generated launch scripts executed for real (`launch-gazebo.sh` against the
  live bench).
- USB param path against the live Cube with user's go-ahead; radio step
  verified by reading back current values before writing anything.

## Out of scope

- Web/map-picker UI (possible later layer on top of the wizard).
- Remaining survey-mosaic TODO items (gimbal nadir, camera FOV match,
  CAMERA_FEEDBACK frame capture, ODM stitching).
- Windows/macOS support; multi-bench deployment.
