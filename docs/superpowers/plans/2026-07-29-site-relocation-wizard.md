# Flight-Site Relocation Wizard Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** An interactive terminal wizard in the HIL-Sim-Bench repo that lets non-technical test operators relocate the simulated flight site (SITL / SIH / External-Physics) to any real-world location, plus an operator wiki page.

**Architecture:** Small focused Python modules in `HIL-Sim-Bench/tools/` (`geo`, `fetch_ortho`, `gazebo_files`, `board`, `sites`, `ui`) orchestrated by `site_wizard.py`. Each site becomes a saved folder under `tools/sites/<name>/` containing `site.json`, downloaded imagery, and a generated launch script. Board params go over the USB MAVLink port while IGC stays connected via the SERIAL1 radio.

**Tech Stack:** Python 3.10 (system), requests, Pillow (PIL), pymavlink, pytest. Bash for generated launch scripts. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-07-29-site-relocation-wizard-design.md` (IFT-ardupilot repo)

**Revision:** 2026-07-29 — revised after code review; see "Review Resolution" at the end for what changed and why.

## Global Constraints

- Repos/paths: HIL-Sim-Bench cloned at `~/git/IFT/HIL-Sim-Bench` (does **not** exist yet — Task 1's clone is correct); wiki at `~/git/IFT/HIL-Sim-Bench.wiki` (**already exists**, clean, correct remote — Task 11 must not re-clone); ardupilot_gazebo at `~/git/ardupilot_gazebo`; IFT-ardupilot at `~/git/IFT/IFT-ardupilot` (branch `hitl-network`).
- Env-var overrides (used by tests, honored everywhere): `HIL_GAZEBO_DIR`, `HIL_ARDUPILOT_DIR`, `HIL_SITES_DIR`, `HIL_OFFLINE=1` (skip all network lookups), `HIL_NO_BOARD=1` (pretend no Cube is on USB — tests run on the bench laptop where the real Cube is plugged in, so board access must be explicitly disabled).
- The whole test suite runs with no network and no board. Nothing in `tools/` may reach the internet or `/dev/serial` during pytest.
- Param names exactly: `SIM_OPOS_LAT`, `SIM_OPOS_LNG`, `SIM_OPOS_ALT`, `SIM_OPOS_HDG`, `SERIAL1_BAUD`, `SYSID_MYGCS` (NOT `SYSID_MY_GCS`).
- Radio values: Herelink → `SERIAL1_BAUD 57`, `SYSID_MYGCS 255`. Taisync → `SERIAL1_BAUD 115`, `SYSID_MYGCS 19`.
- Geo-registration contract: Gazebo world `<spherical_coordinates>` = mosaic center = `SIM_OPOS_LAT/LNG`; `SIM_OPOS_HDG 0`. The wizard NEVER touches the world's physics block (`max_step_size` stays `1/SCHED_LOOP_RATE`) — enforced by an assertion in `gazebo_files`, not just by convention.
- **The site is the mosaic center, never the clicked pin.** Everything derived from position — `<spherical_coordinates>`, `SIM_OPOS_LAT/LNG`, and the ground elevation — is taken at the mosaic center. The pin can be up to a full tile (≈30 m at z20, ≈60 m at z19) away.
- Texture cap: 5120 px per side. Spawn point must be ≥30 m inside the texture edge.
- **Nothing under `~/git/ardupilot_gazebo` that the wizard edits is tracked by git** (verified: `worlds/if800_survey.sdf` and `models/survey_ground/` are untracked). There is no `git checkout` fallback, so `gazebo_files` keeps its own baseline.
- Elevation is rounded to whole meters once, at capture, so the world file and `SIM_OPOS_ALT` can never disagree.
- Operator-facing output: plain English, no stack traces (full traceback only with `--debug`).
- Never build/flash/upload firmware. Never start a second `gz sim` — the conflict is the simulator's UDP port 9002 (`<fdm_port_in>` in `models/iris_with_gimbal/model.sdf`), so both the wizard and the generated script test **that port**, not process names.
- Commit at the end of every task, in the repo that task touches.
- Run tests from the HIL-Sim-Bench repo root: `python3 -m pytest tools/tests -v`.

---

### Task 1: HIL-Sim-Bench scaffolding

**Files:**
- Create: `~/git/IFT/HIL-Sim-Bench/` (clone)
- Create: `tools/tests/conftest.py`
- Create: `.gitignore`

**Interfaces:**
- Produces: repo layout `tools/`, `tools/tests/`, `tools/sites/`; pytest importing modules from `tools/`.

- [ ] **Step 1: Clone the repo and create the layout**

```bash
git clone git@github.com:Inspired-Flight-Technologies/HIL-Sim-Bench.git ~/git/IFT/HIL-Sim-Bench
cd ~/git/IFT/HIL-Sim-Bench
mkdir -p tools/tests tools/sites
touch tools/sites/.gitkeep
```

- [ ] **Step 2: Verify pytest is available**

Run: `python3 -m pytest --version`
Expected: a version line. If missing: `pip install --user pytest`.

- [ ] **Step 3: Write `.gitignore`**

```gitignore
__pycache__/
*.pyc
tools/sites/**/ortho.jpg
tools/sites/**/*.log
tools/sites/active.txt
```

- [ ] **Step 4: Write `tools/tests/conftest.py`**

```python
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
sys.path.insert(0, str(_HERE.parents[1]))   # tools/       — modules under test
sys.path.insert(0, str(_HERE.parent))       # tools/tests/ — shared fixtures
```

- [ ] **Step 5: Sanity-run pytest (no tests yet)**

Run: `python3 -m pytest tools/tests -v`
Expected: "no tests ran" (exit code 5 is fine).

- [ ] **Step 6: Commit**

```bash
git add -A && git commit -m "tools: scaffolding for flight-site wizard"
```

---

### Task 2: `ui.py` — terminal prompts

**Files:**
- Create: `tools/ui.py`
- Test: `tools/tests/test_ui.py`

**Interfaces:**
- Produces: `say(text="")`, `ask(prompt, default=None) -> str`, `ask_yesno(prompt, default=True) -> bool`, `ask_choice(prompt, options: list[str], default_idx=0) -> int` (returns 0-based index). `ask` raises `KeyboardInterrupt` on EOF (so piped input running out aborts cleanly).

- [ ] **Step 1: Write the failing tests**

`tools/tests/test_ui.py`:

```python
import builtins
import pytest

import ui


def feed(monkeypatch, answers):
    it = iter(answers)
    monkeypatch.setattr(builtins, "input", lambda _="": next(it))


def test_ask_returns_default_on_empty(monkeypatch):
    feed(monkeypatch, [""])
    assert ui.ask("name", "cmac") == "cmac"


def test_ask_strips(monkeypatch):
    feed(monkeypatch, ["  hello  "])
    assert ui.ask("name") == "hello"


def test_ask_eof_raises_keyboardinterrupt(monkeypatch):
    def boom(_=""):
        raise EOFError
    monkeypatch.setattr(builtins, "input", boom)
    with pytest.raises(KeyboardInterrupt):
        ui.ask("name")


def test_ask_yesno(monkeypatch):
    feed(monkeypatch, ["", "n", "Yes"])
    assert ui.ask_yesno("ok?") is True          # default
    assert ui.ask_yesno("ok?") is False
    assert ui.ask_yesno("ok?", default=False) is True


def test_ask_choice_default_and_validation(monkeypatch):
    feed(monkeypatch, ["", "9", "2"])
    assert ui.ask_choice("pick", ["a", "b", "c"]) == 0      # empty -> default
    assert ui.ask_choice("pick", ["a", "b", "c"]) == 1      # 9 rejected, then 2
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest tools/tests/test_ui.py -v`
Expected: FAIL / errors with "No module named 'ui'".

- [ ] **Step 3: Write `tools/ui.py`**

```python
"""Tiny terminal prompt helpers for the site wizard (operator-facing)."""


def say(text=""):
    print(text)


def ask(prompt, default=None):
    suffix = f" [{default}]" if default not in (None, "") else ""
    try:
        value = input(f"  {prompt}{suffix} > ").strip()
    except EOFError:
        raise KeyboardInterrupt
    if value:
        return value
    return "" if default is None else str(default)


def ask_yesno(prompt, default=True):
    hint = "Y/n" if default else "y/N"
    value = ask(f"{prompt} [{hint}]", "")
    if not value:
        return default
    return value.lower().startswith("y")


def ask_choice(prompt, options, default_idx=0):
    say(f"  {prompt}")
    for i, label in enumerate(options, 1):
        mark = "  [default]" if i - 1 == default_idx else ""
        say(f"    {i}) {label}{mark}")
    while True:
        value = ask(">", str(default_idx + 1))
        if value.isdigit() and 1 <= int(value) <= len(options):
            return int(value) - 1
        say("  Please enter one of the numbers from the list.")
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest tools/tests/test_ui.py -v`
Expected: 5 PASS.

- [ ] **Step 5: Commit**

```bash
git add tools/ui.py tools/tests/test_ui.py && git commit -m "tools: ui prompt helpers"
```

---

### Task 3: `geo.py` — coordinate parsing, elevation, pin offsets

**Files:**
- Create: `tools/geo.py`
- Test: `tools/tests/test_geo.py`

**Interfaces:**
- Produces: `BadCoordinates(ValueError)`; `parse_latlng(text) -> (lat, lng)`; `wrap_lng_delta(delta) -> float` (shortest signed longitude difference); `get_elevation(lat, lng, timeout=10) -> float | None` (None on any failure or when `HIL_OFFLINE` is set); `pin_to_offset(pin_lat, pin_lng, meta) -> (east_m, north_m)` where `meta` has `center_lat`/`center_lng` (the dict from `fetch_ortho.fetch_mosaic`, Task 4); `distance_m(lat1, lng1, lat2, lng2) -> float`.
- `parse_latlng` still accepts the full ±90 range: SITL and SIH have no imagery and work anywhere. The Web-Mercator limit is enforced in `fetch_ortho` (Task 4), which is where it actually bites.

- [ ] **Step 1: Write the failing tests**

`tools/tests/test_geo.py`:

```python
import pytest

import geo


@pytest.mark.parametrize("text", [
    "39.58012, -104.84325",
    "39.58012 -104.84325",
    " 39.58012,-104.84325 ",
    "39.58012°, -104.84325°",
])
def test_parse_latlng_formats(text):
    lat, lng = geo.parse_latlng(text)
    assert lat == pytest.approx(39.58012)
    assert lng == pytest.approx(-104.84325)


@pytest.mark.parametrize("text", ["", "hello", "39.5", "91, 0", "0, 181", "1 2 3"])
def test_parse_latlng_rejects(text):
    with pytest.raises(geo.BadCoordinates):
        geo.parse_latlng(text)


def test_get_elevation_offline_env(monkeypatch):
    monkeypatch.setenv("HIL_OFFLINE", "1")
    assert geo.get_elevation(39.0, -104.0) is None


def test_get_elevation_parses_api(monkeypatch):
    class FakeResp:
        def raise_for_status(self):
            pass
        def json(self):
            return {"elevation": [1793.0]}
    monkeypatch.delenv("HIL_OFFLINE", raising=False)
    monkeypatch.setattr(geo.requests, "get", lambda *a, **k: FakeResp())
    assert geo.get_elevation(39.0, -104.0) == 1793.0


def test_get_elevation_none_on_error(monkeypatch):
    def boom(*a, **k):
        raise geo.requests.ConnectionError()
    monkeypatch.delenv("HIL_OFFLINE", raising=False)
    monkeypatch.setattr(geo.requests, "get", boom)
    assert geo.get_elevation(39.0, -104.0) is None


def test_pin_to_offset_directions():
    meta = {"center_lat": 39.0, "center_lng": -104.0}
    east, north = geo.pin_to_offset(39.001, -104.0, meta)   # pin north of center
    assert north == pytest.approx(111.0, rel=0.05)
    assert east == pytest.approx(0.0, abs=1e-6)
    east, north = geo.pin_to_offset(39.0, -103.999, meta)   # pin east of center
    assert east == pytest.approx(86.6, rel=0.05)            # 111412*cos(39)/1000
    assert north == pytest.approx(0.0, abs=1e-6)


def test_wrap_lng_delta():
    assert geo.wrap_lng_delta(-359.999) == pytest.approx(0.001)
    assert geo.wrap_lng_delta(359.999) == pytest.approx(-0.001)
    assert geo.wrap_lng_delta(179.9) == pytest.approx(179.9)
    assert geo.wrap_lng_delta(0.0) == pytest.approx(0.0)


def test_pin_to_offset_across_the_antimeridian():
    # center just west of 180, pin just east of it: 111 m apart, not 40 000 km
    meta = {"center_lat": 0.0, "center_lng": 179.9995}
    east, north = geo.pin_to_offset(0.0, -179.9995, meta)
    assert east == pytest.approx(111.4, rel=0.05)
    assert north == pytest.approx(0.0, abs=1e-6)


def test_distance_m():
    assert geo.distance_m(39.0, -104.0, 39.001, -104.0) == pytest.approx(
        111.0, rel=0.05)
    assert geo.distance_m(39.0, -104.0, 39.0, -104.0) == pytest.approx(0.0)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest tools/tests/test_geo.py -v`
Expected: FAIL with "No module named 'geo'".

- [ ] **Step 3: Write `tools/geo.py`**

```python
"""Coordinate parsing, elevation lookup, and pin-to-offset math."""
import math
import os

import requests


class BadCoordinates(ValueError):
    pass


def parse_latlng(text):
    """Accept Google-Maps-style strings: '39.58012, -104.84325' (also with
    spaces or degree signs). Returns (lat, lng)."""
    cleaned = text.strip().replace("°", " ").replace(",", " ")
    parts = [p for p in cleaned.split() if p]
    if len(parts) != 2:
        raise BadCoordinates(
            "expected two numbers, like: 39.58012, -104.84325")
    try:
        lat, lng = float(parts[0]), float(parts[1])
    except ValueError:
        raise BadCoordinates("could not read those as numbers")
    if not -90 <= lat <= 90 or not -180 <= lng <= 180:
        raise BadCoordinates(
            "latitude must be between -90 and 90, longitude -180 and 180")
    return lat, lng


def wrap_lng_delta(delta):
    """Shortest signed longitude difference in degrees. Without this, two
    points either side of the antimeridian look ~40 000 km apart."""
    return (delta + 180.0) % 360.0 - 180.0


def _meters_per_degree(lat):
    return (111132.92 - 559.82 * math.cos(2 * math.radians(lat)),
            111412.84 * math.cos(math.radians(lat)))


def get_elevation(lat, lng, timeout=10):
    """Ground elevation in meters (Open-Meteo), or None if unavailable."""
    if os.environ.get("HIL_OFFLINE"):
        return None
    try:
        r = requests.get("https://api.open-meteo.com/v1/elevation",
                         params={"latitude": lat, "longitude": lng},
                         timeout=timeout)
        r.raise_for_status()
        return float(r.json()["elevation"][0])
    except Exception:
        return None


def pin_to_offset(pin_lat, pin_lng, meta):
    """Meters east/north from the mosaic center to a lat/lng pin."""
    lat_c = meta["center_lat"]
    m_per_deg_lat, m_per_deg_lng = _meters_per_degree(lat_c)
    east = wrap_lng_delta(pin_lng - meta["center_lng"]) * m_per_deg_lng
    north = (pin_lat - lat_c) * m_per_deg_lat
    return east, north


def distance_m(lat1, lng1, lat2, lng2):
    """Approximate ground distance in meters (local flat-earth). Used to tell
    the operator when the saved site, the world and the Cube disagree."""
    m_per_deg_lat, m_per_deg_lng = _meters_per_degree((lat1 + lat2) / 2.0)
    return math.hypot(wrap_lng_delta(lng2 - lng1) * m_per_deg_lng,
                      (lat2 - lat1) * m_per_deg_lat)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest tools/tests/test_geo.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add tools/geo.py tools/tests/test_geo.py && git commit -m "tools: geo parsing, elevation, pin offsets"
```

---

### Task 4: `fetch_ortho.py` — imagery download (moved + refactored)

**Files:**
- Create: `tools/fetch_ortho.py` (source basis: `~/git/IFT/IFT-ardupilot/docs/hitl-tools/fetch_ortho.py`, refactored into functions)
- Test: `tools/tests/test_fetch_ortho.py`

**Interfaces:**
- Produces: `ImageryError(RuntimeError)`; `MERCATOR_LAT_LIMIT = 85.0511`; `MAX_PX = 5120`; `latlng_to_tile(lat, lng, z) -> (x, y)` floats (raises `ImageryError` beyond the Mercator limit); `tile_to_latlng(x, y, z) -> (lat, lng)`; `zoom_available(lat, lng, zoom, nside) -> bool` (network probe; False when `HIL_OFFLINE`, on non-200, or when the response is not a 256×256 image); `pick_zoom_nside(lat, lng, target_m) -> (zoom, nside)`; `fetch_mosaic(lat, lng, zoom, nside, out, progress=None) -> meta` where `meta` is a dict with keys `center_lat, center_lng, bounds{north,south,west,east}, size_ew_m, size_ns_m, px, gsd_m, zoom, texture`; `progress` is called as `progress(row_done, total_rows)`. CLI preserved: `python3 tools/fetch_ortho.py <lat> <lng> <zoom> <nside> <out_jpg>`.
- Tile X wraps across the antimeridian (`x % 2**zoom`); tile Y is validated, never wrapped — running off the top or bottom of the map is a plain-English error, not a garbage mosaic.
- Longitudes are folded back into [-180, 180) by `_wrap_lng`, but **only where the dict is built**. The extents are the raw difference of the unwrapped bounds, so wrapping before that arithmetic turns `size_ew_m` from `+117505` into `-39991117`. Both `center_lng` and the reported `bounds` need the fold: without it a site at exactly longitude 180 yields `center_lng = 180.00034`, which is written to the Gazebo world and to `SIM_OPOS_LNG` on the board. A consequence worth knowing: for a block straddling ±180, `bounds.east < bounds.west`. Nothing consumes `bounds` — it is a record in `site.json` — but a future consumer must not assume the pair is ordered.
- Not attempted: detecting an imagery server's "no data here" placeholder tile, which is a valid image. Zoom probing plus the operator's own `xdg-open` look at the picture covers that.

- [ ] **Step 1: Write the failing tests**

`tools/tests/test_fetch_ortho.py`:

```python
import io

import pytest
from PIL import Image

import fetch_ortho


def test_tile_roundtrip():
    lat, lng = -35.3632622, 149.165237
    x, y = fetch_ortho.latlng_to_tile(lat, lng, 19)
    lat2, lng2 = fetch_ortho.tile_to_latlng(x, y, 19)
    assert lat2 == pytest.approx(lat, abs=1e-9)
    assert lng2 == pytest.approx(lng, abs=1e-9)


def test_tile_zoom0_bounds():
    x, y = fetch_ortho.latlng_to_tile(0.0, 0.0, 0)
    assert x == pytest.approx(0.5)
    assert y == pytest.approx(0.5)


def test_latlng_to_tile_rejects_beyond_mercator():
    with pytest.raises(fetch_ortho.ImageryError):
        fetch_ortho.latlng_to_tile(85.2, 0.0, 19)
    with pytest.raises(fetch_ortho.ImageryError):
        fetch_ortho.latlng_to_tile(-88.0, 0.0, 19)


def _fake_tile():
    buf = io.BytesIO()
    Image.new("RGB", (256, 256), (10, 120, 10)).save(buf, "JPEG")
    return buf.getvalue()


class FakeResp:
    def __init__(self):
        self.content = _fake_tile()
        self.status_code = 200
    def raise_for_status(self):
        pass


def fake_session(seen=None):
    class FakeSession:
        def get(self, url, timeout=30):
            if seen is not None:
                seen.append(url)
            return FakeResp()
    return FakeSession


def test_fetch_mosaic_stitches_and_measures(tmp_path, monkeypatch):
    monkeypatch.setattr(fetch_ortho.requests, "Session", fake_session())
    out = tmp_path / "m.jpg"
    meta = fetch_ortho.fetch_mosaic(-35.3632622, 149.165237, 19, 3, str(out))
    assert out.exists()
    img = Image.open(out)
    assert img.size == (768, 768)
    assert meta["px"] == 768
    assert meta["zoom"] == 19
    assert meta["bounds"]["north"] > meta["bounds"]["south"]
    assert meta["bounds"]["east"] > meta["bounds"]["west"]
    assert 100 < meta["size_ew_m"] < 300          # 3 tiles at z19, lat -35
    assert abs(meta["size_ew_m"] - meta["size_ns_m"]) < 5
    assert meta["bounds"]["south"] < meta["center_lat"] < meta["bounds"]["north"]


def test_fetch_mosaic_wraps_tile_x_across_the_antimeridian(tmp_path,
                                                          monkeypatch):
    seen = []
    monkeypatch.setattr(fetch_ortho.requests, "Session", fake_session(seen))
    fetch_ortho.fetch_mosaic(0.0, 179.9999, 10, 3, str(tmp_path / "m.jpg"))
    xs = [int(u.rsplit("/", 1)[1]) for u in seen]
    assert all(0 <= x < 2 ** 10 for x in xs)      # no out-of-range tile asked for
    assert 0 in xs                                # and it did wrap round


def test_fetch_mosaic_rejects_a_non_image_response(tmp_path, monkeypatch):
    class BadResp:
        content = b"<html>tile temporarily unavailable</html>"
        status_code = 200
        def raise_for_status(self):
            pass

    class BadSession:
        def get(self, url, timeout=30):
            return BadResp()
    monkeypatch.setattr(fetch_ortho.requests, "Session", BadSession)
    with pytest.raises(fetch_ortho.ImageryError):
        fetch_ortho.fetch_mosaic(0.0, 0.0, 19, 3, str(tmp_path / "m.jpg"))


def test_fetch_mosaic_rejects_running_off_the_map(tmp_path, monkeypatch):
    monkeypatch.setattr(fetch_ortho.requests, "Session", fake_session())
    with pytest.raises(fetch_ortho.ImageryError):
        # zoom 0 has exactly one tile; a 3x3 block cannot exist
        fetch_ortho.fetch_mosaic(0.0, 0.0, 0, 3, str(tmp_path / "m.jpg"))


def test_pick_zoom_prefers_20(monkeypatch):
    monkeypatch.setattr(fetch_ortho, "zoom_available", lambda *a, **k: True)
    zoom, nside = fetch_ortho.pick_zoom_nside(39.0, -104.0, 560)
    assert zoom == 20
    assert nside * 256 <= fetch_ortho.MAX_PX


def test_pick_zoom_falls_back_to_19(monkeypatch):
    monkeypatch.setattr(fetch_ortho, "zoom_available",
                        lambda lat, lng, zoom, nside: zoom == 19)
    zoom, nside = fetch_ortho.pick_zoom_nside(-35.36, 149.16, 560)
    assert zoom == 19


def test_pick_zoom_large_area_caps_texture(monkeypatch):
    monkeypatch.setattr(fetch_ortho, "zoom_available", lambda *a, **k: True)
    zoom, nside = fetch_ortho.pick_zoom_nside(-35.36, 149.16, 1100)
    assert nside * 256 <= fetch_ortho.MAX_PX


def test_antimeridian_bounds_and_center_stay_in_range(tmp_path, monkeypatch):
    """Tile arithmetic runs unwrapped so extents stay computable, but every
    longitude handed out must be a real one. A naive wrap before the extent
    maths turns size_ew_m into a large negative number — this pins both
    halves at once."""
    monkeypatch.setattr(fetch_ortho.requests, "Session", fake_session())
    meta = fetch_ortho.fetch_mosaic(0.0, 179.9999, 10, 3,
                                    str(tmp_path / "m.jpg"))
    assert -180.0 <= meta["center_lng"] <= 180.0
    assert -180.0 <= meta["bounds"]["west"] <= 180.0
    assert -180.0 <= meta["bounds"]["east"] <= 180.0
    assert meta["size_ew_m"] > 0
    assert meta["size_ns_m"] > 0


def test_center_lng_valid_at_exactly_180(tmp_path, monkeypatch):
    """center_lng reaches the Gazebo world and SIM_OPOS_LNG on the board, so
    it has to be a longitude the firmware will accept."""
    monkeypatch.setattr(fetch_ortho.requests, "Session", fake_session())
    meta = fetch_ortho.fetch_mosaic(0.0, 180.0, 19, 3, str(tmp_path / "m.jpg"))
    assert -180.0 <= meta["center_lng"] <= 180.0


def test_zoom_available_offline(monkeypatch):
    monkeypatch.setenv("HIL_OFFLINE", "1")
    assert fetch_ortho.zoom_available(0, 0, 19, 3) is False


def test_zoom_available_false_on_non_image(monkeypatch):
    monkeypatch.delenv("HIL_OFFLINE", raising=False)

    class BadResp:
        content = b"not a picture"
        status_code = 200
    monkeypatch.setattr(fetch_ortho.requests, "get",
                        lambda *a, **k: BadResp())
    assert fetch_ortho.zoom_available(0, 0, 19, 3) is False
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest tools/tests/test_fetch_ortho.py -v`
Expected: FAIL with "No module named 'fetch_ortho'".

- [ ] **Step 3: Write `tools/fetch_ortho.py`**

```python
#!/usr/bin/env python3
"""Fetch a tile-aligned orthophoto mosaic around a lat/lng and emit
geo-registration data for the Gazebo ground-plane model.

CLI (developers): fetch_ortho.py <lat> <lng> <zoom> <n_tiles_per_side> <out_jpg>
Library (site_wizard): fetch_mosaic(), pick_zoom_nside(), zoom_available().
"""
import io
import json
import math
import os
import sys

import requests
from PIL import Image

TILE_URL = ("https://server.arcgisonline.com/ArcGIS/rest/services/"
            "World_Imagery/MapServer/tile/{z}/{y}/{x}")
TILE_PX = 256
MAX_PX = 5120           # texture cap per side (GPU-friendly)
MERCATOR_LAT_LIMIT = 85.0511    # Web Mercator is undefined past this


class ImageryError(RuntimeError):
    pass


def latlng_to_tile(lat, lng, z):
    if abs(lat) > MERCATOR_LAT_LIMIT:
        raise ImageryError(
            "satellite imagery only covers up to 85 degrees north and south")
    n = 2 ** z
    x = (lng + 180.0) / 360.0 * n
    lat_r = math.radians(lat)
    y = (1.0 - math.asinh(math.tan(lat_r)) / math.pi) / 2.0 * n
    return x, y


def tile_to_latlng(x, y, z):
    n = 2 ** z
    lng = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))
    return lat, lng


def _wrap_lng(lng):
    """Fold a longitude back into [-180, 180). Tile arithmetic runs on an
    unwrapped axis so extents stay computable as plain differences; only the
    values we hand out get folded."""
    return (lng + 180.0) % 360.0 - 180.0


def _block_origin(lat, lng, zoom, nside):
    """Top-left tile of the block. X may be negative or past the edge — it
    wraps at request time; Y may not, so it is checked here."""
    cx, cy = latlng_to_tile(lat, lng, zoom)
    y0 = int(cy) - nside // 2
    if y0 < 0 or y0 + nside > 2 ** zoom:
        raise ImageryError(
            "that area runs off the top or bottom of the imagery map — "
            "pick a site farther from the poles, or a smaller area")
    return int(cx) - nside // 2, y0


def _tile_url(x, y, zoom):
    return TILE_URL.format(z=zoom, x=x % (2 ** zoom), y=y)


def _decode_tile(content):
    """A 200 is not proof of a picture — imagery servers answer with HTML
    error pages, and PIL would happily paste garbage otherwise."""
    try:
        img = Image.open(io.BytesIO(content))
        img.load()
    except Exception as err:
        raise ImageryError(
            f"the imagery server sent something that is not a picture ({err})")
    if img.size != (TILE_PX, TILE_PX):
        raise ImageryError(f"unexpected tile size {img.size}")
    return img.convert("RGB")


def zoom_available(lat, lng, zoom, nside, timeout=15):
    """True if the tile server serves this zoom across the whole block
    (probes the four corners and the center)."""
    if os.environ.get("HIL_OFFLINE"):
        return False
    try:
        x0, y0 = _block_origin(lat, lng, zoom, nside)
    except ImageryError:
        return False
    probes = [(x0, y0), (x0 + nside - 1, y0), (x0, y0 + nside - 1),
              (x0 + nside - 1, y0 + nside - 1),
              (x0 + nside // 2, y0 + nside // 2)]
    for x, y in probes:
        try:
            r = requests.get(_tile_url(x, y, zoom), timeout=timeout)
            if r.status_code != 200:
                return False
            _decode_tile(r.content)
        except (requests.RequestException, ImageryError):
            return False
    return True


def pick_zoom_nside(lat, lng, target_m):
    """Best (zoom, nside) covering target_m meters: finest imagery the
    server has here, within the MAX_PX texture cap."""
    for zoom in (20, 19):
        mpp = 156543.03392 * math.cos(math.radians(lat)) / 2 ** zoom
        nside = max(3, math.ceil(target_m / (TILE_PX * mpp)))
        if nside * TILE_PX <= MAX_PX and zoom_available(lat, lng, zoom, nside):
            return zoom, nside
    zoom = 19
    mpp = 156543.03392 * math.cos(math.radians(lat)) / 2 ** zoom
    nside = min(math.ceil(target_m / (TILE_PX * mpp)), MAX_PX // TILE_PX)
    return zoom, max(3, nside)


def fetch_mosaic(lat, lng, zoom, nside, out, progress=None):
    """Download an nside x nside tile block centered on lat/lng, stitch it
    to `out` (JPEG), and return geo-registration metadata."""
    x0, y0 = _block_origin(lat, lng, zoom, nside)

    mosaic = Image.new("RGB", (nside * TILE_PX, nside * TILE_PX))
    sess = requests.Session()
    for j in range(nside):
        for i in range(nside):
            url = _tile_url(x0 + i, y0 + j, zoom)
            tile, last_err = None, None
            for _attempt in range(3):
                try:
                    r = sess.get(url, timeout=30)
                    r.raise_for_status()
                    tile = _decode_tile(r.content)   # ImageryError: don't retry
                    last_err = None
                    break
                except requests.RequestException as err:
                    last_err = err
            if last_err is not None:
                raise ImageryError(f"imagery download failed: {last_err}")
            mosaic.paste(tile, (i * TILE_PX, j * TILE_PX))
        if progress:
            progress(j + 1, nside)

    mosaic.save(out, quality=92)

    # Geographic bounds of the tile block (image top edge = north = y0)
    lat_n, lng_w = tile_to_latlng(x0, y0, zoom)
    lat_s, lng_e = tile_to_latlng(x0 + nside, y0 + nside, zoom)
    lat_c = (lat_n + lat_s) / 2.0
    lng_c = (lng_w + lng_e) / 2.0

    # Ground extents in meters (local flat-earth at center latitude)
    m_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * math.radians(lat_c))
    m_per_deg_lng = 111412.84 * math.cos(math.radians(lat_c))
    size_ns = (lat_n - lat_s) * m_per_deg_lat
    size_ew = (lng_e - lng_w) * m_per_deg_lng

    return {
        "center_lat": round(lat_c, 8),
        "center_lng": round(_wrap_lng(lng_c), 8),
        "bounds": {"north": lat_n, "south": lat_s,
                   "west": _wrap_lng(lng_w), "east": _wrap_lng(lng_e)},
        "size_ew_m": round(size_ew, 2),
        "size_ns_m": round(size_ns, 2),
        "px": nside * TILE_PX,
        "gsd_m": round(size_ew / (nside * TILE_PX), 4),
        "zoom": zoom,
        "texture": str(out),
    }


def main():
    lat, lng = float(sys.argv[1]), float(sys.argv[2])
    zoom, nside = int(sys.argv[3]), int(sys.argv[4])
    meta = fetch_mosaic(lat, lng, zoom, nside, sys.argv[5],
                        progress=lambda j, n: print(f"row {j}/{n}",
                                                    file=sys.stderr))
    print(json.dumps(meta, indent=2))


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest tools/tests/test_fetch_ortho.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add tools/fetch_ortho.py tools/tests/test_fetch_ortho.py
git commit -m "tools: imagery fetcher (moved from IFT-ardupilot, refactored)"
```

---

### Task 5: `gazebo_files.py` — SDF rewriting

**Files:**
- Create: `tools/gazebo_files.py`
- Create: `tools/tests/sdf_fixtures.py` (shared SDF fixtures — Tasks 8 and 9 import them too)
- Test: `tools/tests/test_gazebo_files.py`

**Interfaces:**
- Consumes: `meta` dict from `fetch_ortho.fetch_mosaic` **plus** an `elevation_m` key the caller merges in.
- Produces: `GazeboFilesError(RuntimeError)`; `apply_site(gazebo_dir, meta, spawn_east=0.0, spawn_north=0.0, texture_src=None) -> list[Path]`; `read_current(gazebo_dir) -> (lat, lng, elevation) | None`.
- File targets inside `gazebo_dir`: `worlds/if800_survey.sdf` (tags `<latitude_deg>`, `<longitude_deg>`, `<elevation>`, and the `<pose degrees="true">` immediately after `<uri>model://iris_with_gimbal</uri>`) and `models/survey_ground/model.sdf` (exactly two `<plane><size>` entries), texture at `models/survey_ground/materials/textures/survey_ortho.jpg`.
- **Write discipline** (these files are untracked by git — this module is the only safety net):
  1. Every edit is computed and validated in memory first. A malformed source aborts before anything on disk is touched, so it can never destroy its own backup.
  2. `<file>.orig` is a one-time pristine baseline, written once and never overwritten.
  3. `<file>.bak` is the previous state, refreshed on each successful apply.
  4. Writes go to a temp file and `os.replace()` onto the destination.
  5. The world's `<physics>` block is compared before and after; any change is an internal error, not a silent RTF regression.

- [ ] **Step 1: Write the shared SDF fixtures**

`tools/tests/sdf_fixtures.py` — Tasks 8 and 9 import these too, so there is
exactly one description of what the bench world looks like:

```python
"""Miniature stand-ins for the bench's Gazebo files, shared by every test
that rewrites them. One description of the world, so the test modules cannot
drift apart on what they expect it to contain."""
import re

WORLD = """<sdf version="1.9"><world name="if800_survey">
  <physics name="5ms" type="ignore">
    <max_step_size>0.005</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>
  <spherical_coordinates>
    <latitude_deg>-35.36301596</latitude_deg>
    <longitude_deg>149.16515350</longitude_deg>
    <elevation>584</elevation>
    <heading_deg>0</heading_deg>
  </spherical_coordinates>
  <include>
    <uri>model://survey_ground</uri>
    <pose>0 0 0 0 0 0</pose>
  </include>
  <include>
    <uri>model://iris_with_gimbal</uri>
    <pose degrees="true">0 0 0.195 0 0 90</pose>
  </include>
</world></sdf>
"""

MODEL = """<sdf version="1.9"><model name="survey_ground"><link name="link">
  <collision name="c"><geometry><plane>
    <size>561.48 559.14</size>
  </plane></geometry></collision>
  <visual name="v"><geometry><plane>
    <size>561.48 559.14</size>
  </plane></geometry></visual>
</link></model></sdf>
"""


def physics_block(text):
    """The block the wizard must never touch."""
    return re.search(r"<physics\b.*?</physics>", text, re.S).group(0)
```

- [ ] **Step 2: Write the failing tests**

`tools/tests/test_gazebo_files.py`:

```python
import pytest

import gazebo_files
from sdf_fixtures import MODEL, WORLD, physics_block

META = {"center_lat": 39.58001, "center_lng": -104.84301,
        "size_ew_m": 480.25, "size_ns_m": 478.5, "elevation_m": 1793}


@pytest.fixture
def gz(tmp_path):
    (tmp_path / "worlds").mkdir()
    (tmp_path / "models/survey_ground/materials/textures").mkdir(parents=True)
    (tmp_path / "worlds/if800_survey.sdf").write_text(WORLD)
    (tmp_path / "models/survey_ground/model.sdf").write_text(MODEL)
    return tmp_path


def test_apply_site_rewrites_everything(gz, tmp_path):
    tex = tmp_path / "new.jpg"
    tex.write_bytes(b"jpegdata")
    changed = gazebo_files.apply_site(gz, META, 25.0, -40.0, texture_src=tex)
    world = (gz / "worlds/if800_survey.sdf").read_text()
    assert "<latitude_deg>39.58001000</latitude_deg>" in world
    assert "<longitude_deg>-104.84301000</longitude_deg>" in world
    assert "<elevation>1793</elevation>" in world
    assert '<pose degrees="true">25.0 -40.0 0.195 0 0 90</pose>' in world
    model = (gz / "models/survey_ground/model.sdf").read_text()
    assert model.count("<size>480.25 478.50</size>") == 2
    dest = gz / "models/survey_ground/materials/textures/survey_ortho.jpg"
    assert dest.read_bytes() == b"jpegdata"
    assert (gz / "worlds/if800_survey.sdf.bak").exists()
    assert (gz / "models/survey_ground/model.sdf.bak").exists()
    assert len(changed) == 3


def test_apply_site_leaves_the_physics_block_alone(gz):
    gazebo_files.apply_site(gz, META, 25.0, -40.0)
    world = (gz / "worlds/if800_survey.sdf").read_text()
    assert physics_block(world) == physics_block(WORLD)


def test_apply_site_only_touches_iris_pose(gz):
    gazebo_files.apply_site(gz, META, 10.0, 10.0)
    world = (gz / "worlds/if800_survey.sdf").read_text()
    assert "<pose>0 0 0 0 0 0</pose>" in world      # survey_ground untouched


def test_apply_site_idempotent(gz):
    gazebo_files.apply_site(gz, META, 25.0, -40.0)
    first = (gz / "worlds/if800_survey.sdf").read_text()
    gazebo_files.apply_site(gz, META, 25.0, -40.0)
    assert (gz / "worlds/if800_survey.sdf").read_text() == first


def test_apply_site_rejects_unexpected_structure(gz):
    (gz / "worlds/if800_survey.sdf").write_text("<sdf></sdf>")
    with pytest.raises(gazebo_files.GazeboFilesError):
        gazebo_files.apply_site(gz, META)


def test_malformed_world_destroys_nothing(gz):
    """The old code backed up first, so a broken world overwrote the very
    backup its error message told the operator to restore."""
    gazebo_files.apply_site(gz, META)                    # good baseline exists
    good_orig = (gz / "worlds/if800_survey.sdf.orig").read_text()
    (gz / "worlds/if800_survey.sdf").write_text("<sdf>broken</sdf>")
    with pytest.raises(gazebo_files.GazeboFilesError):
        gazebo_files.apply_site(gz, META)
    assert (gz / "worlds/if800_survey.sdf.orig").read_text() == good_orig
    assert (gz / "worlds/if800_survey.sdf.bak").read_text() != "<sdf>broken</sdf>"
    assert (gz / "worlds/if800_survey.sdf").read_text() == "<sdf>broken</sdf>"


def test_unreadable_texture_aborts_before_any_write(gz, tmp_path):
    with pytest.raises(gazebo_files.GazeboFilesError):
        gazebo_files.apply_site(gz, META, texture_src=tmp_path / "missing.jpg")
    assert (gz / "worlds/if800_survey.sdf").read_text() == WORLD
    assert (gz / "models/survey_ground/model.sdf").read_text() == MODEL
    assert not (gz / "worlds/if800_survey.sdf.bak").exists()


def test_orig_baseline_survives_repeated_site_switches(gz):
    gazebo_files.apply_site(gz, META)
    gazebo_files.apply_site(gz, {**META, "center_lat": 10.0})
    gazebo_files.apply_site(gz, {**META, "center_lat": 20.0})
    assert (gz / "worlds/if800_survey.sdf.orig").read_text() == WORLD
    assert (gz / "models/survey_ground/model.sdf.orig").read_text() == MODEL


def test_apply_site_missing_dir(tmp_path):
    with pytest.raises(gazebo_files.GazeboFilesError):
        gazebo_files.apply_site(tmp_path / "nope", META)


def test_read_current(gz):
    lat, lng, elev = gazebo_files.read_current(gz)
    assert lat == pytest.approx(-35.36301596)
    assert lng == pytest.approx(149.16515350)
    assert elev == pytest.approx(584)


def test_read_current_missing(tmp_path):
    assert gazebo_files.read_current(tmp_path) is None
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `python3 -m pytest tools/tests/test_gazebo_files.py -v`
Expected: FAIL with "No module named 'gazebo_files'".

- [ ] **Step 4: Write `tools/gazebo_files.py`**

```python
"""Rewrites the if800_survey Gazebo world + survey_ground model for a new
site. Never touches the physics block (max_step_size stays 1/SCHED_LOOP_RATE).

Nothing this module edits is tracked by git, so it is the only way back:
every edit is computed and checked before anything is written, a one-time
.orig keeps the pristine baseline, .bak keeps the previous state, and each
destination is replaced atomically.
"""
import os
import re
import shutil
from pathlib import Path

PHYSICS_RE = re.compile(r"<physics\b.*?</physics>", re.S)


class GazeboFilesError(RuntimeError):
    pass


def _paths(gazebo_dir):
    gz = Path(gazebo_dir).expanduser()
    return (gz / "worlds/if800_survey.sdf",
            gz / "models/survey_ground/model.sdf",
            gz / "models/survey_ground/materials/textures/survey_ortho.jpg")


def _rewrite_world(text, meta, spawn_east, spawn_north, world):
    w, n_lat = re.subn(r"<latitude_deg>[^<]*</latitude_deg>",
                       f"<latitude_deg>{meta['center_lat']:.8f}</latitude_deg>",
                       text, count=1)
    w, n_lng = re.subn(r"<longitude_deg>[^<]*</longitude_deg>",
                       f"<longitude_deg>{meta['center_lng']:.8f}</longitude_deg>",
                       w, count=1)
    w, n_elev = re.subn(r"<elevation>[^<]*</elevation>",
                        f"<elevation>{meta['elevation_m']:.0f}</elevation>",
                        w, count=1)
    w, n_pose = re.subn(
        r'(<uri>model://iris_with_gimbal</uri>\s*<pose degrees="true">)[^<]*(</pose>)',
        rf"\g<1>{spawn_east:.1f} {spawn_north:.1f} 0.195 0 0 90\g<2>",
        w, count=1)
    if not all((n_lat, n_lng, n_elev, n_pose)):
        raise GazeboFilesError(
            f"{world} did not look as expected — nothing was changed. "
            f"Restore it from {world}.orig and ask a developer")
    if PHYSICS_RE.findall(w) != PHYSICS_RE.findall(text):
        raise GazeboFilesError(
            "internal error: the edit would have changed the physics block")
    return w


def _rewrite_model(text, meta, model):
    m, n_size = re.subn(
        r"<size>[^<]*</size>",
        f"<size>{meta['size_ew_m']:.2f} {meta['size_ns_m']:.2f}</size>", text)
    if n_size != 2:
        raise GazeboFilesError(
            f"expected 2 plane <size> entries in {model}, found {n_size} — "
            "nothing was changed")
    return m


def _baseline(path):
    """One-time pristine copy. Never overwritten, so switching sites a
    hundred times still leaves a way back to the original bench setup."""
    orig = path.with_suffix(path.suffix + ".orig")
    if not orig.exists():
        shutil.copy2(path, orig)


def _atomic_write(path, data):
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_bytes(data)
    os.replace(tmp, path)


def apply_site(gazebo_dir, meta, spawn_east=0.0, spawn_north=0.0,
               texture_src=None):
    """Point the survey world at a new mosaic. `meta` is the dict from
    fetch_ortho.fetch_mosaic plus an `elevation_m` key. Returns changed paths."""
    world, model, tex = _paths(gazebo_dir)
    for p in (world, model):
        if not p.exists():
            raise GazeboFilesError(
                f"missing {p} — External Physics is not set up on this laptop")

    # --- compute and validate everything before touching the disk ---
    world_text = world.read_text()
    model_text = model.read_text()
    new_world = _rewrite_world(world_text, meta, spawn_east, spawn_north, world)
    new_model = _rewrite_model(model_text, meta, model)
    tex_bytes = None
    if texture_src is not None:
        try:
            tex_bytes = Path(texture_src).read_bytes()
        except OSError as err:
            raise GazeboFilesError(
                f"could not read the new imagery ({err}) — nothing was changed")

    # --- from here on, every write is known to be good ---
    for p in (world, model):
        _baseline(p)
        shutil.copy2(p, p.with_suffix(p.suffix + ".bak"))
    _atomic_write(world, new_world.encode())
    _atomic_write(model, new_model.encode())
    changed = [world, model]

    if tex_bytes is not None:
        tex.parent.mkdir(parents=True, exist_ok=True)
        if tex.exists():
            _baseline(tex)
            shutil.copy2(tex, tex.with_suffix(tex.suffix + ".bak"))
        _atomic_write(tex, tex_bytes)
        changed.append(tex)
    return changed


def read_current(gazebo_dir):
    """(lat, lng, elevation) currently in the world file, or None."""
    world, _, _ = _paths(gazebo_dir)
    if not world.exists():
        return None
    w = world.read_text()
    try:
        lat = float(re.search(r"<latitude_deg>([^<]*)<", w).group(1))
        lng = float(re.search(r"<longitude_deg>([^<]*)<", w).group(1))
        elev = float(re.search(r"<elevation>([^<]*)<", w).group(1))
    except (AttributeError, ValueError):
        return None
    return lat, lng, elev
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `python3 -m pytest tools/tests/test_gazebo_files.py -v`
Expected: all PASS.

- [ ] **Step 6: Commit**

```bash
git add tools/gazebo_files.py tools/tests/sdf_fixtures.py tools/tests/test_gazebo_files.py
git commit -m "tools: gazebo world/model rewriter"
```

---

### Task 6: `board.py` — Cube params over USB

**Files:**
- Create: `tools/board.py`
- Test: `tools/tests/test_board.py`

**Interfaces:**
- Produces: `BoardError(RuntimeError)`; `HERELINK = {"SERIAL1_BAUD": 57, "SYSID_MYGCS": 255}`; `TAISYNC = {"SERIAL1_BAUD": 115, "SYSID_MYGCS": 19}`; `find_mavlink_port() -> str | None`; `connect(port, timeout=10) -> conn`; `set_params(conn, params: dict, retries=3) -> dict` (raises `BoardError` naming the first param that fails to verify); `get_param(conn, name, timeout=3) -> float`; `reboot(conn)`.
- **Verification rule:** MAVLink params are float32. Quantize the requested value to float32 and require the echo to match *that*, within re-encode noise only. Measured on the real values: float32 costs 0.19–0.69 m of coordinate precision, while the previously-proposed `abs(value) * 1e-6` relative tolerance would have accepted a 9.0 m error at Meridian and a 13.6 m error at CMAC.
- Every transport failure surfaces as `BoardError` so the wizard always reaches its manual-IGC fallback instead of aborting through the top-level handler.
- `param_id` is normalized: pymavlink 2.4.49 on this bench returns `str`, but a bytes payload must not silently fail every comparison.

- [ ] **Step 1: Write the failing tests**

`tools/tests/test_board.py`:

```python
import struct
import types

import pytest

import board


def f32(v):
    return struct.unpack("f", struct.pack("f", float(v)))[0]


class FakeMav:
    def __init__(self, conn):
        self.conn = conn
        self.commands = []

    def param_set_send(self, sysid, compid, name, value, ptype):
        self.conn.store[name.decode()] = value
        self.conn.last = name.decode()

    def param_request_read_send(self, sysid, compid, name, index):
        self.conn.last = name.decode()

    def command_long_send(self, *args):
        self.commands.append(args)


class FakeConn:
    target_system = 1
    target_component = 1

    def __init__(self, store=None):
        self.store = store if store is not None else {}
        self.last = None
        self.mav = FakeMav(self)

    def recv_match(self, type=None, blocking=True, timeout=1):
        if self.last is None or self.last not in self.store:
            return None
        # echo back as float32, like the real firmware
        return types.SimpleNamespace(param_id=self.last,
                                     param_value=f32(self.store[self.last]))

    def close(self):
        pass


def test_find_mavlink_port(monkeypatch):
    monkeypatch.delenv("HIL_NO_BOARD", raising=False)
    monkeypatch.setattr(board.glob, "glob",
                        lambda pat: ["/dev/serial/by-id/usb-Cube-if00-port0"])
    assert "if00" in board.find_mavlink_port()
    monkeypatch.setattr(board.glob, "glob", lambda pat: [])
    assert board.find_mavlink_port() is None


def test_find_mavlink_port_disabled_by_env(monkeypatch):
    monkeypatch.setenv("HIL_NO_BOARD", "1")
    monkeypatch.setattr(board.glob, "glob",
                        lambda pat: ["/dev/serial/by-id/usb-Cube-if00-port0"])
    assert board.find_mavlink_port() is None


def test_set_params_accepts_float32_quantization():
    conn = FakeConn()
    verified = board.set_params(conn, {"SIM_OPOS_LAT": -35.36301596,
                                       "SIM_OPOS_HDG": 0.0})
    assert set(verified) == {"SIM_OPOS_LAT", "SIM_OPOS_HDG"}
    assert verified["SIM_OPOS_LAT"] == pytest.approx(-35.36301596, abs=1e-5)


def test_set_params_rejects_a_nine_metre_longitude_error():
    """1e-4 deg at this longitude is ~9 m of ground error, and the old
    relative tolerance (abs(value) * 1e-6 = 1.5e-4 deg) accepted it."""
    class DriftConn(FakeConn):
        def recv_match(self, type=None, blocking=True, timeout=1):
            return types.SimpleNamespace(
                param_id=self.last,
                param_value=f32(self.store[self.last] + 1e-4))
    with pytest.raises(board.BoardError, match="SIM_OPOS_LNG"):
        board.set_params(DriftConn(), {"SIM_OPOS_LNG": 149.16515350},
                         retries=1)


def test_set_params_accepts_a_bytes_param_id():
    class BytesConn(FakeConn):
        def recv_match(self, type=None, blocking=True, timeout=1):
            return types.SimpleNamespace(
                param_id=self.last.encode() + b"\x00\x00",
                param_value=f32(self.store[self.last]))
    assert board.set_params(BytesConn(), {"SIM_OPOS_HDG": 0.0}, retries=1)


def test_set_params_raises_on_no_echo():
    class DeafConn(FakeConn):
        def recv_match(self, type=None, blocking=True, timeout=1):
            return None
    with pytest.raises(board.BoardError, match="SIM_OPOS_LAT"):
        board.set_params(DeafConn(), {"SIM_OPOS_LAT": 1.0}, retries=1)


def test_set_params_wraps_transport_errors_as_boarderror():
    """The wizard only catches BoardError; a raw serial exception here would
    skip the manual-IGC fallback and abort the whole wizard."""
    class BrokenConn(FakeConn):
        def recv_match(self, type=None, blocking=True, timeout=1):
            raise OSError("device disconnected")
    with pytest.raises(board.BoardError):
        board.set_params(BrokenConn(), {"SIM_OPOS_HDG": 0.0}, retries=1)


def test_get_param_reads_back():
    conn = FakeConn(store={"SYSID_MYGCS": 255.0})
    assert board.get_param(conn, "SYSID_MYGCS") == 255.0


def test_get_param_wraps_transport_errors():
    class BrokenConn(FakeConn):
        def recv_match(self, type=None, blocking=True, timeout=1):
            raise OSError("device disconnected")
    with pytest.raises(board.BoardError):
        board.get_param(BrokenConn(), "SYSID_MYGCS", timeout=1)


def test_reboot_sends_command():
    conn = FakeConn()
    board.reboot(conn)
    assert len(conn.mav.commands) == 1
    # command_long_send is (target_system, target_component, command,
    # confirmation, param1, ...) — so param1, the "reboot autopilot"
    # selector, is index 4. Index 3 is confirmation, which we send as 0.
    assert conn.mav.commands[0][3] == 0        # confirmation
    assert conn.mav.commands[0][4] == 1        # param1 = reboot autopilot


def test_radio_tables():
    assert board.HERELINK == {"SERIAL1_BAUD": 57, "SYSID_MYGCS": 255}
    assert board.TAISYNC == {"SERIAL1_BAUD": 115, "SYSID_MYGCS": 19}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest tools/tests/test_board.py -v`
Expected: FAIL with "No module named 'board'".

- [ ] **Step 3: Write `tools/board.py`**

```python
"""Talk to the Cube over the USB MAVLink port. IGC can stay connected the
whole time — it uses the SERIAL1 radio, not USB."""
import glob
import os
import struct
import time

HERELINK = {"SERIAL1_BAUD": 57, "SYSID_MYGCS": 255}
TAISYNC = {"SERIAL1_BAUD": 115, "SYSID_MYGCS": 19}


class BoardError(RuntimeError):
    pass


def _f32(value):
    """The value as the board will actually store it — MAVLink params are
    float32, which costs ~0.2-0.7 m on a coordinate. That much is expected;
    anything more means the write did not land."""
    return struct.unpack("f", struct.pack("f", float(value)))[0]


def _param_name(msg):
    name = msg.param_id
    if isinstance(name, (bytes, bytearray)):
        name = bytes(name).decode(errors="replace")
    return name.rstrip("\x00")


def find_mavlink_port():
    """The Cube's MAVLink CDC port (USB interface 00), or None.
    HIL_NO_BOARD=1 forces None (tests run on the bench where a real
    Cube is plugged in)."""
    if os.environ.get("HIL_NO_BOARD"):
        return None
    hits = sorted(glob.glob("/dev/serial/by-id/*if00*"))
    return hits[0] if hits else None


def connect(port, timeout=10):
    from pymavlink import mavutil
    try:
        conn = mavutil.mavlink_connection(port, baud=115200)
    except Exception as err:
        raise BoardError(f"could not open {port}: {err}")
    try:
        beat = conn.wait_heartbeat(timeout=timeout)
    except Exception as err:
        conn.close()
        raise BoardError(f"lost the USB link waiting for the Cube: {err}")
    if beat is None:
        conn.close()
        raise BoardError("no heartbeat from the Cube — check the USB cable")
    return conn


def set_params(conn, params, retries=3):
    """Set each param and verify the echo against the float32 value the
    board stores. Returns the verified values."""
    from pymavlink import mavutil
    verified = {}
    for name, value in params.items():
        want = _f32(value)
        tol = 1e-9 * max(1.0, abs(want))     # re-encode noise only (~0.1 mm)
        ok = False
        for _attempt in range(retries):
            try:
                conn.mav.param_set_send(
                    conn.target_system, conn.target_component,
                    name.encode(), float(value),
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                deadline = time.time() + 3
                while time.time() < deadline:
                    msg = conn.recv_match(type="PARAM_VALUE", blocking=True,
                                          timeout=1)
                    if msg is None or _param_name(msg) != name:
                        continue
                    if abs(msg.param_value - want) <= tol:
                        verified[name] = msg.param_value
                        ok = True
                    break
            except Exception as err:
                raise BoardError(f"lost the USB link while setting {name}: {err}")
            if ok:
                break
        if not ok:
            raise BoardError(f"{name} did not verify over USB")
    return verified


def get_param(conn, name, timeout=3):
    try:
        conn.mav.param_request_read_send(conn.target_system,
                                         conn.target_component,
                                         name.encode(), -1)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if msg is not None and _param_name(msg) == name:
                return float(msg.param_value)
    except Exception as err:
        raise BoardError(f"lost the USB link while reading {name}: {err}")
    raise BoardError(f"no answer reading {name}")


def reboot(conn):
    from pymavlink import mavutil
    try:
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0, 1, 0, 0, 0, 0, 0, 0)
    except Exception as err:
        raise BoardError(f"could not send the reboot command: {err}")
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest tools/tests/test_board.py -v`
Expected: all PASS. (Note: `set_params`/`reboot` import `mavutil` only for constants — the fake connection objects carry the traffic.)

- [ ] **Step 5: Commit**

```bash
git add tools/board.py tools/tests/test_board.py
git commit -m "tools: Cube param access over USB MAVLink"
```

---

### Task 7: `sites.py` — saved sites, launching, simulator readiness

**Files:**
- Create: `tools/sites.py`
- Test: `tools/tests/test_sites.py`

**Interfaces:**
- Produces: `SITES_DIR`, `GAZEBO_DIR`, `ARDUPILOT_DIR` (Path constants honoring `HIL_SITES_DIR`/`HIL_GAZEBO_DIR`/`HIL_ARDUPILOT_DIR`); `SIM_UDP_PORT = 9002`; `slugify(text) -> str`; `site_dir(name) -> Path`; `save_site(name, data)`; `load_site(name) -> dict`; `list_sites() -> list[str]`; `set_active(name)` / `get_active() -> str | None`; `write_launch_scripts(name, data) -> Path | None` (executable script path; None for SIH); `sim_port_busy(port=SIM_UDP_PORT, _list_udp=None) -> bool`; `wait_for_sim_port(timeout=90, poll=1.0) -> bool`; `ppp0_up(_runner=None) -> bool`; `launch_script(script, log_path) -> str` (human-readable description of where it went).
- `data` dict keys (set by Task 8): `name, mode ("sitl"|"sih"|"external"), lat, lng, elevation_m, heading, spawn_east, spawn_north, mosaic (fetch_mosaic meta dict or None), pin_lat, pin_lng`.
- **Why a UDP-port check, not `pgrep`:** the conflict is the simulator's `<fdm_port_in>9002</fdm_port_in>` socket, and `pgrep -f "gz sim"` matches any shell whose command line merely contains that string (observed live while writing this plan). `ss -lun` tests the actual resource, and doubles as the "is Gazebo ready for the board yet" probe.
- `launch_script` deliberately does **not** report success: with `gnome-terminal` the spawned process is the terminal, not the script, so its liveness proves nothing. Readiness is `wait_for_sim_port`.

- [ ] **Step 1: Write the failing tests**

`tools/tests/test_sites.py`:

```python
import shutil
import stat

import pytest

import sites


@pytest.fixture
def sandbox(tmp_path, monkeypatch):
    monkeypatch.setattr(sites, "SITES_DIR", tmp_path / "sites")
    monkeypatch.setattr(sites, "ARDUPILOT_DIR", tmp_path / "ap")
    return tmp_path


DATA = {"name": "meridian", "mode": "sitl", "lat": 39.58012,
        "lng": -104.84325, "elevation_m": 1793, "heading": 0.0,
        "spawn_east": 0.0, "spawn_north": 0.0, "mosaic": None,
        "pin_lat": 39.58012, "pin_lng": -104.84325}

SS_OUT = """State  Recv-Q Send-Q Local Address:Port  Peer Address:Port
UNCONN 0      0            0.0.0.0:53          0.0.0.0:*
UNCONN 0      0            0.0.0.0:19002       0.0.0.0:*
"""
SS_BUSY = SS_OUT + "UNCONN 0      0            0.0.0.0:9002        0.0.0.0:*\n"


def test_slugify():
    assert sites.slugify("Meridian Test Site!") == "meridian-test-site"
    assert sites.slugify("site--39.58") == "site-39-58"


def test_save_load_roundtrip(sandbox):
    sites.save_site("meridian", DATA)
    assert sites.load_site("meridian") == DATA
    assert sites.list_sites() == ["meridian"]


def test_list_ignores_junk(sandbox):
    sites.save_site("meridian", DATA)
    (sites.SITES_DIR / "notasite").mkdir()
    assert sites.list_sites() == ["meridian"]


def test_active_site_roundtrip(sandbox):
    assert sites.get_active() is None
    sites.save_site("meridian", DATA)
    sites.set_active("meridian")
    assert sites.get_active() == "meridian"
    assert sites.list_sites() == ["meridian"]      # the marker is not a site


def test_active_site_forgets_a_deleted_site(sandbox):
    sites.save_site("meridian", DATA)
    sites.set_active("meridian")
    shutil.rmtree(sites.site_dir("meridian"))
    assert sites.get_active() is None


def test_sitl_launch_script(sandbox):
    path = sites.write_launch_scripts("meridian", DATA)
    text = path.read_text()
    assert path.name == "launch-sitl.sh"
    assert "--custom-location=39.58012000,-104.84325000,1793.0,0" in text
    assert "--no-rebuild" in text
    assert str(sites.ARDUPILOT_DIR) in text
    assert path.stat().st_mode & stat.S_IXUSR


def test_gazebo_launch_script(sandbox):
    data = {**DATA, "mode": "external"}
    path = sites.write_launch_scripts("meridian", data)
    text = path.read_text()
    assert path.name == "launch-gazebo.sh"
    assert "ppp0" in text
    assert "gz sim -v4 -r if800_survey.sdf" in text
    assert "9002" in text                # guards the real conflict, the port
    assert "pgrep" not in text           # not process names
    assert path.stat().st_mode & stat.S_IXUSR


def test_sih_has_no_script(sandbox):
    assert sites.write_launch_scripts("meridian", {**DATA, "mode": "sih"}) is None


def test_sim_port_busy_detects_the_listener():
    assert sites.sim_port_busy(_list_udp=lambda: SS_BUSY) is True


def test_sim_port_busy_ignores_similar_ports():
    assert sites.sim_port_busy(_list_udp=lambda: SS_OUT) is False


def test_sim_port_busy_is_false_when_ss_fails():
    def boom():
        raise OSError("no ss")
    assert sites.sim_port_busy(_list_udp=boom) is False


def test_wait_for_sim_port_returns_immediately_when_up(monkeypatch):
    monkeypatch.setattr(sites, "sim_port_busy", lambda: True)
    assert sites.wait_for_sim_port(timeout=0) is True


def test_wait_for_sim_port_times_out(monkeypatch):
    monkeypatch.setattr(sites, "sim_port_busy", lambda: False)
    assert sites.wait_for_sim_port(timeout=0) is False


def test_ppp0_up_assumes_yes_when_it_cannot_tell():
    def boom():
        raise OSError("no ip command")
    assert sites.ppp0_up(_runner=boom) is True
    assert sites.ppp0_up(_runner=lambda: 0) is True
    assert sites.ppp0_up(_runner=lambda: 1) is False


def test_launch_script_uses_a_terminal_when_one_is_available(sandbox,
                                                             monkeypatch):
    spawned = []
    monkeypatch.setattr(sites.subprocess, "Popen",
                        lambda argv, **kw: spawned.append(argv))
    script, log = sandbox / "launch-gazebo.sh", sandbox / "gz.log"
    where = sites.launch_script(script, log)
    assert spawned == [["gnome-terminal", "--", "bash", str(script)]]
    assert "launch-gazebo.sh" in where
    assert not log.exists()          # a terminal took it; no log file needed


def test_launch_script_falls_back_to_a_log_file(sandbox, monkeypatch):
    """No gnome-terminal (headless bench): the script still runs, and the
    operator is told where its output went."""
    spawned = []

    def popen(argv, **kw):
        if argv[0] == "gnome-terminal":
            raise FileNotFoundError("no gnome-terminal here")
        spawned.append(argv)

    monkeypatch.setattr(sites.subprocess, "Popen", popen)
    script, log = sandbox / "launch-gazebo.sh", sandbox / "gz.log"
    where = sites.launch_script(script, log)
    assert spawned == [["bash", str(script)]]
    assert str(log) in where
    assert log.exists()              # opened for append
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest tools/tests/test_sites.py -v`
Expected: FAIL with "No module named 'sites'".

- [ ] **Step 3: Write `tools/sites.py`**

```python
"""Saved-site storage (tools/sites/<name>/), generated launch scripts, and
the checks that say whether the Gazebo simulator is free or ready."""
import json
import os
import re
import stat
import subprocess
import time
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
SITES_DIR = Path(os.environ.get("HIL_SITES_DIR", TOOLS_DIR / "sites"))
GAZEBO_DIR = Path(os.environ.get("HIL_GAZEBO_DIR",
                                 "~/git/ardupilot_gazebo")).expanduser()
ARDUPILOT_DIR = Path(os.environ.get("HIL_ARDUPILOT_DIR",
                                    "~/git/IFT/IFT-ardupilot")).expanduser()

# The board's physics link. Two simulators cannot share it, and its being
# bound is the only honest signal that Gazebo is ready for the Cube.
SIM_UDP_PORT = 9002
ACTIVE_MARKER = "active.txt"

LAUNCH_SITL = """#!/bin/bash
# Generated by site_wizard.py — desktop SITL at site: @NAME@
# Safe to run any time: everything stays on the laptop.
set -e
AP="@AP@"
BIN="$AP/build/sitl/bin/arducopter"
if [ ! -x "$BIN" ]; then
  echo "Desktop SITL has not been built on this laptop (missing $BIN)."
  echo "Ask a developer to build it once:"
  echo "  cd $AP && ./waf configure --board sitl && ./waf copter"
  echo "(Do NOT let sim_vehicle rebuild — the build directory is configured"
  echo " for the flight controller, not the desktop.)"
  exit 1
fi
cd "$AP"
exec Tools/autotest/sim_vehicle.py -v ArduCopter --no-rebuild --console --map \\
  --custom-location=@LAT@,@LNG@,@ALT@,@HDG@
"""

LAUNCH_GAZEBO = """#!/bin/bash
# Generated by site_wizard.py — Gazebo (External Physics) at site: @NAME@
set -e
if ! ip addr show ppp0 &>/dev/null; then
  echo "The USB network link (ppp0) is not up — start pppd first."
  echo "See the wiki page 'External Physics SIH' for the command."
  exit 1
fi
if ss -lun | grep -qE '[:.]@PORT@\\b'; then
  echo "Another simulator already has the physics port (@PORT@)."
  echo "Close that Gazebo window (Ctrl+C in its terminal), then run this"
  echo "script again — otherwise it would keep flying the old site."
  exit 1
fi
export GZ_VERSION=harmonic
exec gz sim -v4 -r if800_survey.sdf
"""


def slugify(text):
    slug = re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-")
    return re.sub(r"-+", "-", slug)


def site_dir(name):
    return SITES_DIR / name


def save_site(name, data):
    d = site_dir(name)
    d.mkdir(parents=True, exist_ok=True)
    (d / "site.json").write_text(json.dumps(data, indent=2))


def load_site(name):
    return json.loads((site_dir(name) / "site.json").read_text())


def list_sites():
    if not SITES_DIR.exists():
        return []
    return sorted(p.name for p in SITES_DIR.iterdir()
                  if (p / "site.json").exists())


def set_active(name):
    """Remember which saved site was last applied, so 'Show current site'
    can answer for SITL and SIH too, not just the Gazebo world."""
    SITES_DIR.mkdir(parents=True, exist_ok=True)
    (SITES_DIR / ACTIVE_MARKER).write_text(name + "\n")


def get_active():
    marker = SITES_DIR / ACTIVE_MARKER
    if not marker.exists():
        return None
    name = marker.read_text().strip()
    return name if (site_dir(name) / "site.json").exists() else None


def write_launch_scripts(name, data):
    """Write the launch script for this site's mode. Returns its path
    (None for SIH — nothing to launch on the laptop)."""
    if data["mode"] == "sih":
        return None
    d = site_dir(name)
    d.mkdir(parents=True, exist_ok=True)
    if data["mode"] == "sitl":
        path = d / "launch-sitl.sh"
        text = (LAUNCH_SITL
                .replace("@NAME@", name)
                .replace("@AP@", str(ARDUPILOT_DIR))
                .replace("@LAT@", f"{data['lat']:.8f}")
                .replace("@LNG@", f"{data['lng']:.8f}")
                .replace("@ALT@", f"{data['elevation_m']:.1f}")
                .replace("@HDG@", f"{data.get('heading', 0):.0f}"))
    else:  # external
        path = d / "launch-gazebo.sh"
        text = (LAUNCH_GAZEBO.replace("@NAME@", name)
                .replace("@PORT@", str(SIM_UDP_PORT)))
    path.write_text(text)
    path.chmod(path.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    return path


def _ss_listening_udp():
    return subprocess.run(["ss", "-lun"], capture_output=True, text=True,
                          timeout=5).stdout


def sim_port_busy(port=SIM_UDP_PORT, _list_udp=None):
    """True if something already holds the simulator's UDP port. This is the
    exact resource two Gazebos would fight over — far more precise than
    matching process names, which also hit unrelated shells."""
    lister = _list_udp or _ss_listening_udp
    try:
        out = lister()
    except Exception:
        return False        # can't tell — the launch script re-checks anyway
    return re.search(rf"[:.]{port}\b", out) is not None


def wait_for_sim_port(timeout=90, poll=1.0):
    """Block until Gazebo's ArduPilot plugin is listening (it binds the port
    while loading the world; a big texture makes that slow)."""
    deadline = time.time() + timeout
    while True:
        if sim_port_busy():
            return True
        if time.time() >= deadline:
            return False
        time.sleep(poll)


def ppp0_up(_runner=None):
    """The Cube's USB network link. Unknown counts as up: the generated
    script checks again and the operator should not be blocked by a
    missing `ip` command."""
    def run():
        return subprocess.run(["ip", "addr", "show", "ppp0"],
                              capture_output=True, text=True,
                              timeout=5).returncode
    try:
        return (_runner or run)() == 0
    except Exception:
        return True


def launch_script(script, log_path):
    """Run a generated launch script — new terminal window if possible,
    otherwise in the background with output to a log file. Returns where it
    went. Deliberately not a success signal: with a terminal window the
    process we spawn is the terminal, so its being alive proves nothing
    about the script. Use wait_for_sim_port() for that."""
    script = Path(script)
    try:
        subprocess.Popen(["gnome-terminal", "--", "bash", str(script)])
        return f"a new terminal window ({script.name})"
    except FileNotFoundError:
        pass
    with open(log_path, "ab") as log:
        subprocess.Popen(["bash", str(script)], stdout=log, stderr=log)
    return f"the background — output goes to {log_path}"
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest tools/tests/test_sites.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add tools/sites.py tools/tests/test_sites.py
git commit -m "tools: saved sites, launch scripts, simulator readiness checks"
```

---

### Task 8: `site_wizard.py` — the wizard itself

**Files:**
- Create: `tools/site_wizard.py`
- Test: `tools/tests/test_wizard_flows.py` (subprocess, piped answers — SITL / SIH / show / crash)

**Interfaces:**
- Consumes: everything above, by the exact names in each task's Produces block.
- Produces: `python3 tools/site_wizard.py [--debug]`. Menu: 1 new site / 2 saved site / 3 show current / 4 radio setup / 5 quit.
- **Order of operations for External Physics** (each rule exists because skipping it desynchronizes the Cube from the world):
  1. Preflight *before any change*: `ppp0` up, simulator port free. A Gazebo that is already running keeps the old world in memory; moving the Cube underneath it is the "drone on a grey void" failure.
  2. Download imagery → the site becomes the mosaic center → look up elevation **at that center**.
  3. Apply files, save site, mark active.
  4. Launch, then wait for the simulator port to be bound.
  5. Board params last; offer the reboot **only** if the simulator actually came up, and say plainly why not when it didn't.

- [ ] **Step 1: Write the failing integration tests (subprocess, piped answers)**

`tools/tests/test_wizard_flows.py`:

```python
import json
import subprocess
import sys
from pathlib import Path

from sdf_fixtures import WORLD

TOOLS = Path(__file__).resolve().parents[1]


def run_wizard(answers, tmp_path):
    gz = tmp_path / "gz"
    (gz / "worlds").mkdir(parents=True)
    (gz / "worlds/if800_survey.sdf").write_text(WORLD)
    env = {"PATH": "/usr/bin:/bin", "HOME": str(tmp_path),
           "HIL_SITES_DIR": str(tmp_path / "sites"),
           "HIL_GAZEBO_DIR": str(gz),
           "HIL_ARDUPILOT_DIR": str(tmp_path / "ap"),
           "HIL_OFFLINE": "1", "HIL_NO_BOARD": "1"}
    return subprocess.run(
        [sys.executable, str(TOOLS / "site_wizard.py")],
        input="\n".join(answers) + "\n", text=True,
        capture_output=True, env=env, timeout=60), tmp_path


def test_sitl_flow_creates_site_and_script(tmp_path):
    answers = [
        "1",                         # new site
        "1",                         # SITL
        "39.58012, -104.84325",      # coords
        "meridian",                  # site name
        "1793",                      # elevation (offline -> manual entry)
        "90",                        # heading
        "n",                         # don't launch
        "1",                         # radio: keep current
        "5",                         # quit
    ]
    result, base = run_wizard(answers, tmp_path)
    assert result.returncode == 0, result.stderr
    site = base / "sites/meridian"
    data = json.loads((site / "site.json").read_text())
    assert data["mode"] == "sitl"
    assert data["lat"] == 39.58012
    assert data["heading"] == 90.0
    script = (site / "launch-sitl.sh").read_text()
    assert "--custom-location=39.58012000,-104.84325000,1793.0,90" in script
    assert (base / "sites/active.txt").read_text().strip() == "meridian"


def test_sih_flow_falls_back_to_manual_steps(tmp_path):
    # No Cube on USB in the test env -> wizard must print the IGC steps.
    answers = [
        "1",                         # new site
        "2",                         # SIH
        "-35.363, 149.165",
        "cmac-sih",
        "584",
        "1",                         # radio: keep current
        "5",
    ]
    result, base = run_wizard(answers, tmp_path)
    assert result.returncode == 0, result.stderr
    assert "SIM_OPOS_LAT" in result.stdout       # manual fallback shown
    assert "SIM_OPOS_ALT = 584" in result.stdout
    assert (base / "sites/cmac-sih/site.json").exists()


def test_show_current_reads_world_and_says_no_active_site(tmp_path):
    answers = ["3", "5"]
    result, _ = run_wizard(answers, tmp_path)
    assert result.returncode == 0, result.stderr
    assert "-35.363016" in result.stdout
    assert "No site has been applied" in result.stdout


def test_crash_is_friendly_without_debug(tmp_path):
    # Feed garbage until EOF: wizard must exit cleanly, never traceback.
    answers = ["1", "3", "not coordinates"]
    result, _ = run_wizard(answers, tmp_path)
    assert "Traceback" not in result.stderr
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest tools/tests/test_wizard_flows.py -v`
Expected: FAIL (site_wizard.py does not exist).

- [ ] **Step 3: Write `tools/site_wizard.py`**

```python
#!/usr/bin/env python3
"""HIL Sim Bench — Flight Site Wizard.

Moves the simulated flight site for SITL / SIH / External-Physics setups.
Run from the repo root:

    python3 tools/site_wizard.py
"""
import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import board
import fetch_ortho
import gazebo_files
import geo
import sites
import ui

BANNER = """
  HIL Sim Bench — Flight Site Wizard
  ──────────────────────────────────"""

SIM_MODES = [
    "SITL — everything on the laptop",
    "SIH — everything on the Cube",
    "External Physics — Cube + Gazebo",
]
MODE_KEYS = ["sitl", "sih", "external"]
MODE_LABEL = {"sitl": "SITL (laptop only)",
              "sih": "SIH (Cube only)",
              "external": "External Physics (Cube + Gazebo)"}
EDGE_MARGIN_M = 30
MISMATCH_WARN_M = 5.0
NO_SIM_REASON = ("the simulator is not running, and the Cube needs its "
                 "physics up before it reboots")


def choose_coords(prompt="coordinates"):
    ui.say("  Paste the coordinates from Google Maps")
    ui.say("  (right-click the spot on the map, then click the numbers to copy)")
    while True:
        try:
            return geo.parse_latlng(ui.ask(prompt))
        except geo.BadCoordinates as err:
            ui.say(f"  {err}")


def choose_elevation(lat, lng, what="the site"):
    """Whole meters, so the world file and SIM_OPOS_ALT cannot disagree."""
    elev = geo.get_elevation(lat, lng)
    if elev is not None:
        if ui.ask_yesno(f"Ground elevation at {what}: {elev:.0f} m — keep?"):
            return round(elev)
    else:
        ui.say("  Elevation lookup unavailable (offline, or VPN blocking it).")
    while True:
        try:
            return round(float(ui.ask("ground elevation in meters")))
        except ValueError:
            ui.say("  Please enter a number, e.g. 584")


def choose_site_name(lat, lng):
    default = sites.slugify(f"site-{lat:.4f}-{lng:.4f}")
    while True:
        name = sites.slugify(ui.ask("short name for this site", default)) or default
        if name not in sites.list_sites():
            return name
        existing = sites.load_site(name)
        ui.say(f"  A site called '{name}' already exists "
               f"({MODE_LABEL[existing['mode']]}, "
               f"{existing['lat']:.5f}, {existing['lng']:.5f}).")
        if ui.ask_yesno(f"Replace the saved '{name}'?", default=False):
            return name
        ui.say("  Pick a different name.")


def sim_opos_params(data):
    return {
        "SIM_OPOS_LAT": data["lat"],
        "SIM_OPOS_LNG": data["lng"],
        "SIM_OPOS_ALT": data["elevation_m"],
        "SIM_OPOS_HDG": 0.0,
    }


def print_manual_param_steps(params):
    ui.say("  Set these yourself in IGC (Vehicle Setup → Parameters,")
    ui.say("  search for each name):")
    for name, value in params.items():
        shown = int(value) if float(value).is_integer() else f"{value:.7f}"
        ui.say(f"    {name} = {shown}")
    ui.say("  Then reboot: Parameters → Tools → Reboot Vehicle.")


def apply_board_params(params, offer_reboot=True, no_reboot_reason=None):
    """Set params over USB. IGC can stay connected — it uses the radio."""
    port = board.find_mavlink_port()
    if port is None:
        ui.say("  Could not find the Cube's USB port — is the cable plugged in?")
        print_manual_param_steps(params)
        return False
    if not ui.ask_yesno(f"Set the Cube's parameters now (over USB, {port})?"):
        print_manual_param_steps(params)
        return False
    try:
        conn = board.connect(port)
    except board.BoardError as err:
        ui.say(f"  {err}")
        print_manual_param_steps(params)
        return False
    try:
        board.set_params(conn, params)
        ui.say("  ✓ Parameters set and verified.")
        if offer_reboot:
            if ui.ask_yesno("Reboot the Cube so they take effect?"):
                board.reboot(conn)
                ui.say("  ✓ Reboot sent — the Cube reconnects on its own.")
        else:
            ui.say(f"  Not rebooting yet: {no_reboot_reason}.")
            ui.say("  Start the simulator, then reboot the Cube from IGC")
            ui.say("  (Parameters → Tools → Reboot Vehicle).")
        return True
    except board.BoardError as err:
        ui.say(f"  {err}")
        print_manual_param_steps(params)
        return False
    finally:
        conn.close()


def radio_prompt():
    idx = ui.ask_choice("Which radio is on the bench?",
                        ["Keep current setup", "Herelink", "Taisync"])
    if idx == 0:
        return
    apply_board_params(board.HERELINK if idx == 1 else board.TAISYNC)


def gazebo_preflight():
    """Nothing may already hold the simulator port: a running Gazebo keeps
    the OLD world in memory, so moving the Cube would leave the drone
    hovering over the wrong ground. Returns False if the operator gives up."""
    if not sites.ppp0_up():
        ui.say("  The USB network link to the Cube (ppp0) is not up.")
        ui.say("  Start pppd first — see the wiki page 'External Physics SIH'.")
        if not ui.ask_yesno("Started it — carry on?", default=False):
            return False
    while sites.sim_port_busy():
        ui.say("  A simulator is already running on this laptop (it holds")
        ui.say(f"  physics port {sites.SIM_UDP_PORT}). Close that Gazebo window")
        ui.say("  (Ctrl+C in its terminal) — otherwise it would keep flying the")
        ui.say("  old site while the Cube moves to the new one.")
        if not ui.ask_yesno("Closed it — check again?"):
            return False
    return True


def launch_gazebo(name):
    """Launch, then wait until the simulator is really accepting the board's
    physics link. Returns True only if it came up."""
    where = sites.launch_script(sites.site_dir(name) / "launch-gazebo.sh",
                               sites.site_dir(name) / "gazebo.log")
    ui.say(f"  Started in {where}.")
    ui.say("  Waiting for the simulator to come up...")
    if sites.wait_for_sim_port():
        ui.say("  ✓ Simulator is up and waiting for the Cube.")
        return True
    ui.say("  ✗ The simulator did not come up.")
    ui.say(f"    Check its window, or {sites.site_dir(name) / 'gazebo.log'}.")
    return False


def choose_spawn(meta):
    if ui.ask_yesno("Start the drone at the center of the area?"):
        return 0.0, 0.0
    while True:
        pin_lat, pin_lng = choose_coords("start-point coordinates")
        east, north = geo.pin_to_offset(pin_lat, pin_lng, meta)
        if (abs(east) <= meta["size_ew_m"] / 2 - EDGE_MARGIN_M
                and abs(north) <= meta["size_ns_m"] / 2 - EDGE_MARGIN_M):
            return east, north
        ui.say("  That point is outside the imagery (or too close to its "
               "edge) — pick another.")


def external_flow(name, data):
    if not gazebo_preflight():
        ui.say("  Nothing was changed.")
        return False

    idx = ui.ask_choice("How much area do you need around the site?",
                        ["Standard — about 560 m square",
                         "Large — about 1100 m square"])
    target_m = 560 if idx == 0 else 1100
    ui.say("  Checking what imagery detail is available here...")
    zoom, nside = fetch_ortho.pick_zoom_nside(data["lat"], data["lng"],
                                              target_m)

    ortho = sites.site_dir(name) / "ortho.jpg"
    ortho.parent.mkdir(parents=True, exist_ok=True)
    ui.say("  Downloading imagery (can take a minute)...")
    meta = fetch_ortho.fetch_mosaic(
        data["lat"], data["lng"], zoom, nside, str(ortho),
        progress=lambda j, n: ui.say(f"    {j}/{n}"))
    ui.say(f"  ✓ Got {meta['size_ew_m']:.0f} x {meta['size_ns_m']:.0f} m of "
           f"real imagery ({meta['gsd_m'] * 100:.0f} cm/pixel).")

    # The site is the imagery center, not the clicked pin — they can be a
    # whole tile apart, so the elevation has to be looked up again here or
    # the ground ends up at the wrong height on sloping terrain.
    data["lat"], data["lng"] = meta["center_lat"], meta["center_lng"]
    data["mosaic"] = meta
    data["elevation_m"] = choose_elevation(data["lat"], data["lng"],
                                           what="the center of the area")

    if ui.ask_yesno("Open the picture to check it's the right place?",
                    default=False):
        subprocess.Popen(["xdg-open", str(ortho)])

    data["spawn_east"], data["spawn_north"] = choose_spawn(meta)

    gazebo_files.apply_site(sites.GAZEBO_DIR,
                            {**meta, "elevation_m": data["elevation_m"]},
                            data["spawn_east"], data["spawn_north"],
                            texture_src=ortho)
    ui.say("  ✓ Gazebo world updated.")

    sites.write_launch_scripts(name, data)
    sites.save_site(name, data)
    sites.set_active(name)

    ready = False
    if ui.ask_yesno("Launch Gazebo at the new site now?"):
        ready = launch_gazebo(name)
    else:
        ui.say("  Gazebo not started. The Cube has no physics until you run")
        ui.say(f"  {sites.site_dir(name) / 'launch-gazebo.sh'}.")
    apply_board_params(sim_opos_params(data), offer_reboot=ready,
                       no_reboot_reason=NO_SIM_REASON)
    return True


def sitl_flow(name, data):
    while True:
        try:
            data["heading"] = float(
                ui.ask("drone starting heading in degrees (0 = facing north)",
                       "0"))
            break
        except ValueError:
            ui.say("  Please enter a number, e.g. 0")
    script = sites.write_launch_scripts(name, data)
    sites.save_site(name, data)
    sites.set_active(name)
    ui.say(f"  ✓ Saved. Relaunch any time with: {script}")
    if ui.ask_yesno("Launch SITL now?"):
        ui.say("  Started in " + sites.launch_script(
            script, sites.site_dir(name) / "sitl.log"))
    return True


def sih_flow(name, data):
    sites.save_site(name, data)
    sites.set_active(name)
    apply_board_params(sim_opos_params(data))
    return True


def new_site():
    mode = MODE_KEYS[ui.ask_choice("Which setup are you using?", SIM_MODES,
                                   default_idx=2)]
    lat, lng = choose_coords()
    while mode == "external" and abs(lat) > fetch_ortho.MERCATOR_LAT_LIMIT:
        ui.say("  Satellite imagery only exists up to 85 degrees north and")
        ui.say("  south. Pick a site farther from the poles (SITL needs no")
        ui.say("  imagery and works anywhere).")
        lat, lng = choose_coords()
    name = choose_site_name(lat, lng)
    data = {"name": name, "mode": mode, "lat": lat, "lng": lng,
            "elevation_m": None, "heading": 0.0,
            "spawn_east": 0.0, "spawn_north": 0.0, "mosaic": None,
            "pin_lat": lat, "pin_lng": lng}
    if mode != "external":
        data["elevation_m"] = choose_elevation(lat, lng)
    flow = {"sitl": sitl_flow, "sih": sih_flow, "external": external_flow}[mode]
    if not flow(name, data):
        return
    radio_prompt()
    ui.say("\n  Done — this site is saved. Next time, pick it from "
           "'Switch to a saved site'.")


def use_saved_site():
    names = sites.list_sites()
    if not names:
        ui.say("  No saved sites yet — set up a new site first.")
        return
    name = names[ui.ask_choice("Which site?", names)]
    data = sites.load_site(name)
    if data["mode"] == "external":
        if not gazebo_preflight():
            ui.say("  Nothing was changed.")
            return
        ortho = sites.site_dir(name) / "ortho.jpg"
        if not ortho.exists():
            ui.say("  This site's imagery is missing — downloading again...")
            meta = data["mosaic"]
            fetch_ortho.fetch_mosaic(meta["center_lat"], meta["center_lng"],
                                     meta["zoom"], meta["px"] // 256,
                                     str(ortho),
                                     progress=lambda j, n: ui.say(f"    {j}/{n}"))
        gazebo_files.apply_site(sites.GAZEBO_DIR,
                                {**data["mosaic"],
                                 "elevation_m": data["elevation_m"]},
                                data["spawn_east"], data["spawn_north"],
                                texture_src=ortho)
        ui.say("  ✓ Gazebo world switched to this site.")
        sites.set_active(name)
        ready = launch_gazebo(name) if ui.ask_yesno("Launch Gazebo now?") else False
        apply_board_params(sim_opos_params(data), offer_reboot=ready,
                           no_reboot_reason=NO_SIM_REASON)
    elif data["mode"] == "sih":
        sites.set_active(name)
        apply_board_params(sim_opos_params(data))
    else:
        sites.set_active(name)
        if ui.ask_yesno("Launch SITL now?"):
            ui.say("  Started in " + sites.launch_script(
                sites.site_dir(name) / "launch-sitl.sh",
                sites.site_dir(name) / "sitl.log"))
    radio_prompt()


def read_cube_position():
    port = board.find_mavlink_port()
    if port is None:
        ui.say("  (Cube not connected over USB.)")
        return None
    try:
        conn = board.connect(port, timeout=5)
        try:
            return (board.get_param(conn, "SIM_OPOS_LAT"),
                    board.get_param(conn, "SIM_OPOS_LNG"))
        finally:
            conn.close()
    except board.BoardError as err:
        ui.say(f"  (Could not read the Cube: {err})")
        return None


def warn_mismatch(active, world, cube):
    """The 'drone on a grey void' failure: whichever of these apply to the
    active mode must agree."""
    if not active:
        return
    places = []
    if active["mode"] == "external":
        places.append(("the saved site", active["lat"], active["lng"]))
        if world:
            places.append(("the Gazebo world", world[0], world[1]))
    elif active["mode"] == "sih":
        places.append(("the saved site", active["lat"], active["lng"]))
    if cube and active["mode"] in ("sih", "external"):
        places.append(("the Cube", cube[0], cube[1]))
    for i in range(len(places)):
        for j in range(i + 1, len(places)):
            (a, alat, alng), (b, blat, blng) = places[i], places[j]
            d = geo.distance_m(alat, alng, blat, blng)
            if d > MISMATCH_WARN_M:
                shown = f"{d / 1000:.0f} km" if d >= 1000 else f"{d:.0f} m"
                ui.say(f"  ! {a} and {b} are {shown} apart — re-apply the site")
                ui.say("    ('Switch to a saved site') before flying.")


def show_current():
    name = sites.get_active()
    active = sites.load_site(name) if name else None
    if active:
        ui.say(f"  Active site:   {name} — {MODE_LABEL[active['mode']]}")
        ui.say(f"                 {active['lat']:.6f}, {active['lng']:.6f} "
               f"(ground {active['elevation_m']:.0f} m)")
        ui.say("                 Map: https://www.google.com/maps?q="
               f"{active['lat']:.6f},{active['lng']:.6f}")
    else:
        ui.say("  No site has been applied by this wizard yet.")

    world = gazebo_files.read_current(sites.GAZEBO_DIR)
    if world is None:
        ui.say("  Gazebo world files not found — External Physics is not "
               "set up on this laptop.")
    else:
        ui.say(f"  Gazebo world:  {world[0]:.6f}, {world[1]:.6f} "
               f"(ground {world[2]:.0f} m)")

    cube = read_cube_position()
    if cube:
        ui.say(f"  Cube:          {cube[0]:.6f}, {cube[1]:.6f}")
    warn_mismatch(active, world, cube)


def run_menu():
    ui.say(BANNER)
    actions = ["Set up a new site", "Switch to a saved site",
               "Show current site", "Radio setup (Herelink / Taisync)",
               "Quit"]
    while True:
        idx = ui.ask_choice("What do you want to do?", actions)
        if idx == 0:
            new_site()
        elif idx == 1:
            use_saved_site()
        elif idx == 2:
            show_current()
        elif idx == 3:
            radio_prompt()
        else:
            return
        ui.say("")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--debug", action="store_true",
                        help="show full error details (for developers)")
    args = parser.parse_args()
    try:
        run_menu()
    except KeyboardInterrupt:
        ui.say("\n  Cancelled.")
    except Exception as err:  # operator-facing catch-all
        if args.debug:
            raise
        ui.say(f"\n  Something went wrong: {err}")
        ui.say("  Try again — or run 'python3 tools/site_wizard.py --debug' "
               "and send the output to a developer.")
        sys.exit(1)


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run the tests**

Run: `python3 -m pytest tools/tests/test_wizard_flows.py -v`
Expected: 4 PASS.

- [ ] **Step 5: Commit**

```bash
git add tools/site_wizard.py tools/tests/test_wizard_flows.py
git commit -m "tools: interactive flight-site wizard"
```

---

### Task 9: External-Physics and state tests (in-process)

**Files:**
- Test: `tools/tests/test_wizard_external.py`

**Interfaces:**
- Consumes: `site_wizard` as an importable module. These run in-process with monkeypatched imagery, board, launcher and elevation — a subprocess can't mock a network, and adding a "fake tiles" env-var backdoor to production code to make it testable would be worse than the bug it guards.
- Covers the External-Physics path end to end plus the states `show_current` has to distinguish. Task 10 (live bench) is validation, not a substitute for this.

- [ ] **Step 1: Write the failing tests**

`tools/tests/test_wizard_external.py`:

```python
import builtins
import io
import json
import types
from pathlib import Path

import pytest
from PIL import Image

import board
import fetch_ortho
import sites
import site_wizard
from sdf_fixtures import MODEL, WORLD, physics_block


def answers(monkeypatch, *values):
    it = iter(values)
    monkeypatch.setattr(builtins, "input", lambda _="": next(it))


class FakeResp:
    status_code = 200

    def __init__(self):
        buf = io.BytesIO()
        Image.new("RGB", (256, 256), (10, 120, 10)).save(buf, "JPEG")
        self.content = buf.getvalue()

    def raise_for_status(self):
        pass


class FakeSession:
    def get(self, url, timeout=30):
        return FakeResp()


@pytest.fixture
def bench(tmp_path, monkeypatch):
    gz = tmp_path / "gz"
    (gz / "worlds").mkdir(parents=True)
    (gz / "models/survey_ground/materials/textures").mkdir(parents=True)
    (gz / "worlds/if800_survey.sdf").write_text(WORLD)
    (gz / "models/survey_ground/model.sdf").write_text(MODEL)
    monkeypatch.setattr(sites, "GAZEBO_DIR", gz)
    monkeypatch.setattr(sites, "SITES_DIR", tmp_path / "sites")
    monkeypatch.setattr(sites, "ARDUPILOT_DIR", tmp_path / "ap")
    monkeypatch.setattr(board, "find_mavlink_port", lambda: None)
    monkeypatch.setattr(fetch_ortho.requests, "Session", FakeSession)
    monkeypatch.setattr(fetch_ortho, "zoom_available", lambda *a, **k: True)
    monkeypatch.setattr(sites, "ppp0_up", lambda *a, **k: True)
    monkeypatch.setattr(sites, "sim_port_busy", lambda *a, **k: False)
    monkeypatch.setattr(sites, "wait_for_sim_port", lambda *a, **k: True)
    monkeypatch.setattr(site_wizard.geo, "get_elevation",
                        lambda lat, lng, **k: 1793.0)
    launched = []
    monkeypatch.setattr(
        sites, "launch_script",
        lambda script, log: launched.append(Path(script).name) or "a test")
    return types.SimpleNamespace(gz=gz, base=tmp_path, launched=launched)


def fake_board(monkeypatch):
    """A Cube that answers, so the reboot decision is reachable."""
    store, rebooted = {}, []

    class Conn:
        def close(self):
            pass
    monkeypatch.setattr(board, "find_mavlink_port", lambda: "/dev/fake")
    monkeypatch.setattr(board, "connect", lambda port, timeout=10: Conn())
    monkeypatch.setattr(board, "set_params",
                        lambda conn, params, **k: store.update(params) or dict(params))
    monkeypatch.setattr(board, "reboot", lambda conn: rebooted.append(True))
    return store, rebooted


def test_external_new_site_end_to_end(bench, monkeypatch):
    before = (bench.gz / "worlds/if800_survey.sdf").read_text()
    answers(monkeypatch,
            "3",                        # External Physics
            "39.58012, -104.84325",     # coordinates
            "meridian",                 # site name
            "1",                        # area: standard
            "y",                        # keep the looked-up elevation
            "n",                        # don't open the picture
            "n",                        # not the center — pick a start point
            "39.5805, -104.8430",       # start point
            "y",                        # launch Gazebo
            "1")                        # radio: keep current
    site_wizard.new_site()

    data = json.loads((bench.base / "sites/meridian/site.json").read_text())
    assert data["mode"] == "external"
    # the site is the mosaic center, and the clicked pin is kept separately
    assert data["lat"] == data["mosaic"]["center_lat"]
    assert data["lng"] == data["mosaic"]["center_lng"]
    assert data["pin_lat"] == 39.58012
    assert data["spawn_east"] != 0.0

    world = (bench.gz / "worlds/if800_survey.sdf").read_text()
    assert f"<latitude_deg>{data['lat']:.8f}</latitude_deg>" in world
    assert f"<longitude_deg>{data['lng']:.8f}</longitude_deg>" in world
    assert "<elevation>1793</elevation>" in world
    assert (f'<pose degrees="true">{data["spawn_east"]:.1f} '
            f'{data["spawn_north"]:.1f} 0.195 0 0 90</pose>') in world
    assert physics_block(world) == physics_block(before)   # never touched

    model = (bench.gz / "models/survey_ground/model.sdf").read_text()
    assert model.count(f"<size>{data['mosaic']['size_ew_m']:.2f} "
                       f"{data['mosaic']['size_ns_m']:.2f}</size>") == 2

    tex = bench.gz / "models/survey_ground/materials/textures/survey_ortho.jpg"
    assert tex.read_bytes() == (bench.base / "sites/meridian/ortho.jpg").read_bytes()
    assert (bench.base / "sites/meridian/launch-gazebo.sh").exists()
    assert bench.launched == ["launch-gazebo.sh"]
    assert sites.get_active() == "meridian"


def test_elevation_is_looked_up_at_the_mosaic_center(bench, monkeypatch):
    seen = []

    def fake_elev(lat, lng, **k):
        seen.append((lat, lng))
        return 1800.0
    monkeypatch.setattr(site_wizard.geo, "get_elevation", fake_elev)
    answers(monkeypatch, "3", "39.58012, -104.84325", "meridian", "1",
            "y", "n", "y", "n", "1")
    site_wizard.new_site()
    data = json.loads((bench.base / "sites/meridian/site.json").read_text())
    # exactly one lookup, at the center — not at the clicked pin
    assert seen == [(data["mosaic"]["center_lat"], data["mosaic"]["center_lng"])]
    assert (bench.base / "sites/meridian/site.json").exists()
    world = (bench.gz / "worlds/if800_survey.sdf").read_text()
    assert "<elevation>1800</elevation>" in world
    assert data["elevation_m"] == 1800


def test_a_running_gazebo_blocks_the_whole_change(bench, monkeypatch, capsys):
    monkeypatch.setattr(sites, "sim_port_busy", lambda *a, **k: True)
    store, rebooted = fake_board(monkeypatch)
    before = (bench.gz / "worlds/if800_survey.sdf").read_text()
    answers(monkeypatch, "3", "39.58012, -104.84325", "meridian",
            "n")                      # give up rather than close it
    site_wizard.new_site()
    out = capsys.readouterr().out
    assert "already running" in out
    assert (bench.gz / "worlds/if800_survey.sdf").read_text() == before
    assert store == {}                # the Cube was never moved
    assert rebooted == []
    assert not (bench.base / "sites/meridian/site.json").exists()


def test_declining_the_launch_defers_the_reboot(bench, monkeypatch, capsys):
    store, rebooted = fake_board(monkeypatch)
    answers(monkeypatch, "3", "39.58012, -104.84325", "meridian", "1",
            "y", "n", "y",
            "n",                      # do NOT launch Gazebo
            "y",                      # yes, set the Cube's parameters
            "1")
    site_wizard.new_site()
    out = capsys.readouterr().out
    assert "Not rebooting yet" in out
    assert rebooted == []
    assert store["SIM_OPOS_HDG"] == 0.0


def test_a_gazebo_that_never_comes_up_defers_the_reboot(bench, monkeypatch,
                                                       capsys):
    monkeypatch.setattr(sites, "wait_for_sim_port", lambda *a, **k: False)
    store, rebooted = fake_board(monkeypatch)
    answers(monkeypatch, "3", "39.58012, -104.84325", "meridian", "1",
            "y", "n", "y",
            "y",                      # launch it (it fails to come up)
            "y",                      # set params
            "1")
    site_wizard.new_site()
    out = capsys.readouterr().out
    assert "did not come up" in out
    assert "Not rebooting yet" in out
    assert rebooted == []


def test_spawn_outside_the_imagery_is_rejected(bench, monkeypatch, capsys):
    answers(monkeypatch, "3", "39.58012, -104.84325", "meridian", "1",
            "y", "n",
            "n", "39.6500, -104.8430",   # 8 km north — outside
            "39.5805, -104.8430",        # inside
            "n", "1")
    site_wizard.new_site()
    out = capsys.readouterr().out
    assert "outside the imagery" in out
    data = json.loads((bench.base / "sites/meridian/site.json").read_text())
    assert abs(data["spawn_north"]) < data["mosaic"]["size_ns_m"] / 2


def test_site_name_collision_asks_before_replacing(bench, monkeypatch):
    answers(monkeypatch,
            "1", "39.58012, -104.84325", "meridian", "y", "0", "n", "1")
    site_wizard.new_site()                       # first SITL site
    answers(monkeypatch,
            "1", "39.60000, -104.80000",
            "meridian",          # taken
            "n",                 # don't replace
            "meridian-2",        # a different name
            "y", "0", "n", "1")
    site_wizard.new_site()
    assert sorted(sites.list_sites()) == ["meridian", "meridian-2"]


def test_show_current_reports_the_active_external_site_and_mismatch(
        bench, monkeypatch, capsys):
    answers(monkeypatch, "3", "39.58012, -104.84325", "meridian", "1",
            "y", "n", "y", "n", "1")
    site_wizard.new_site()
    # move the world out from under the saved site
    world_path = bench.gz / "worlds/if800_survey.sdf"
    world_path.write_text(world_path.read_text().replace(
        "<latitude_deg>39.", "<latitude_deg>38."))
    capsys.readouterr()
    site_wizard.show_current()
    out = capsys.readouterr().out
    assert "meridian" in out
    assert "External Physics" in out
    assert "apart" in out


def test_show_current_for_sitl_does_not_warn_about_gazebo(bench, monkeypatch,
                                                          capsys):
    answers(monkeypatch, "1", "39.58012, -104.84325", "desk", "y", "0", "n", "1")
    site_wizard.new_site()
    capsys.readouterr()
    site_wizard.show_current()
    out = capsys.readouterr().out
    assert "desk" in out
    assert "SITL" in out
    assert "apart" not in out


def test_show_current_for_sih_reports_the_site(bench, monkeypatch, capsys):
    answers(monkeypatch, "2", "-35.363, 149.165", "cmac-sih", "y", "1")
    site_wizard.new_site()
    capsys.readouterr()
    site_wizard.show_current()
    out = capsys.readouterr().out
    assert "cmac-sih" in out
    assert "SIH" in out


def test_polar_site_is_refused_for_external_physics(bench, monkeypatch,
                                                    capsys):
    answers(monkeypatch, "3",
            "87.0, 20.0",                 # beyond the Mercator limit
            "39.58012, -104.84325",       # then a real one
            "meridian", "1", "y", "n", "y", "n", "1")
    site_wizard.new_site()
    out = capsys.readouterr().out
    assert "85 degrees" in out
    assert (bench.base / "sites/meridian/site.json").exists()
```

- [ ] **Step 2: Run them against the Task 8 wizard**

Run: `python3 -m pytest tools/tests/test_wizard_external.py -v`

Unlike the earlier tasks these are not expected to start red: they describe
behavior Task 8 was already built to have. Record the real result. Every
test that passes confirms Task 8; every test that fails is a defect in
`site_wizard.py`.

- [ ] **Step 3: Fix the wizard for any failures**

Change `tools/site_wizard.py` only. Never edit an assertion to match the
code's current behavior, and do not add test-only branches, env-var
backdoors, or "if testing" checks to production code. If a test looks wrong
to you, stop and report it rather than adjusting it.

- [ ] **Step 4: Run the whole suite**

Run: `python3 -m pytest tools/tests -v`
Expected: all PASS, no network access, no `/dev/serial` access.

- [ ] **Step 5: Commit**

```bash
git add tools/tests/test_wizard_external.py tools/site_wizard.py
git commit -m "tools: External-Physics and current-site tests for the wizard"
```

---

### Task 10: Live bench verification (CHECKPOINT — needs the user)

**Files:** none (verification only). Do NOT check these boxes without actually doing them on the bench.

**What "idempotent" can and cannot mean here:** the current bench world holds `-35.36301596, 149.16515350`, which is exactly the z19/9-tile block center for CMAC, and `561.48 x 559.14` matches that block — so today's files came from this same math. But the wizard now prefers zoom 20 when the server has it, and the z20/18-tile block center for CMAC is **44 m away** from the z19 one. A first run may therefore legitimately move the coordinates. The real invariant is: *two identical runs produce identical files.*

- [ ] **Step 1: Ask the user for go-ahead** — this task restarts the bench Gazebo and writes params to the live Cube.

- [ ] **Step 2: Baseline**

```bash
cd ~/git/IFT/HIL-Sim-Bench
cp ~/git/ardupilot_gazebo/worlds/if800_survey.sdf /tmp/world.before
cp ~/git/ardupilot_gazebo/models/survey_ground/model.sdf /tmp/model.before
```

- [ ] **Step 3: The running-Gazebo guard (do this first, while a sim is up)**

With the bench Gazebo already running, start the wizard and choose a new
External-Physics site. Expected: it stops at the preflight, names port 9002,
and changes nothing — `diff /tmp/world.before ~/git/ardupilot_gazebo/worlds/if800_survey.sdf`
is empty and the Cube's `SIM_OPOS_LAT` is unchanged. Then close Gazebo.

- [ ] **Step 4: New-site run at CMAC**

```bash
python3 tools/site_wizard.py
# answers: new site -> External Physics -> "-35.36301596, 149.16515350"
#          -> name "cmac" -> standard area -> keep elevation -> center spawn
#          -> launch Gazebo YES -> set params YES -> reboot YES -> radio: keep
cp ~/git/ardupilot_gazebo/worlds/if800_survey.sdf /tmp/world.run1
diff /tmp/world.before /tmp/world.run1
```

Expected: the wizard waits for the simulator and reports it is up before
offering params. The diff shows **only**: possibly `<latitude_deg>`/
`<longitude_deg>`/plane `<size>` if zoom 20 was selected (record which zoom
the wizard reported), possibly `<elevation>`, and the spawn pose reformatted
from `0 0 0.195 0 0 90` to `0.0 0.0 0.195 0 0 90`. The `<physics>` block must
be byte-identical. Board reconnects; statustext shows `SIM rate 200/200Hz
RTF ~1.00` within ~30 s of reboot.

- [ ] **Step 5: Idempotence**

Close Gazebo, rerun the wizard with the same answers, then
`diff /tmp/world.run1 ~/git/ardupilot_gazebo/worlds/if800_survey.sdf`.
Expected: empty. Also confirm `worlds/if800_survey.sdf.orig` still matches
`/tmp/world.before`.

- [ ] **Step 6: Saved-site switch** — close Gazebo, rerun the wizard, "Switch to a saved site" → `cmac`. Expected: no re-download, world re-applied, launch waits for readiness, radio prompt appears.

- [ ] **Step 7: Show current site** — menu 3. Expected: active site `cmac` with its mode, matching Gazebo-world and Cube coordinates, and no mismatch warning.

- [ ] **Step 8: Radio read-back only** — menu 4 → check current values first via menu 3 / IGC; choose the radio actually on the bench so values are written but unchanged. Expected: "Parameters set and verified."

- [ ] **Step 9: Record results** — paste the observed statustext line, the selected zoom, and any deviations into the session notes; fix anything broken before Task 11.

---

### Task 11: Wiki page + link edits

**Files:**
- Create: `~/git/IFT/HIL-Sim-Bench.wiki/Changing-the-Flight-Site.md`
- Modify: `HIL-Software-Overview.md`, `External-Physics-SIH.md`, `SITL-Instructions.md`, `SIH-Instructions.md` (same wiki repo)

- [ ] **Step 1: Get the wiki, without clobbering it**

The clone already exists at `~/git/IFT/HIL-Sim-Bench.wiki` (verified 2026-07-29:
clean tree, remote `git@github.com:Inspired-Flight-Technologies/HIL-Sim-Bench.wiki.git`).
Cloning unconditionally would fail.

```bash
WIKI=~/git/IFT/HIL-Sim-Bench.wiki
if [ -d "$WIKI/.git" ]; then
  cd "$WIKI"
  git remote -v | grep -q 'HIL-Sim-Bench.wiki' || { echo "unexpected remote — stop"; exit 1; }
  test -z "$(git status --porcelain)" || { echo "uncommitted changes — stop and ask"; exit 1; }
  git pull --ff-only
else
  git clone git@github.com:Inspired-Flight-Technologies/HIL-Sim-Bench.wiki.git "$WIKI"
fi
```

- [ ] **Step 2: Write `Changing-the-Flight-Site.md`**

````markdown
# Changing the Flight Site

The simulator can fly almost anywhere on Earth. The **Flight Site Wizard**
moves it for you — no file editing, no parameter typing.

## What you need

* The bench laptop (internet on, VPN off — it can block the imagery download)
* For SIH / External Physics: the Cube connected by USB
* For External Physics: the USB network link (ppp0) up, and **no other Gazebo
  running** — the wizard will tell you if one is
* IGC can stay open and connected the whole time — it talks over the radio

## 1. Copy the coordinates from Google Maps

1. Open [Google Maps](https://maps.google.com) and find the place to fly.
2. **Right-click** the exact spot.
3. Click the numbers at the top of the menu (like `39.58012, -104.84325`) —
   that copies them.

## 2. Run the wizard

```bash
cd ~/git/IFT/HIL-Sim-Bench
python3 tools/site_wizard.py
```

Pick **Set up a new site**, answer the questions (paste the coordinates when
asked), and say yes when it offers to set the Cube's parameters and launch the
simulator. That's it.

The wizard saves every site. Next time, pick **Switch to a saved site** —
it's instant, no download.

## Example: moving External Physics to a new site

```
  HIL Sim Bench — Flight Site Wizard
  ──────────────────────────────────
  What do you want to do?
    1) Set up a new site
    2) Switch to a saved site
    3) Show current site
    4) Radio setup (Herelink / Taisync)
    5) Quit
  > [1] 1

  Which setup are you using?
    1) SITL — everything on the laptop
    2) SIH — everything on the Cube
    3) External Physics — Cube + Gazebo  [default]
  > [3]                                   ← Enter takes the default

  Paste the coordinates from Google Maps
  (right-click the spot on the map, then click the numbers to copy)
  coordinates > 39.58012, -104.84325      ← pasted from Google Maps

  short name for this site [site-39-5801-104-8433] > meridian

  How much area do you need around the site?
    1) Standard — about 560 m square  [default]
    2) Large — about 1100 m square
  > [1]

  Checking what imagery detail is available here...
  Downloading imagery (can take a minute)...
    1/19
    ...
    19/19
  ✓ Got 560 x 559 m of real imagery (12 cm/pixel).

  Ground elevation at the center of the area: 1793 m — keep? [Y/n] >
                                          ← looked up at the imagery center,
                                            not where you clicked
  Open the picture to check it's the right place? [y/N] > y

  Start the drone at the center of the area? [Y/n] >
  ✓ Gazebo world updated.

  Launch Gazebo at the new site now? [Y/n] >
  Started in a new terminal window (launch-gazebo.sh).
  Waiting for the simulator to come up...
  ✓ Simulator is up and waiting for the Cube.
                                          ← only now does it touch the Cube
  Set the Cube's parameters now (over USB, /dev/serial/by-id/usb-...-if00)? [Y/n] >
  ✓ Parameters set and verified.
  Reboot the Cube so they take effect? [Y/n] >
  ✓ Reboot sent — the Cube reconnects on its own.

  Which radio is on the bench?
    1) Keep current setup  [default]
    2) Herelink
    3) Taisync
  > [1]

  Done — this site is saved. Next time, pick it from 'Switch to a saved site'.
```

Watch IGC after the reboot: within about 30 seconds it should report
`SIM rate 200/200Hz RTF ~1.00`. That means the Cube and Gazebo are in step.

## Radios

The wizard's **Radio setup** menu configures the telemetry radio. It sets:

| Radio    | SERIAL1_BAUD | SYSID_MYGCS |
|----------|--------------|-------------|
| Herelink | 57 (57600)   | 255         |
| Taisync  | 115 (115200) | 19          |

## If something goes wrong

| What you see | What to do |
|---|---|
| Drone sits on a grey/empty ground in Gazebo | The Cube's location doesn't match the world. Run the wizard → **Show current site**; if it warns that things are apart, use **Switch to a saved site**, pick the site again, and say yes to setting parameters. |
| "A simulator is already running" | Close the old Gazebo window (Ctrl+C in its terminal) and answer yes to check again. The wizard stops here on purpose: a running Gazebo keeps the old site. |
| "The simulator did not come up" | Look at the Gazebo window (or the `gazebo.log` path it prints). The usual causes are ppp0 not up and another simulator still running. The wizard will not reboot the Cube into a dead simulator. |
| "imagery download failed" | The laptop is offline or the VPN is blocking it (see [[Home]]). Turn the VPN off and try again. |
| "no heartbeat from the Cube" | Check the USB cable. Or set the values the wizard printed yourself in IGC (it prints exact steps). |
| "satellite imagery only covers up to 85 degrees" | There is no satellite imagery that far north or south. Pick a site farther from the poles, or use SITL, which needs no imagery. |
| The wizard crashes | Run `python3 tools/site_wizard.py --debug` and send the output to a developer. |
````

- [ ] **Step 3: Add links + fix the param name**

In `HIL-Software-Overview.md`, append to the end of the "When to use SITL vs SIH" section:

```markdown
To move any of the three setups to a different real-world location, use the
Flight Site Wizard — see [[Changing the Flight Site]].
```

In `External-Physics-SIH.md`, append to the end of the "Overview" section:

```markdown
To choose *where* it flies, see [[Changing the Flight Site]].
```

In `SITL-Instructions.md`, replace `**Page still under construction**` with:

```markdown
To choose where SITL flies — and to launch it — use the Flight Site Wizard:
see [[Changing the Flight Site]].
```

In `SIH-Instructions.md`, replace both occurrences of `SYSID_MY_GCS` with `SYSID_MYGCS` (the real parameter name — the old spelling doesn't exist in the firmware).

- [ ] **Step 4: Review the diff, then push**

```bash
cd ~/git/IFT/HIL-Sim-Bench.wiki
git add -A && git diff --cached
git commit -m "Add Changing the Flight Site page; link from overview pages; fix SYSID_MYGCS"
git push
```

- [ ] **Step 5: Verify** — open https://github.com/Inspired-Flight-Technologies/HIL-Sim-Bench/wiki/Changing-the-Flight-Site in a browser (or re-fetch with git) and confirm it renders, including the example session block.

---

### Task 12: IFT-ardupilot pointer updates

**Files:**
- Modify: `~/git/IFT/IFT-ardupilot/docs/hitl-instructions.md` (§2.6.1 retexture block)
- Delete: `~/git/IFT/IFT-ardupilot/docs/hitl-tools/fetch_ortho.py`

- [ ] **Step 1: Update §2.6.1**

In `docs/hitl-instructions.md`, replace the paragraph + code block that begins `To rebuild the texture for the **real survey site**:` (through the sentence ending `center = SIM_OPOS).`) with:

```markdown
To move the site: operators use the interactive **Flight Site Wizard** —
`python3 tools/site_wizard.py` in the HIL-Sim-Bench repo (wiki page
"Changing the Flight Site"). It downloads imagery, rewrites this world's
`<spherical_coordinates>`/plane sizes/spawn pose, sets `SIM_OPOS_*` over USB,
and generates per-site launch scripts. The imagery fetcher lives there too
(`tools/fetch_ortho.py`, moved from this repo); its output JSON carries the
same registration values described above. Note the wizard keeps `.orig` and
`.bak` copies beside the world, model and texture — nothing under
ardupilot_gazebo is tracked by git. A 1:1 drone orthophoto of the site still
beats Esri imagery if available — same registration rules (north-up image,
plane size = ground extent, center = SIM_OPOS).
```

- [ ] **Step 2: Remove the moved script**

```bash
cd ~/git/IFT/IFT-ardupilot
git rm docs/hitl-tools/fetch_ortho.py
```

- [ ] **Step 3: Commit**

```bash
git add docs/hitl-instructions.md
git commit -m "docs/hitl: fetch_ortho + site tooling moved to HIL-Sim-Bench"
```

- [ ] **Step 4: Final check** — `grep -rn "hitl-tools" docs/hitl-instructions.md` returns nothing; full wizard test suite green: `cd ~/git/IFT/HIL-Sim-Bench && python3 -m pytest tools/tests -v`.

---

## Review Resolution

Every finding from the review was checked against the bench before acting.
Verification notes are recorded here so the next reader doesn't have to redo them.

| # | Finding | Verdict | Where it's addressed |
|---|---------|---------|----------------------|
| 1 | Gazebo launch failure leaves Cube and simulator at different sites | **Confirmed** | Task 7 (`sim_port_busy`, `wait_for_sim_port`, `ppp0_up`), Task 8 (`gazebo_preflight`, `launch_gazebo`, reboot gating), Task 9 tests, Task 10 Step 3 |
| 2 | External Physics absent from wizard-flow tests | **Confirmed** | Task 9 (new in-process test module) |
| 3 | "Show current site" doesn't show the active saved site | **Confirmed** — the spec explicitly says it "reads `site.json` state" | Task 7 (`set_active`/`get_active`), Task 8 (`show_current`, `warn_mismatch`), Task 9 tests |
| 4 | Elevation taken at the clicked pin, not the mosaic center | **Confirmed** — offset is up to 15 m (z20, odd nside) to 62 m (z19, even nside) | Task 8 (`choose_elevation` moved into `external_flow`; whole-meter rounding) |
| 5 | Param verification tolerance too loose | **Confirmed, numbers exact** — `abs(v)*1e-6` allows 9.00 m at Meridian and 13.55 m at CMAC, while real float32 quantization costs 0.19–0.69 m | Task 6 (`_f32` comparison, `_param_name`, transport wrapping) |
| 6 | `.bak` files not reliably recoverable | **Confirmed, and worse than stated** — `worlds/if800_survey.sdf` and `models/survey_ground/` are *untracked* in ardupilot_gazebo, so there is no `git checkout` fallback | Task 5 (validate-then-write, one-time `.orig`, atomic replace, physics-block assertion) |
| 7 | Reusing a site name silently overwrites | **Confirmed** for the collision itself | Task 8 (`choose_site_name`), Task 9 test |
| 8 | "Anywhere on Earth" exceeds Web Mercator handling | **Confirmed** — lat 85.06 yields tile y = −149.9, i.e. off the map | Task 4 (Mercator guard, X wrap, Y validation, tile decoding), Task 3 (`wrap_lng_delta`), Task 8 (re-prompt) |
| 9 | Wiki clone step not idempotent | **Confirmed** — `~/git/IFT/HIL-Sim-Bench.wiki` exists, clean, correct remote | Task 11 Step 1 |
| 10 | Wiki omits the annotated example session | **Confirmed** — the spec calls for one | Task 11 Step 2 |

### Where this plan does not follow the review

- **Finding 7, "build in a temporary directory, then atomically promote."** Not
  adopted. The recovery cost of a failed site build is re-running the wizard,
  which is already safe and idempotent; a promote-from-temp scheme adds a
  second directory layout, a cleanup path, and its own failure modes to protect
  a `site.json` the operator can regenerate in a minute. The collision prompt
  removes the surprise, which was the real defect. `site.json` is still written
  before the launch scripts so a site folder is never left without its manifest.
- **Finding 8, "validate returned tile content."** Adopted only as far as
  "it must decode as a 256×256 image". Detecting an imagery service's
  *placeholder* tile — a valid image that happens to say "no data" — is a
  different problem needing content heuristics; the zoom probe plus the
  operator's own look at the downloaded picture (`xdg-open`) covers it, and the
  limitation is stated in Task 4 rather than silently ignored.
- **Finding 5, "normalize `param_id`."** Adopted, but as hardening rather than
  a bug fix: pymavlink 2.4.49 on this bench decodes `param_id` to `str`
  (verified by round-tripping a real `PARAM_VALUE`), so no current failure
  depended on it.

### Defect the review missed

The original Task 9 (now Task 10) expected the CMAC re-run to produce an empty
world diff. That cannot hold: the wizard prefers zoom 20 when the server has
it, and the z20/18-tile block center for CMAC sits **44.1 m** from the z19/9-tile
center currently in the world file — and the spawn pose is reformatted from
`0 0 0.195` to `0.0 0.0 0.195` even when the spawn point is unchanged. Task 10
now states the achievable invariant (two identical runs produce identical files,
physics block byte-identical) and lists exactly which lines are allowed to move.
