# Flight-Site Relocation Wizard Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** An interactive terminal wizard in the HIL-Sim-Bench repo that lets non-technical test operators relocate the simulated flight site (SITL / SIH / External-Physics) to any real-world location, plus an operator wiki page.

**Architecture:** Small focused Python modules in `HIL-Sim-Bench/tools/` (`geo`, `fetch_ortho`, `gazebo_files`, `board`, `sites`, `ui`) orchestrated by `site_wizard.py`. Each site becomes a saved folder under `tools/sites/<name>/` containing `site.json`, downloaded imagery, and a generated launch script. Board params go over the USB MAVLink port while IGC stays connected via the SERIAL1 radio.

**Tech Stack:** Python 3.10 (system), requests, Pillow (PIL), pymavlink, pytest. Bash for generated launch scripts. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-07-29-site-relocation-wizard-design.md` (IFT-ardupilot repo)

## Global Constraints

- Repos/paths: HIL-Sim-Bench cloned at `~/git/IFT/HIL-Sim-Bench`; wiki at `~/git/IFT/HIL-Sim-Bench.wiki`; ardupilot_gazebo at `~/git/ardupilot_gazebo`; IFT-ardupilot at `~/git/IFT/IFT-ardupilot` (branch `hitl-network`).
- Env-var overrides (used by tests, honored everywhere): `HIL_GAZEBO_DIR`, `HIL_ARDUPILOT_DIR`, `HIL_SITES_DIR`, `HIL_OFFLINE=1` (skip all network lookups), `HIL_NO_BOARD=1` (pretend no Cube is on USB — tests run on the bench laptop where the real Cube is plugged in, so board access must be explicitly disabled).
- Param names exactly: `SIM_OPOS_LAT`, `SIM_OPOS_LNG`, `SIM_OPOS_ALT`, `SIM_OPOS_HDG`, `SERIAL1_BAUD`, `SYSID_MYGCS` (NOT `SYSID_MY_GCS`).
- Radio values: Herelink → `SERIAL1_BAUD 57`, `SYSID_MYGCS 255`. Taisync → `SERIAL1_BAUD 115`, `SYSID_MYGCS 19`.
- Geo-registration contract: Gazebo world `<spherical_coordinates>` = mosaic center = `SIM_OPOS_LAT/LNG`; `SIM_OPOS_HDG 0`. The wizard NEVER touches the world's physics block (`max_step_size` stays `1/SCHED_LOOP_RATE`).
- Texture cap: 5120 px per side. Spawn point must be ≥30 m inside the texture edge.
- Operator-facing output: plain English, no stack traces (full traceback only with `--debug`).
- Never build/flash/upload firmware. Never start a second `gz sim` (both bind UDP 9002) — generated launch script refuses.
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
```

- [ ] **Step 4: Write `tools/tests/conftest.py`**

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
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
- Produces: `BadCoordinates(ValueError)`; `parse_latlng(text) -> (lat, lng)`; `get_elevation(lat, lng, timeout=10) -> float | None` (None on any failure or when `HIL_OFFLINE` is set); `pin_to_offset(pin_lat, pin_lng, meta) -> (east_m, north_m)` where `meta` has `center_lat`/`center_lng` (the dict from `fetch_ortho.fetch_mosaic`, Task 4).

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
    m_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * math.radians(lat_c))
    m_per_deg_lng = 111412.84 * math.cos(math.radians(lat_c))
    east = (pin_lng - meta["center_lng"]) * m_per_deg_lng
    north = (pin_lat - lat_c) * m_per_deg_lat
    return east, north
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
- Produces: `latlng_to_tile(lat, lng, z) -> (x, y)` floats; `tile_to_latlng(x, y, z) -> (lat, lng)`; `zoom_available(lat, lng, zoom, nside) -> bool` (network probe; False when `HIL_OFFLINE`); `pick_zoom_nside(lat, lng, target_m) -> (zoom, nside)` (probes zoom 20 then 19, texture capped at `MAX_PX = 5120`); `fetch_mosaic(lat, lng, zoom, nside, out, progress=None) -> meta` where `meta` is a dict with keys `center_lat, center_lng, bounds{north,south,west,east}, size_ew_m, size_ns_m, px, gsd_m, zoom, texture`; `progress` is called as `progress(row_done, total_rows)`. CLI preserved: `python3 tools/fetch_ortho.py <lat> <lng> <zoom> <nside> <out_jpg>`.

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


def test_fetch_mosaic_stitches_and_measures(tmp_path, monkeypatch):
    class FakeSession:
        def get(self, url, timeout=30):
            return FakeResp()
    monkeypatch.setattr(fetch_ortho.requests, "Session", FakeSession)
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


def test_zoom_available_offline(monkeypatch):
    monkeypatch.setenv("HIL_OFFLINE", "1")
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


def latlng_to_tile(lat, lng, z):
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


def _block_origin(lat, lng, zoom, nside):
    cx, cy = latlng_to_tile(lat, lng, zoom)
    return int(cx) - nside // 2, int(cy) - nside // 2


def zoom_available(lat, lng, zoom, nside, timeout=15):
    """True if the tile server serves this zoom across the whole block
    (probes the four corners and the center)."""
    if os.environ.get("HIL_OFFLINE"):
        return False
    x0, y0 = _block_origin(lat, lng, zoom, nside)
    probes = [(x0, y0), (x0 + nside - 1, y0), (x0, y0 + nside - 1),
              (x0 + nside - 1, y0 + nside - 1),
              (x0 + nside // 2, y0 + nside // 2)]
    for x, y in probes:
        try:
            r = requests.get(TILE_URL.format(z=zoom, x=x, y=y),
                             timeout=timeout)
            if r.status_code != 200:
                return False
        except requests.RequestException:
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
            url = TILE_URL.format(z=zoom, x=x0 + i, y=y0 + j)
            last_err = None
            for _attempt in range(3):
                try:
                    r = sess.get(url, timeout=30)
                    r.raise_for_status()
                    last_err = None
                    break
                except requests.RequestException as err:
                    last_err = err
            if last_err is not None:
                raise RuntimeError(f"imagery download failed: {last_err}")
            mosaic.paste(Image.open(io.BytesIO(r.content)),
                         (i * TILE_PX, j * TILE_PX))
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
        "center_lng": round(lng_c, 8),
        "bounds": {"north": lat_n, "south": lat_s,
                   "west": lng_w, "east": lng_e},
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
- Test: `tools/tests/test_gazebo_files.py`

**Interfaces:**
- Consumes: `meta` dict from `fetch_ortho.fetch_mosaic` **plus** an `elevation_m` key the caller merges in.
- Produces: `GazeboFilesError(RuntimeError)`; `apply_site(gazebo_dir, meta, spawn_east=0.0, spawn_north=0.0, texture_src=None) -> list[Path]` (backs up to `.bak`, idempotent); `read_current(gazebo_dir) -> (lat, lng, elevation) | None`.
- File targets inside `gazebo_dir`: `worlds/if800_survey.sdf` (tags `<latitude_deg>`, `<longitude_deg>`, `<elevation>`, and the `<pose degrees="true">` immediately after `<uri>model://iris_with_gimbal</uri>`) and `models/survey_ground/model.sdf` (exactly two `<plane><size>` entries), texture at `models/survey_ground/materials/textures/survey_ortho.jpg`.

- [ ] **Step 1: Write the failing tests**

`tools/tests/test_gazebo_files.py`:

```python
import pytest

import gazebo_files

WORLD = """<sdf version="1.9"><world name="if800_survey">
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

META = {"center_lat": 39.58001, "center_lng": -104.84301,
        "size_ew_m": 480.25, "size_ns_m": 478.5, "elevation_m": 1793.0}


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

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest tools/tests/test_gazebo_files.py -v`
Expected: FAIL with "No module named 'gazebo_files'".

- [ ] **Step 3: Write `tools/gazebo_files.py`**

```python
"""Rewrites the if800_survey Gazebo world + survey_ground model for a new
site. Never touches the physics block (max_step_size stays 1/SCHED_LOOP_RATE)."""
import re
import shutil
from pathlib import Path


class GazeboFilesError(RuntimeError):
    pass


def _paths(gazebo_dir):
    gz = Path(gazebo_dir).expanduser()
    return (gz / "worlds/if800_survey.sdf",
            gz / "models/survey_ground/model.sdf",
            gz / "models/survey_ground/materials/textures/survey_ortho.jpg")


def apply_site(gazebo_dir, meta, spawn_east=0.0, spawn_north=0.0,
               texture_src=None):
    """Point the survey world at a new mosaic. `meta` is the dict from
    fetch_ortho.fetch_mosaic plus an `elevation_m` key. Returns changed paths."""
    world, model, tex = _paths(gazebo_dir)
    for p in (world, model):
        if not p.exists():
            raise GazeboFilesError(
                f"missing {p} — External Physics is not set up on this laptop")
        shutil.copy2(p, p.with_suffix(p.suffix + ".bak"))

    w = world.read_text()
    w, n_lat = re.subn(r"<latitude_deg>[^<]*</latitude_deg>",
                       f"<latitude_deg>{meta['center_lat']:.8f}</latitude_deg>",
                       w, count=1)
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
            f"{world} did not look as expected — restore the .bak file "
            "and ask a developer")

    m = model.read_text()
    m, n_size = re.subn(
        r"<size>[^<]*</size>",
        f"<size>{meta['size_ew_m']:.2f} {meta['size_ns_m']:.2f}</size>", m)
    if n_size != 2:
        raise GazeboFilesError(
            f"expected 2 plane <size> entries in {model}, found {n_size}")

    world.write_text(w)
    model.write_text(m)
    changed = [world, model]
    if texture_src is not None:
        tex.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(texture_src, tex)
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

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest tools/tests/test_gazebo_files.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add tools/gazebo_files.py tools/tests/test_gazebo_files.py
git commit -m "tools: gazebo world/model rewriter"
```

---

### Task 6: `board.py` — Cube params over USB

**Files:**
- Create: `tools/board.py`
- Test: `tools/tests/test_board.py`

**Interfaces:**
- Produces: `BoardError(RuntimeError)`; `HERELINK = {"SERIAL1_BAUD": 57, "SYSID_MYGCS": 255}`; `TAISYNC = {"SERIAL1_BAUD": 115, "SYSID_MYGCS": 19}`; `find_mavlink_port() -> str | None`; `connect(port, timeout=10) -> conn`; `set_params(conn, params: dict, retries=3) -> dict` (raises `BoardError` naming the first param that fails to verify); `get_param(conn, name, timeout=3) -> float`; `reboot(conn)`.
- The wizard (Task 8) treats `BoardError` as "print manual IGC steps".

- [ ] **Step 1: Write the failing tests**

`tools/tests/test_board.py`:

```python
import struct
import types

import pytest

import board


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
        v32 = struct.unpack("f", struct.pack("f", float(self.store[self.last])))[0]
        return types.SimpleNamespace(param_id=self.last, param_value=v32)


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


def test_set_params_verifies_float32_values():
    conn = FakeConn()
    verified = board.set_params(conn, {"SIM_OPOS_LAT": -35.36301596,
                                       "SIM_OPOS_HDG": 0.0})
    assert set(verified) == {"SIM_OPOS_LAT", "SIM_OPOS_HDG"}
    # float32 quantization of the latitude is accepted
    assert verified["SIM_OPOS_LAT"] == pytest.approx(-35.36301596, abs=1e-4)


def test_set_params_raises_on_no_echo():
    class DeafConn(FakeConn):
        def recv_match(self, type=None, blocking=True, timeout=1):
            return None
    with pytest.raises(board.BoardError, match="SIM_OPOS_LAT"):
        board.set_params(DeafConn(), {"SIM_OPOS_LAT": 1.0}, retries=1)


def test_get_param_reads_back():
    conn = FakeConn(store={"SYSID_MYGCS": 255.0})
    assert board.get_param(conn, "SYSID_MYGCS") == 255.0


def test_reboot_sends_command():
    conn = FakeConn()
    board.reboot(conn)
    assert len(conn.mav.commands) == 1
    assert conn.mav.commands[0][3] == 1        # param1 = reboot autopilot


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
import time

HERELINK = {"SERIAL1_BAUD": 57, "SYSID_MYGCS": 255}
TAISYNC = {"SERIAL1_BAUD": 115, "SYSID_MYGCS": 19}


class BoardError(RuntimeError):
    pass


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
    if conn.wait_heartbeat(timeout=timeout) is None:
        conn.close()
        raise BoardError("no heartbeat from the Cube — check the USB cable")
    return conn


def set_params(conn, params, retries=3):
    """Set each param and verify the echo. Returns verified values."""
    from pymavlink import mavutil
    verified = {}
    for name, value in params.items():
        ok = False
        for _attempt in range(retries):
            conn.mav.param_set_send(conn.target_system, conn.target_component,
                                    name.encode(), float(value),
                                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            deadline = time.time() + 3
            while time.time() < deadline:
                msg = conn.recv_match(type="PARAM_VALUE", blocking=True,
                                      timeout=1)
                if msg is None or msg.param_id != name:
                    continue
                # float32 quantization is expected (~0.2 m on a latitude)
                if abs(msg.param_value - float(value)) <= max(
                        1e-6, abs(float(value)) * 1e-6):
                    verified[name] = msg.param_value
                    ok = True
                break
            if ok:
                break
        if not ok:
            raise BoardError(f"{name} did not verify over USB")
    return verified


def get_param(conn, name, timeout=3):
    conn.mav.param_request_read_send(conn.target_system,
                                     conn.target_component,
                                     name.encode(), -1)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is not None and msg.param_id == name:
            return float(msg.param_value)
    raise BoardError(f"no answer reading {name}")


def reboot(conn):
    from pymavlink import mavutil
    conn.mav.command_long_send(conn.target_system, conn.target_component,
                               mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                               0, 1, 0, 0, 0, 0, 0, 0)
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

### Task 7: `sites.py` — saved sites and launch scripts

**Files:**
- Create: `tools/sites.py`
- Test: `tools/tests/test_sites.py`

**Interfaces:**
- Produces: `SITES_DIR`, `GAZEBO_DIR`, `ARDUPILOT_DIR` (Path constants honoring `HIL_SITES_DIR`/`HIL_GAZEBO_DIR`/`HIL_ARDUPILOT_DIR`); `slugify(text) -> str`; `site_dir(name) -> Path`; `save_site(name, data)`; `load_site(name) -> dict`; `list_sites() -> list[str]`; `write_launch_scripts(name, data) -> Path | None` (executable script path; None for SIH).
- `data` dict keys (set by Task 8): `name, mode ("sitl"|"sih"|"external"), lat, lng, elevation_m, heading, spawn_east, spawn_north, mosaic (fetch_mosaic meta dict or None)`.

- [ ] **Step 1: Write the failing tests**

`tools/tests/test_sites.py`:

```python
import json
import os
import stat

import pytest

import sites


@pytest.fixture
def sandbox(tmp_path, monkeypatch):
    monkeypatch.setattr(sites, "SITES_DIR", tmp_path / "sites")
    monkeypatch.setattr(sites, "ARDUPILOT_DIR", tmp_path / "ap")
    return tmp_path


DATA = {"name": "meridian", "mode": "sitl", "lat": 39.58012,
        "lng": -104.84325, "elevation_m": 1793.0, "heading": 0.0,
        "spawn_east": 0.0, "spawn_north": 0.0, "mosaic": None}


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
    assert "pgrep" in text
    assert path.stat().st_mode & stat.S_IXUSR


def test_sih_has_no_script(sandbox):
    assert sites.write_launch_scripts("meridian", {**DATA, "mode": "sih"}) is None
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest tools/tests/test_sites.py -v`
Expected: FAIL with "No module named 'sites'".

- [ ] **Step 3: Write `tools/sites.py`**

```python
"""Saved-site storage (tools/sites/<name>/) and generated launch scripts."""
import json
import os
import re
import stat
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
SITES_DIR = Path(os.environ.get("HIL_SITES_DIR", TOOLS_DIR / "sites"))
GAZEBO_DIR = Path(os.environ.get("HIL_GAZEBO_DIR",
                                 "~/git/ardupilot_gazebo")).expanduser()
ARDUPILOT_DIR = Path(os.environ.get("HIL_ARDUPILOT_DIR",
                                    "~/git/IFT/IFT-ardupilot")).expanduser()

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
if pgrep -x "gz" >/dev/null || pgrep -f "gz sim" >/dev/null; then
  echo "Another Gazebo is already running — two would fight over the"
  echo "simulator port (9002). Close it (Ctrl+C in its terminal), then"
  echo "run this script again."
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
        text = LAUNCH_GAZEBO.replace("@NAME@", name)
    path.write_text(text)
    path.chmod(path.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    return path
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest tools/tests/test_sites.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add tools/sites.py tools/tests/test_sites.py
git commit -m "tools: saved sites + generated launch scripts"
```

---

### Task 8: `site_wizard.py` — the wizard itself

**Files:**
- Create: `tools/site_wizard.py`
- Test: `tools/tests/test_wizard_flows.py`

**Interfaces:**
- Consumes: everything above, by the exact names in each task's Produces block.
- Produces: `python3 tools/site_wizard.py [--debug]`. Menu: 1 new site / 2 saved site / 3 show current / 4 radio setup / 5 quit.

- [ ] **Step 1: Write the failing integration tests (subprocess, piped answers)**

`tools/tests/test_wizard_flows.py`:

```python
import json
import subprocess
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1]

WORLD = """<sdf version="1.9"><world name="if800_survey">
  <spherical_coordinates>
    <latitude_deg>-35.36301596</latitude_deg>
    <longitude_deg>149.16515350</longitude_deg>
    <elevation>584</elevation>
  </spherical_coordinates>
  <include>
    <uri>model://iris_with_gimbal</uri>
    <pose degrees="true">0 0 0.195 0 0 90</pose>
  </include>
</world></sdf>
"""


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
        "1793",                      # elevation (offline -> manual entry)
        "meridian",                  # site name
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


def test_sih_flow_falls_back_to_manual_steps(tmp_path):
    # No Cube on USB in the test env -> wizard must print the IGC steps.
    answers = [
        "1",                         # new site
        "2",                         # SIH
        "-35.363, 149.165",
        "584",
        "cmac-sih",
        "1",                         # radio: keep current
        "5",
    ]
    result, base = run_wizard(answers, tmp_path)
    assert result.returncode == 0, result.stderr
    assert "SIM_OPOS_LAT" in result.stdout       # manual fallback shown
    assert "SIM_OPOS_ALT = 584" in result.stdout
    assert (base / "sites/cmac-sih/site.json").exists()


def test_show_current_reads_world(tmp_path):
    answers = ["3", "5"]
    result, _ = run_wizard(answers, tmp_path)
    assert result.returncode == 0, result.stderr
    assert "-35.363016" in result.stdout


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
EDGE_MARGIN_M = 30


def choose_coords(prompt="coordinates"):
    ui.say("  Paste the coordinates from Google Maps")
    ui.say("  (right-click the spot on the map, then click the numbers to copy)")
    while True:
        try:
            return geo.parse_latlng(ui.ask(prompt))
        except geo.BadCoordinates as err:
            ui.say(f"  {err}")


def choose_elevation(lat, lng):
    elev = geo.get_elevation(lat, lng)
    if elev is not None:
        if ui.ask_yesno(f"Ground elevation looked up: {elev:.0f} m — keep?"):
            return elev
    else:
        ui.say("  Elevation lookup unavailable (offline, or VPN blocking it).")
    while True:
        try:
            return float(ui.ask("ground elevation in meters"))
        except ValueError:
            ui.say("  Please enter a number, e.g. 584")


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


def apply_board_params(params, offer_reboot=True):
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
        if offer_reboot and ui.ask_yesno(
                "Reboot the Cube so they take effect?"):
            board.reboot(conn)
            ui.say("  ✓ Reboot sent — the Cube reconnects on its own.")
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


def launch_script(script, log_path):
    """Run a generated launch script — new terminal window if possible,
    otherwise background with output to a log file."""
    try:
        subprocess.Popen(["gnome-terminal", "--", "bash", str(script)])
        ui.say(f"  ✓ Launched in a new terminal window ({script.name}).")
        return
    except FileNotFoundError:
        pass
    log = open(log_path, "ab")
    subprocess.Popen(["bash", str(script)], stdout=log, stderr=log)
    ui.say(f"  ✓ Launched in the background — output goes to {log_path}")


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
    # The Cube's location must be the imagery center, not the clicked point.
    data["lat"], data["lng"] = meta["center_lat"], meta["center_lng"]
    data["mosaic"] = meta

    if ui.ask_yesno("Open the picture to check it's the right place?",
                    default=False):
        subprocess.Popen(["xdg-open", str(ortho)])

    data["spawn_east"], data["spawn_north"] = choose_spawn(meta)

    gazebo_files.apply_site(sites.GAZEBO_DIR,
                            {**meta, "elevation_m": data["elevation_m"]},
                            data["spawn_east"], data["spawn_north"],
                            texture_src=ortho)
    ui.say("  ✓ Gazebo world updated.")

    script = sites.write_launch_scripts(name, data)
    sites.save_site(name, data)
    if ui.ask_yesno("Launch Gazebo at the new site now?"):
        launch_script(script, sites.site_dir(name) / "gazebo.log")
    apply_board_params(sim_opos_params(data))


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
    ui.say(f"  ✓ Saved. Relaunch any time with: {script}")
    if ui.ask_yesno("Launch SITL now?"):
        launch_script(script, sites.site_dir(name) / "sitl.log")


def sih_flow(name, data):
    sites.save_site(name, data)
    apply_board_params(sim_opos_params(data))


def new_site():
    mode = MODE_KEYS[ui.ask_choice("Which setup are you using?", SIM_MODES,
                                   default_idx=2)]
    lat, lng = choose_coords()
    elev = choose_elevation(lat, lng)
    default_name = sites.slugify(f"site-{lat:.4f}-{lng:.4f}")
    name = sites.slugify(ui.ask("short name for this site", default_name))
    name = name or default_name
    data = {"name": name, "mode": mode, "lat": lat, "lng": lng,
            "elevation_m": elev, "heading": 0.0,
            "spawn_east": 0.0, "spawn_north": 0.0, "mosaic": None}
    {"sitl": sitl_flow, "sih": sih_flow, "external": external_flow}[mode](
        name, data)
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
        if ui.ask_yesno("Launch Gazebo now?"):
            launch_script(sites.site_dir(name) / "launch-gazebo.sh",
                          sites.site_dir(name) / "gazebo.log")
        apply_board_params(sim_opos_params(data))
    elif data["mode"] == "sih":
        apply_board_params(sim_opos_params(data))
    else:
        if ui.ask_yesno("Launch SITL now?"):
            launch_script(sites.site_dir(name) / "launch-sitl.sh",
                          sites.site_dir(name) / "sitl.log")
    radio_prompt()


def show_current():
    current = gazebo_files.read_current(sites.GAZEBO_DIR)
    if current is None:
        ui.say("  Gazebo world files not found — External Physics is not "
               "set up on this laptop.")
    else:
        lat, lng, elev = current
        ui.say(f"  Gazebo world is at: {lat:.6f}, {lng:.6f} "
               f"(ground {elev:.0f} m)")
        ui.say(f"  Map: https://www.google.com/maps?q={lat:.6f},{lng:.6f}")
    port = board.find_mavlink_port()
    if port is None:
        ui.say("  (Cube not connected over USB.)")
        return
    try:
        conn = board.connect(port, timeout=5)
        lat = board.get_param(conn, "SIM_OPOS_LAT")
        lng = board.get_param(conn, "SIM_OPOS_LNG")
        conn.close()
        ui.say(f"  Cube is set to:     {lat:.6f}, {lng:.6f}")
    except board.BoardError as err:
        ui.say(f"  (Could not read the Cube: {err})")


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

- [ ] **Step 4: Run the full test suite**

Run: `python3 -m pytest tools/tests -v`
Expected: all PASS (all modules + the four wizard flow tests).

- [ ] **Step 5: Commit**

```bash
git add tools/site_wizard.py tools/tests/test_wizard_flows.py
git commit -m "tools: interactive flight-site wizard"
```

---

### Task 9: Live bench verification (CHECKPOINT — needs the user)

**Files:** none (verification only). Do NOT check these boxes without actually doing them on the bench.

- [ ] **Step 1: Ask the user for go-ahead** — this task restarts the bench Gazebo and writes params to the live Cube.

- [ ] **Step 2: New-site run at CMAC (idempotence check)**

```bash
cd ~/git/IFT/HIL-Sim-Bench
cp ~/git/ardupilot_gazebo/worlds/if800_survey.sdf /tmp/world.before
python3 tools/site_wizard.py
# answers: new site -> External Physics -> "-35.36301596, 149.16515350"
#          -> keep elevation -> name "cmac" -> standard area -> center spawn
#          -> launch Gazebo YES -> set params YES -> reboot YES -> radio: keep
diff /tmp/world.before ~/git/ardupilot_gazebo/worlds/if800_survey.sdf
```

Expected: world diff empty or coordinates identical to 8 decimals; Gazebo running; board reconnects; statustext shows `SIM rate 200/200Hz RTF ~1.00` within ~30 s of reboot.

- [ ] **Step 3: Saved-site switch** — rerun wizard, "Switch to a saved site" → `cmac`. Expected: no re-download, world re-applied, radio prompt appears.

- [ ] **Step 4: Radio read-back only** — wizard menu 4 → check current values first via menu 3 / IGC; choose the radio actually on the bench so values are written but unchanged. Expected: "Parameters set and verified."

- [ ] **Step 5: Record results** — paste the observed statustext line and any deviations into the session notes; fix anything broken before Task 10.

---

### Task 10: Wiki page + link edits

**Files:**
- Create: `~/git/IFT/HIL-Sim-Bench.wiki/Changing-the-Flight-Site.md`
- Modify: `HIL-Software-Overview.md`, `External-Physics-SIH.md`, `SITL-Instructions.md`, `SIH-Instructions.md` (same wiki repo)

- [ ] **Step 1: Clone the wiki**

```bash
git clone git@github.com:Inspired-Flight-Technologies/HIL-Sim-Bench.wiki.git ~/git/IFT/HIL-Sim-Bench.wiki
```

- [ ] **Step 2: Write `Changing-the-Flight-Site.md`**

```markdown
# Changing the Flight Site

The simulator can fly anywhere on Earth. The **Flight Site Wizard** moves it
for you — no file editing, no parameter typing.

## What you need

* The bench laptop (internet on, VPN off — it can block the imagery download)
* For SIH / External Physics: the Cube connected by USB
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

## Radios

The wizard's **Radio setup** menu configures the telemetry radio. It sets:

| Radio    | SERIAL1_BAUD | SYSID_MYGCS |
|----------|--------------|-------------|
| Herelink | 57 (57600)   | 255         |
| Taisync  | 115 (115200) | 19          |

## If something goes wrong

| What you see | What to do |
|---|---|
| Drone sits on a grey/empty ground in Gazebo | The Cube's location doesn't match the world. Run the wizard → **Switch to a saved site** → pick the site again, and say yes to setting parameters. |
| "imagery download failed" | The laptop is offline or the VPN is blocking it (see [[Home]]). Turn the VPN off and try again. |
| "no heartbeat from the Cube" | Check the USB cable. Or set the values the wizard printed yourself in IGC (it prints exact steps). |
| "Another Gazebo is already running" | Close the old Gazebo window (Ctrl+C in its terminal), then relaunch from the wizard or the site's `launch-gazebo.sh`. |
| The wizard crashes | Run `python3 tools/site_wizard.py --debug` and send the output to a developer. |
```

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

- [ ] **Step 4: Push the wiki**

```bash
cd ~/git/IFT/HIL-Sim-Bench.wiki
git add -A && git commit -m "Add Changing the Flight Site page; link from overview pages; fix SYSID_MYGCS"
git push
```

- [ ] **Step 5: Verify** — open https://github.com/Inspired-Flight-Technologies/HIL-Sim-Bench/wiki/Changing-the-Flight-Site in a browser (or re-fetch with git) and confirm it renders.

---

### Task 11: IFT-ardupilot pointer updates

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
same registration values described above. A 1:1 drone orthophoto of the site
still beats Esri imagery if available — same registration rules (north-up
image, plane size = ground extent, center = SIM_OPOS).
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
