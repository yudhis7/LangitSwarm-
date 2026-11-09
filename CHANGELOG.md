# Changelog

## 0.3.2 — 2026-11-09
### Fixed
- Windows: the app window crashed while opening ("Argument 'picture' must be a picture that can
  be used as a Icon"): the window icon was handed to .NET as a PNG. The window now takes the
  icon from LangitSwarm.exe.
- If the app window closes or fails before the app has started in it, the launcher now opens
  Langit Swarm in the web browser instead of stopping.
- `LangitSwarm.exe.config` next to the exe keeps the program alive after an error in the window
  thread (so the browser fallback can run) and allows the window libraries from a downloaded folder.

## 0.3.1 — 2026-11-09
### Fixed
- Windows: the app window could stay blank/black. The launcher now
  - removes Windows' "downloaded from the internet" block from the program's own files
    (it stops .NET from loading the window's WebView2 libraries),
  - checks for the Microsoft Edge WebView2 Runtime and .NET Framework before opening the window,
  - gives WebView2 a writable data folder (`%LOCALAPPDATA%\LangitSwarm\WebView2`),
  - watches the window: if the app has not started in it within 45 s, it closes the window and
    opens Langit Swarm in the default web browser instead.
- Start-up problems are shown on the splash screen and in the console instead of a blank screen;
  the console shows progress and everything is logged to `%USERPROFILE%\LangitSwarm\logs\launcher.log`.
- UI files are always revalidated, so an update is never shown with cached old scripts.
### Added
- `LangitSwarm.exe --safe-mode` (app window without GPU acceleration) and `--browser`;
  shortcuts `Open in browser.cmd` and `Safe mode (no GPU).cmd` in the Windows package.

## 0.3.0 — 2026-11-09
### Brand
- Langit Swarm logo throughout the app: title-bar lockup, start-up splash with the tagline,
  About box (click the logo), optional logo on the 3D view for client previews and
  recordings (toggle *logo*), browser/tab icon, Windows `.exe` and window icon.
- Brand colours in the operator UI (orange accents, cream text) while keeping the dark
  night-operations theme and the safety colours (green go, amber warning, red danger).
- Demo show ends with the Langit Swarm emblem flown by the drones.
- `tools/make_brand_assets.py` regenerates every logo file and icon from the master artwork
  (`assets/brand/logo_master.png`) if the logo changes.
- Export manifests carry `"generator": "Langit Swarm <version>"`.

## 0.2.0 — 2026-11-09
### Added
- **Import from Blender / Skybrush Studio**: `.skyc`, `show.json` and Skybrush CSV `.zip`.
  Exact Bézier trajectories, Skybrush LED bytecode (incl. loops), staggered take-off times,
  cues, import notes. Placement by rotation about the pad-grid centre and x/y offset.
- Validation for imported shows: whole-show pairwise separation (cached per file, reused for
  moved shows and live re-plans), speed, acceleration, geofence, audience, altitude, pad
  spacing, battery.
- Waypoint missions for imported shows: synchronised-distance path simplification with
  automatic loosening to the new `mission_max_items` setting, per-leg speeds, hover holds or
  UTC NAV_DELAY, TAKEOFF/LAND only below 2.5 m.
- Staggered take-off in fleet operations: per-drone start times over MAVLink; HOLD pauses
  waiting drones; LAND ALL / RETURN / KILL cancel pending take-offs.
- Export: `imported_show.npz`, Bézier trajectory CSVs, per-drone 25 fps light files.
- UI: *Import Blender / Skybrush show…*, imported-show panel (placement, tolerance, notes).
- Tests: importer, simulator and MAVLink rehearsals of imported shows, server API test.

### Changed / fixed
- RESUME now resumes exactly the drones that were paused (a drone that had just lifted off
  could be left in HOLD).
- A compile that finishes after the show was replaced (import, load, new) is discarded.
- MAVLink emulator: separate climb / descent speeds, relative NAV_DELAY.

## 0.1.0 — 2026-11-07
- First release: designer, CAPT compiler, validator, simulator, MAVLink fleet control.
