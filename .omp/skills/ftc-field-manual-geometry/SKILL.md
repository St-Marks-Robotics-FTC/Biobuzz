---
name: ftc-field-manual-geometry
description: "FTC field element/goal coordinates missing from a team repo (auto pathing, vision targeting); extract from the official Competition Manual, never guess."
---

When a season name (e.g. "Biobuzz" = BIOBUZZ presented by RTX, 2026-27) matches a real FTC game, authoritative geometry exists — never guess from a screenshot or ask the user for one.

1. Check `https://ftc-resources.firstinspires.org/ftc/game` — confirms the season name and links Competition Manual sections.
2. Manual sections are PDF; `read` extracts text only for some pages. `curl` the PDF, rasterize with `pdftoppm -png -r 150 -f N -l M file.pdf prefix`, then `read` the PNGs (vision) for tile grids, dimensions, and figure callouts missing as text.
3. Cross-validate manual numbers against the repo's existing pose data: compute bearing (`atan2`) from an existing shoot-pose to your candidate coordinate and compare with that pose's heading — match within a couple degrees corroborates the target and resolves ambiguity (e.g. which symmetric structure is "blue" vs "red").
4. Disclose tolerance: the manual's general dimensional tolerance (often ±1 in.); the STEP/CAD link is the source of truth for competition-grade precision — prefer it over pixel positions when precision matters.

## PedroPathing setup ground-truthing

NEVER write pinned-library setup code from memory. Read the pinned version from `build.dependencies.gradle`, find the artifact's `-sources.jar` under `~/.gradle/caches/modules-2/files-2.1/<group>/<artifact>/<version>/` (or download from `https://repo1.maven.org/maven2/...` via curl for transitive deps), `unzip` to scratch, and grep the real source for exact field names and constructor signatures (e.g. `PinpointConfig.name`/`.xPodOffset`/`.yPodOffset`, and `Follower(Localizer, Drivetrain, Algorithm)` is the only constructor). Cross-check the docs site for the idiomatic wiring — e.g. the AutoTune web UI at `192.168.43.1:10158` generates the exact tuned `PinpointConfig`/`ForesightConfig` to paste in; hand-deriving those constants is always wrong. If `pedro/Tuning.java` is an empty stub while `pedro/procedures/` vendors tuners, wire it via `@Tuner`-annotated static factories returning `Procedure`.
