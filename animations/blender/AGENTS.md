# Blender animation baseline

The user confirmed V3 as the latest Blender concept-animation baseline on 2026-10-28. Continue future animation work from `versions/v3/` and `LATEST.json`; default the next revision to V4. Keep V3, `backups/v1/`, `backups/v2/`, and `backups/autosaves/` intact as snapshots unless the user explicitly requests changes to those snapshots.

Use the V3 `.blend`, its matching generation script, and `source_cad/` assets together. Blender's glTF importer already restores metres and Z-up; do not add another axis conversion. X points into the greenhouse, Y along its length, and Z upward.

This baseline covers the historical F04 R02 animation. It does not supersede the repository's later Mode A engineering designs or integrate their geometry and trajectories. Retain the documented limits: concept folding, affine film unfolding, and no complete collision or winding validation.
