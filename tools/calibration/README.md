# Calibrating a Flexiv robot USD from RDK

Apply a real robot's per-unit kinematic calibration to its SimReady USD so the
simulated arm matches the physical one.

## Usage (`calibrate_usd_from_rdk.py`)

With the robot connected, run with Isaac Sim's bundled Python (so `flexivrdk`
and `pxr` resolve):

```
~/isaacsim/kit/python/bin/python3 calibrate_usd_from_rdk.py \
    --robot-sn "Rizon4-000001" \
    --usd ~/isaacsim/extsDeprecated/isaacsim.robot.manipulators.examples/data/flexiv/Rizon4/Rizon4.usda
```

For all options, run `calibrate_usd_from_rdk.py --help`.

### Output

The original source USD is not modified. The script writes a sibling of the
source model dir, named after `--robot-sn`, with the same internal layout:

```
data/flexiv/Rizon4/            <- source (unchanged; keeps geometries.usd)
data/flexiv/Rizon4-000001/     <- calibrated copy
  Rizon4-000001.usda              (point the bridge app's `usd:` here)
  Rizon4_synced_kinematics.yaml   (the values that were applied)
  payloads/ ...                   (no geometries.usd -- shared from the source)
```

The meshes (`geometries.usd`) are shared, not duplicated, so each copy is
~100 KB. If the base asset changes, re-run to regenerate.
