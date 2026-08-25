# Calibrating a Flexiv robot USD from RDK

Apply a real robot's per-unit kinematic calibration to its SimReady USD so the
simulated arm matches the physical one.

## Requirements

Install the tool's dependencies into the Python you will run it with — Isaac
Sim's bundled interpreter is a convenient choice:

```
~/isaacsim/kit/python/bin/python3 -m pip install flexivrdk usd-core pyyaml
```

- `flexivrdk` — connects to the robot (`numpy` comes with it). Pin the version to
  match the robot's software package, e.g. `flexivrdk==1.9.3` for package 3.11.2.
  Tested with 1.9.x and 2.x (2.1, 2.2).
- `usd-core` — provides `pxr` for editing the USD offline (no Isaac Sim runtime
  needed).
- `pyyaml` — reads the kinematics template.

## Usage (`calibrate_usd_from_rdk.py`)

With the robot connected, pass just its serial number — the source USD is the
bundled asset for that model in the Isaac Sim install:

```
~/isaacsim/kit/python/bin/python3 calibrate_usd_from_rdk.py --robot-sn "Rizon4-000001"
```

Pass `--usd <path>` to calibrate a specific USD instead of the bundled one. For
all options, run `calibrate_usd_from_rdk.py --help`. The nominal kinematics
template is fetched from flexiv_description on GitHub, so the machine needs
internet access.

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
