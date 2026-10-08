# Exporting a calibrated Flexiv robot USD

**You do not need this tool to run the Flexiv-Isaac Bridge App.** The app
calibrates every robot itself once it connects, without writing any files; see
[Robot calibration](../../README.md#robot-calibration) in the main README.

Use this tool when you want a USD file that carries one robot's per-unit
kinematic calibration, to use outside the bridge app, e.g. in your own Isaac Sim
scenes. It pulls the robot's calibration through RDK, the same way the bridge
app does, and saves it into a per-robot copy of the robot's USD.

## Requirements

The tool does not need Isaac Sim. Install its dependencies into any Python 3:

```
python3 -m pip install flexivrdk usd-core pyyaml
```

- `flexivrdk` — connects to the robot. Use the version that matches the robot's
  software, e.g. 2.2 for Elements Studio v3E.2.
- `usd-core` — provides `pxr` for editing the USD.
- `pyyaml` — reads the kinematics template.

RDK must be able to reach the robot: Remote Mode (*Ethernet*) must be on in
Elements Studio, and for a simulated robot, it must be connected to the bridge
app.

The nominal kinematics template that RDK fills in is fetched from
flexiv_description on GitHub, so the computer needs internet access.

## Usage

From the root of this repo, pass the robot's serial number. The source USD is
the bundled asset for that model in `assets/`:

```
python3 tools/calibration/export_calibrated_usd.py --robot-sn "Rizon4-000001"
```

Pass `--usd <path>` to calibrate a specific USD instead of the bundled one. The
Isaac Sim container (`launch_isaac_sim.sh`) mounts this repo read-only, so run
the tool on the host. For all options, run
`python3 tools/calibration/export_calibrated_usd.py --help`.

### Output

The source USD is not modified. The tool writes a sibling of the source model
dir, named after `--robot-sn`, with the same internal layout:

```
assets/rizon_4/                <- source (unchanged; keeps geometries.usd)
assets/Rizon4-000001/          <- calibrated copy
  Rizon4-000001.usda              (the calibrated USD)
  Rizon4_synced_kinematics.yaml   (the values that were applied)
  payloads/ ...                   (no geometries.usd -- shared from the source)
```

The meshes (`geometries.usd`) are shared, not duplicated, so each copy is
~100 KB. If the base asset changes, re-run to regenerate.

You can also point a bridge app config's `usd:` at the calibrated copy. The app
still pulls and applies the robot's calibration once it connects, which gives
the same result.

## Supported models

The 7-DoF Rizon series (Rizon 4, 4s, 10, ...), the Enlight L and the dual-arm
Enlight LL. The bridge app supports the same models.

### Enlight LL

Where the two arms are mounted is part of an Enlight LL's calibration (its two
arm adapters), not of the model, so the bundled `enlight_ll` asset mounts both
arms at the origin and they overlap. Calibrating places them where the robot
has them, as well as applying each arm's own kinematic calibration.

`SyncKinematicsYAML` fills in only the entries the template lists, and the
flexiv_description Enlight-LL template lists the arms but not the arm adapters,
although the robot has their calibration. So an `EXT_AXIS` section with
`left_arm_adapter` / `right_arm_adapter` is added to the template before the
sync, and RDK fills them in along with `ARM_1` (the left arm) and `ARM_2` (the
right arm).

## How it works

The calibration code is shared with the bridge app, in the `flexiv_isaac`
package at the root of this repo:

- `flexiv_isaac/kinematics_sync.py` pulls the calibrated kinematics through RDK.
- `flexiv_isaac/calibration.py` writes them into the USD's link poses and joint
  anchors. The bridge app applies them as overrides on one robot's prims, and
  this tool saves them into the per-robot copy.

The tests in [test/](test/README.md) check both against reference URDFs.
