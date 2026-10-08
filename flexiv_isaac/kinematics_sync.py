# Copyright (c) 2026, Flexiv Ltd. All rights reserved.
#
# Pulls a robot's calibrated kinematics through Flexiv RDK. RDK's
# Model.SyncKinematicsYAML() fills in a template YAML, the nominal kinematics of
# the robot's model from flexiv_description, with the robot's own values.
#
# Needs only PyYAML, and flexivrdk for the sync itself, so it also runs outside
# Isaac Sim. Run as a script, it is the child process KinematicsSync starts:
#   python kinematics_sync.py <robot_sn> <out_yaml>

import os
import re
import subprocess
import sys
import tempfile
import time
import urllib.request

import yaml

# The nominal kinematics templates, one per model at
# config/<model dir>/default_kinematics.yaml
FLEXIV_DESCRIPTION_REPO = "flexivrobotics/flexiv_description"
FLEXIV_DESCRIPTION_BRANCH = "humble"

# Models whose SimReady asset name and flexiv_description config dir do not
# follow from the model by the default rules: (asset name, config dir).
_MODEL_NAMES = {
    "EnlightL": ("enlight_l", "Enlight-L"),
    "EnlightLL": ("enlight_ll", "Enlight-LL"),
}

# Template entries to sync beyond what a model's template lists, by asset name.
# SyncKinematicsYAML fills in only the entries the template lists, and the robot
# has its arm adapters' calibration, but the Enlight-LL template lists only the
# arms (ARM_1 / ARM_2). The adapters are added at their nominal pose (identity,
# as in the model) so the sync fills them in too; entries the template already
# lists are left as they are.
_NOMINAL_ADAPTER = {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
TEMPLATE_ADDITIONS_BY_MODEL = {
    "enlight_ll": {
        "EXT_AXIS": {
            "left_arm_adapter": dict(_NOMINAL_ADAPTER),
            "right_arm_adapter": dict(_NOMINAL_ADAPTER),
        },
    },
}


def model_from_serial(robot_sn):
    """Model of a robot serial number: the part before the first '-', without
    spaces, e.g. "Rizon 4s-123456" -> "Rizon4s", "Enlight L-zAVAA2" -> "EnlightL"."""
    return robot_sn.split("-")[0].strip().replace(" ", "")


def robot_name_from_serial(robot_sn):
    """Name of a robot in Isaac Sim, which allows neither spaces nor dashes, e.g.
    "Enlight L-zAVAA2" -> "EnlightL_zAVAA2". Elements Studio displays serial numbers
    with a space in the model name, so stripping spaces is required, not cosmetic.
    This is also exactly how the plugin derives its own topic suffix."""
    return robot_sn.replace(" ", "").replace("-", "_")


def asset_name_for_model(model):
    """SimReady asset name of a model, e.g. "Rizon4s" -> "rizon_4s"."""
    if model in _MODEL_NAMES:
        return _MODEL_NAMES[model][0]
    return re.sub(r"(?<=[A-Za-z])(?=\d)", "_", model).lower()


def description_dir_for_model(model):
    """flexiv_description config dir of a model, e.g. "EnlightLL" -> "Enlight-LL"."""
    return _MODEL_NAMES[model][1] if model in _MODEL_NAMES else model


def nominal_template(model):
    """The model's nominal kinematics template from flexiv_description, as YAML
    text, with the entries of TEMPLATE_ADDITIONS_BY_MODEL added."""
    url = (
        f"https://raw.githubusercontent.com/{FLEXIV_DESCRIPTION_REPO}/"
        f"{FLEXIV_DESCRIPTION_BRANCH}/config/{description_dir_for_model(model)}/"
        f"default_kinematics.yaml"
    )
    try:
        with urllib.request.urlopen(url, timeout=30) as resp:
            content = resp.read().decode("utf-8")
    except Exception as e:  # noqa: BLE001 -- say which model and URL failed
        raise RuntimeError(f"could not fetch the kinematics template of [{model}] from {url}: {e}")

    additions = TEMPLATE_ADDITIONS_BY_MODEL.get(asset_name_for_model(model), {})
    if not additions:
        return content
    doc = yaml.safe_load(content) or {}
    kine = doc.get("kinematics") or {}
    for section, entries in additions.items():
        missing = {k: v for k, v in entries.items() if k not in (kine.get(section) or {})}
        if section in kine:
            kine[section] = {**(kine[section] or {}), **missing}
        else:
            # Prepend, as in the templates that carry it (e.g. MICO-Core's EXT_AXIS)
            kine = {section: missing, **kine}
    doc["kinematics"] = kine
    return yaml.safe_dump(doc, sort_keys=False)


def sync_kinematics_yaml(robot_sn, out_path):
    """Write the robot's calibrated kinematics YAML to out_path: its model's
    template, filled in by RDK. Returns the number of joints synced."""
    with open(out_path, "w") as f:
        f.write(nominal_template(model_from_serial(robot_sn)))
    import flexivrdk

    robot = flexivrdk.Robot(robot_sn)
    return flexivrdk.Model(robot).SyncKinematicsYAML(out_path)


def load_kinematics(yaml_path):
    """The `kinematics` node of a kinematics YAML."""
    with open(yaml_path) as f:
        kine = (yaml.safe_load(f) or {}).get("kinematics")
    if not kine:
        raise ValueError(f"[{yaml_path}] has no top-level 'kinematics' node")
    return kine


class KinematicsSync:
    """Runs sync_kinematics_yaml() for one robot in a child process, so the
    caller keeps running while RDK connects and syncs. The bridge app needs this
    too because RDK fails to connect from inside the Isaac Sim process.

    The YAML and the child's log go to a temporary directory, deleted once the
    result is read.
    """

    def __init__(self, robot_sn, timeout=60.0):
        self.robot_sn = robot_sn
        self._timeout = timeout
        self._start = time.monotonic()
        self._dir = tempfile.TemporaryDirectory(prefix="flexiv_kinematics_")
        self._yaml = os.path.join(self._dir.name, "kinematics.yaml")
        self._log_path = os.path.join(self._dir.name, "rdk.log")
        with open(self._log_path, "w") as log:
            self._proc = subprocess.Popen(
                [sys.executable, os.path.abspath(__file__), robot_sn, self._yaml],
                stdout=log,
                stderr=subprocess.STDOUT,
            )

    def poll(self):
        """None while the sync runs, then the synced `kinematics` node. Raises
        RuntimeError, with the end of the child's log, if the sync failed."""
        if self._proc.poll() is None:
            if time.monotonic() - self._start < self._timeout:
                return None
            self._proc.kill()
            self._proc.wait()
            return self._finish(f"no result after {self._timeout:.0f} s")
        if self._proc.returncode != 0:
            return self._finish(f"exit code {self._proc.returncode}")
        return self._finish(None)

    def _finish(self, error):
        try:
            if error is None:
                return load_kinematics(self._yaml)
            with open(self._log_path) as f:
                log = f.read().strip().splitlines()[-5:]
            raise RuntimeError(f"{error}: " + " | ".join(log))
        finally:
            self._dir.cleanup()


if __name__ == "__main__":
    n = sync_kinematics_yaml(sys.argv[1], sys.argv[2])
    print(f"Synced {n} joints")
