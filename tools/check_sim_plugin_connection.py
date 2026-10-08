#!/usr/bin/env python3
# Copyright (c) 2026, Flexiv Ltd. All rights reserved.
#
# Check that Flexiv Sim Plugin can reach the simulated robot(s) in Flexiv
# Elements Studio, before starting the Flexiv-Isaac Bridge App. The bridge app
# waits for the connection without saying why it does not come, so this is a
# quick way to tell a setup problem (serial number, network) from a bridge app
# one.
#
# Run with the Python that has flexivsimplugin installed (setup_ws.sh installs it
# into Isaac Sim's), while the simulated robot is started in Elements Studio and
# the bridge app is NOT running for the same robot:
#   cd <isaac_sim_root_dir>
#   ./python.sh /path/to/check_sim_plugin_connection.py "Enlight L-123456"

import argparse
import shutil
import subprocess
import sys
import time

import flexivsimplugin

# Zenoh multicast scouting group the plugin discovers Elements Studio on
SCOUTING_GROUP = "224.0.0.224"


def check(serial_number, timeout):
    """Wait up to timeout seconds for the plugin to connect to the robot.
    Returns (connected, seconds waited)."""
    node = flexivsimplugin.UserNode(serial_number)
    start = time.time()
    while not node.connected() and time.time() - start < timeout:
        time.sleep(0.2)
    return node.connected(), time.time() - start


def multicast_route():
    """The route multicast scouting takes, e.g. through a VPN interface, if `ip` is available."""
    if not shutil.which("ip"):
        return None
    result = subprocess.run(
        ["ip", "route", "get", SCOUTING_GROUP], capture_output=True, text=True
    )
    return result.stdout.strip().splitlines()[0] if result.returncode == 0 else None


def main():
    p = argparse.ArgumentParser(
        description="Check that Flexiv Sim Plugin can reach simulated robots in Elements Studio."
    )
    p.add_argument(
        "serial_numbers",
        nargs="+",
        help="Serial number of each robot, as Elements Studio shows it, e.g. 'Enlight L-123456'",
    )
    p.add_argument("--timeout", type=float, default=10.0, help="Seconds to wait per robot")
    args = p.parse_args()

    print(f"flexivsimplugin {flexivsimplugin.__version__}")
    failed = []
    for sn in args.serial_numbers:
        ok, waited = check(sn, args.timeout)
        print(f"[{'OK  ' if ok else 'FAIL'}] {sn}: {'connected' if ok else 'no connection'} after {waited:.1f} s")
        if not ok:
            failed.append(sn)

    if failed:
        route = multicast_route()
        print(
            "\nNot connected. Check that:\n"
            "  * the simulated robot is started in Elements Studio (the Connect toggle is on);\n"
            "  * the serial number matches the robot in Elements Studio (spaces do not matter);\n"
            "  * the Flexiv-Isaac Bridge App is not running for the same robot;\n"
            f"  * multicast to {SCOUTING_GROUP} reaches the Elements Studio computer: the plugin\n"
            "    discovers Elements Studio by Zenoh multicast scouting, which a VPN can route away\n"
            "    and Docker's default bridge network does not pass."
        )
        if route:
            print(f"    Multicast route on this computer: {route}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
