#!/usr/bin/env python3
# Copyright (c) 2026, Flexiv Ltd. All rights reserved.
#
# Swing all joints of every arm of a robot, e.g. both arms of an Enlight LL or a
# MICO, back and forth around the home pose, by alternating MoveJ primitives sent
# to all arms at once. Useful to see a simulated dual-arm robot move in Isaac Sim.
#
# Based on the RDK example intermediate1_non_realtime_joint_position_control.py,
# but uses primitive execution instead of streamed joint position commands.
#
# Needs a flexivrdk with multi-arm support (tested with 2.2). Enable Remote Mode in
# Elements Studio first, then run, e.g.
#   python3 dual_arm_joint_swing.py "Enlight LL-123456" --amplitude 6 --cycles 3

import sys
import time
import math
import argparse
import logging
import flexivrdk  # pip install "flexivrdk==2.2.*", the RDK release line of Elements Studio v3E.2

logging.basicConfig(level=logging.INFO, format="[%(asctime)s] [Example] [%(levelname)s] %(message)s")
logger = logging.getLogger()


def wait_until_reached(robot, groups, targets_deg, tol_deg=0.5, timeout=10.0):
    """Block until every group's joints are within tol_deg of its target."""
    start = time.time()
    while time.time() - start < timeout:
        if robot.fault():
            raise RuntimeError("Fault occurred on the connected robot")
        states = robot.states()
        if all(
            max(abs(math.degrees(q) - t) for q, t in zip(states[g].q, targets_deg[g])) < tol_deg
            for g in groups
        ):
            return
        time.sleep(0.02)
    raise RuntimeError("Timed out waiting for the arms to reach the target")


def wait_until_settled(robot, groups, timeout=15.0):
    """Block until every joint of every group has stopped moving, e.g. after Home."""
    start = time.time()
    still = 0
    while time.time() - start < timeout:
        if robot.fault():
            raise RuntimeError("Fault occurred on the connected robot")
        states = robot.states()
        moving = any(abs(dq) > 0.005 for g in groups for dq in states[g].dq)
        still = 0 if moving else still + 1
        if still >= 25:  # 0.5 s without motion
            return
        time.sleep(0.02)
    raise RuntimeError("Timed out waiting for the arms to settle")


def main():
    argparser = argparse.ArgumentParser()
    argparser.add_argument("robot_sn", help="Serial number of the robot, e.g. Enlight LL-123456")
    argparser.add_argument("--amplitude", type=float, default=6.0, help="Swing amplitude [deg]")
    argparser.add_argument("--cycles", type=int, default=5, help="Number of swings (0: forever)")
    args = argparser.parse_args()

    robot = flexivrdk.Robot(args.robot_sn)

    # Clear fault on the connected robot if any
    if robot.fault():
        logger.warning("Fault occurred on the connected robot, trying to clear ...")
        if not robot.ClearFault():
            logger.error("Fault cannot be cleared, exiting ...")
            return 1

    # Servo on the robot, make sure the E-stop is released
    logger.info("Servo on the robot ...")
    robot.ServoOn()
    while not robot.operational():
        time.sleep(1)
    logger.info("Robot is now operational")

    # Every arm of the robot, e.g. ARM_1 and ARM_2 of a dual-arm robot
    groups = list(robot.info().single_arm_groups)
    logger.info(f"Arms: {[g.name for g in groups]}")

    robot.SwitchMode(flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION)
    logger.info("Moving to home pose")
    robot.ExecutePrimitive({g: flexivrdk.PrimitiveArgs("Home", {}) for g in groups})
    time.sleep(0.2)
    wait_until_settled(robot, groups)

    states = robot.states()
    home_deg = {g: [math.degrees(q) for q in states[g].q] for g in groups}
    for g in groups:
        logger.info(f"[{g.name}] Home joint positions [deg]: {[round(q, 1) for q in home_deg[g]]}")

    # Alternate between home + amplitude and home - amplitude on every joint of every arm
    cycle = 0
    sign = 1.0
    while args.cycles == 0 or cycle < 2 * args.cycles:
        targets = {g: [q + sign * args.amplitude for q in home_deg[g]] for g in groups}
        robot.ExecutePrimitive(
            {
                g: flexivrdk.PrimitiveArgs("MoveJ", {"target": flexivrdk.JPos(targets[g], [0.0] * 6)})
                for g in groups
            }
        )
        wait_until_reached(robot, groups, targets)
        # Let the arms come to rest before reversing: preempting a MoveJ while the arms still
        # move commands a sudden reversal, which trips collision detection
        wait_until_settled(robot, groups)
        logger.info(f"Swing {cycle // 2 + 1}: {'+' if sign > 0 else '-'}{args.amplitude} deg reached")
        sign = -sign
        cycle += 1

    # Back to home
    robot.ExecutePrimitive(
        {g: flexivrdk.PrimitiveArgs("MoveJ", {"target": flexivrdk.JPos(home_deg[g], [0.0] * 6)}) for g in groups}
    )
    wait_until_reached(robot, groups, home_deg)
    logger.info("Back at home, done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
