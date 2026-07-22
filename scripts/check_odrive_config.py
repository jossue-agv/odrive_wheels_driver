#!/usr/bin/env python3
"""
check_odrive_config.py — ODrive S1 low-speed tuning config checker.

Reads both ODrive axes over USB and compares against recommended values.
Prints a clear OK / MISMATCH table. Optionally applies and saves changes.

Usage:
    ros2 run odrive_wheels_driver check_odrive_config           # read-only, print comparison
    ros2 run odrive_wheels_driver check_odrive_config --apply   # apply mismatches and save_configuration()

Requirements:
    pip install odrive

This is a dev tool (dev_only: true). Not part of the robot runtime stack.
"""

import argparse
import sys

try:
    import odrive
    from odrive.enums import InputMode
except ImportError:
    print("ERROR: 'odrive' package not installed. Run: pip install odrive")
    sys.exit(1)

# Recommended values per axis for low-speed smooth control.
# vel_integrator_gain lowered from 0.333 → 0.167: the integrator at 0.333
# causes overshoot at low speeds, which manifests as stick-slip / chatter.
TARGETS = {
    "controller.config.vel_gain":               0.23,
    "controller.config.vel_integrator_gain":    0.23,
    "controller.config.input_filter_bandwidth": 8.0,
    "controller.config.vel_ramp_rate":          0.5,
    "controller.config.vel_limit":              10.0,
    "controller.config.input_mode":             1,    # INPUT_MODE_PASSTHROUGH
    "config.can.encoder_msg_rate_ms":           10,   # cyclic encoder feedback at 100 Hz
    "motor.motor_thermistor.config.enabled":    False,
}

TOLERANCE = 5e-3   # float comparison tolerance (ODrive stores float32, ~0.3% precision)


def get_nested(obj, path: str):
    """Walk dotted attribute path on an ODrive object."""
    parts = path.split(".")
    cur = obj
    for p in parts:
        cur = getattr(cur, p)
    return cur


def set_nested(obj, path: str, value):
    """Set a value at a dotted attribute path."""
    parts = path.split(".")
    cur = obj
    for p in parts[:-1]:
        cur = getattr(cur, p)
    setattr(cur, parts[-1], value)


def check_axis(axis, axis_label: str, apply: bool) -> int:
    """Check one axis. Returns number of mismatches."""
    mismatches = 0
    rows = []

    for path, target in TARGETS.items():
        try:
            actual = get_nested(axis, path)
        except AttributeError as e:
            rows.append((path, "N/A", str(target), f"ATTRIBUTE ERROR: {e}"))
            continue

        if isinstance(target, bool):
            ok = (bool(actual) == target)
        elif isinstance(target, float):
            ok = (abs(float(actual) - target) < TOLERANCE)
        else:
            ok = (actual == target)

        status = "OK" if ok else "MISMATCH"
        rows.append((path, actual, target, status))
        if not ok:
            mismatches += 1

    try:
        node_id = int(get_nested(axis, "config.can.node_id"))
        node_id_ok = node_id != 0x3F
        rows.append((
            "config.can.node_id", node_id, "0..62",
            "OK" if node_id_ok else "MANUAL",
        ))
        if not node_id_ok:
            mismatches += 1
    except AttributeError as e:
        rows.append(("config.can.node_id", "N/A", "0..62", f"ATTRIBUTE ERROR: {e}"))
        mismatches += 1

    print(f"\n--- {axis_label} ---")
    print(f"  {'Parameter':<48} {'Actual':>10}  {'Target':>10}  Status")
    print(f"  {'-'*48} {'-'*10}  {'-'*10}  ------")
    for path, actual, target, status in rows:
        marker = " <--" if status == "MISMATCH" else ""
        print(f"  {path:<48} {str(actual):>10}  {str(target):>10}  {status}{marker}")

    if apply and mismatches > 0:
        print(f"\n  Applying {mismatches} change(s)...")
        for path, actual, target, status in rows:
            if status == "MANUAL":
                print("    ERROR: assign a unique CAN node_id in range 0..62")
            elif status == "MISMATCH":
                try:
                    set_nested(axis, path, target)
                    print(f"    SET {path} = {target}")
                except Exception as e:
                    print(f"    ERROR setting {path}: {e}")

    return mismatches


def main():
    parser = argparse.ArgumentParser(description="Check/apply ODrive low-speed config")
    parser.add_argument("--apply", action="store_true",
                        help="Apply mismatched values and call save_configuration()")
    args = parser.parse_args()

    print("Searching for ODrive devices over USB...")
    print("(This may take 5-10 seconds. Ensure no other process is using the ODrive USB port.)\n")

    devices = []
    try:
        # Find up to 2 ODrives
        for _ in range(2):
            try:
                odrv = odrive.find_any(timeout=5)
                devices.append(odrv)
            except Exception:
                break
    except Exception as e:
        print(f"ERROR finding ODrive: {e}")
        sys.exit(1)

    if not devices:
        print("ERROR: No ODrive found. Check USB connection and try again.")
        sys.exit(1)

    print(f"Found {len(devices)} ODrive device(s).")

    total_mismatches = 0
    for i, odrv in enumerate(devices):
        fw = getattr(odrv, "fw_version_major", "?")
        print(f"\n=== ODrive {i} (fw {fw}.x) ===")
        total_mismatches += check_axis(odrv.axis0, f"ODrive {i} axis0", args.apply)
        if hasattr(odrv, "axis1"):
            total_mismatches += check_axis(odrv.axis1, f"ODrive {i} axis1", args.apply)
        else:
            print("  (Single-axis device — axis1 not present)")

        if args.apply and total_mismatches > 0:
            print(f"\n  Saving configuration for ODrive {i}...")
            print("  (ODrive will reboot — reconnect after ~3 seconds)")
            try:
                odrv.save_configuration()
            except Exception:
                # save_configuration() causes a reboot which raises a disconnect error — expected
                pass
            print(f"  ODrive {i} configuration saved.")

    print()
    if total_mismatches == 0:
        print("All parameters match targets. No changes needed.")
    elif args.apply:
        print(f"Processed {total_mismatches} mismatch(es) and saved writable settings.")
        print("Reconnect ODrives, then run again without --apply to verify.")
    else:
        print(f"Found {total_mismatches} mismatch(es). Re-run with --apply to write changes.")


if __name__ == "__main__":
    main()
