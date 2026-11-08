#!/usr/bin/env python3
"""Servo-only power cycle check (CoreXY_UFO_Game HANDOFF_FIRMWARE_Z_SERVO, Issue #11).

The STS3020 loses multi-turn mode (limits 0/0 -> 0/4095) when only the servo is
power cycled. This script checks that SERVO_STATUS then says tracking=LOST and
that the next JOG re-enables multi-turn (tracking=RESET) and really moves.

Interactive: it asks you to power cycle the servo. The servo moves; the XY axes
do not. See tools/serial_tool/docs/servo-power-cycle-check.md.
"""

from __future__ import annotations

import argparse
import re
import sys
import time

from jog_check import Checker, Firmware

STATUS_FIELDS = re.compile(r"(\w+)=(\S+)")


def servo_status(fw: Firmware, servo_id: int) -> dict[str, str]:
    t0 = fw.send(f"SERVO_STATUS {servo_id}")
    line = fw.wait_for(rf"^SERVO_STATUS id={servo_id} ", t0, 2.0)
    if line is None:
        return {}
    print(f"  {line.text}")
    return dict(STATUS_FIELDS.findall(line.text))


def jog_servo(fw: Firmware, servo_id: int, rate: int, duration_s: float) -> tuple[str, str]:
    """Jogs one servo; returns (begin/disabled line, end line)."""
    z, arm = (rate, 0) if servo_id == 1 else (0, rate)
    t0 = time.monotonic()
    fw.jog(f"JOG 0 0 {z} {arm}", duration_s)
    fw.send("JOG 0 0")
    end = fw.wait_for(r"^JOG end", t0, 3.0)
    begin = fw.wait_for(rf"^JOG servo id={servo_id} (begin_ms|disabled)", t0, 0.1)
    failed = fw.wait_for(rf"^JOG servo id={servo_id} (disabled|stopped)", t0, 0.1)
    for line in (begin, failed, end):
        if line is not None:
            print(f"  {line.text}")
    return (failed or begin).text if (failed or begin) else "", end.text if end else ""


def abs_counts(status: dict[str, str]) -> int | None:
    try:
        return int(status["abs"])
    except (KeyError, ValueError):
        return None


def prompt(message: str) -> None:
    input(f"\n>>> {message} (Enter) ")


def run(fw: Firmware, c: Checker, servo_id: int, rate: int, duration_s: float,
        servo_turn: bool) -> None:
    expected = abs(rate) * duration_s
    print(f"--- servo id={servo_id}: establish tracking")
    begin, _ = jog_servo(fw, servo_id, rate, duration_s)
    c.check("1 first JOG starts", "begin_ms=" in begin, begin)
    status = servo_status(fw, servo_id)
    c.check("2 SERVO_STATUS tracking=YES limits=0/0",
            status.get("tracking") == "YES" and status.get("limits") == "0/0")

    prompt(f"Power cycle ONLY servo id={servo_id} (keep the Core2 on), then wait 2 s")
    status = servo_status(fw, servo_id)
    c.check("3 SERVO_STATUS says tracking=LOST", status.get("tracking") == "LOST" and
            status.get("limits") not in (None, "0/0"), f"limits={status.get('limits')}")

    print("--- JOG after the power cycle")
    begin, end = jog_servo(fw, servo_id, rate, duration_s)
    c.check("4 JOG re-enables multi-turn (tracking=RESET)",
            "begin_ms=" in begin and "tracking=RESET" in begin, begin)
    before = servo_status(fw, servo_id)
    c.check("5 tracking=YES limits=0/0 again",
            before.get("tracking") == "YES" and before.get("limits") == "0/0")

    print("--- JOG again: tracking continues")
    begin, _ = jog_servo(fw, servo_id, rate, duration_s)
    after = servo_status(fw, servo_id)
    moved = None
    if abs_counts(before) is not None and abs_counts(after) is not None:
        moved = abs_counts(after) - abs_counts(before)
    c.check("6 second JOG keeps tracking (tracking=KEPT)", "tracking=KEPT" in begin, begin)
    c.check("7 servo really moved", moved is not None and abs(moved) >= expected * 0.5 and
            (moved > 0) == (rate > 0), f"moved={moved} expected~{expected:+.0f}")

    if not servo_turn:
        return
    prompt(f"Power cycle ONLY servo id={servo_id} again for the SERVO_TURN check")
    t0 = fw.send("TEST_MODE ON")
    fw.wait_for(r"^TEST_MODE result=", t0, 2.0)
    turns = 0.05 if rate > 0 else -0.05
    t0 = fw.send(f"SERVO_TURN {servo_id} {turns}")
    first = fw.wait_for(rf"^SERVO_TURN id={servo_id} ", t0, 5.0)
    t0 = fw.send(f"SERVO_TURN {servo_id} {turns}")
    second = fw.wait_for(rf"^SERVO_TURN id={servo_id} ", t0, 10.0)
    for line in (first, second):
        if line is not None:
            print(f"  {line.text}")
    c.check("8 SERVO_TURN unchanged: rejects once with multi_turn_lost",
            first is not None and "result=REJECTED" in first.text and
            "multi_turn_lost" in first.text)
    c.check("9 SERVO_TURN retry passes", second is not None and "result=PASS" in second.text)
    t0 = fw.send("TEST_MODE OFF")
    fw.wait_for(r"^TEST_MODE result=", t0, 2.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--startup-delay", type=float, default=4.0)
    parser.add_argument("--id", type=int, choices=(1, 2), default=1,
                        help="1 = Z drum, 2 = arm")
    parser.add_argument("--rate", type=int, default=-800,
                        help="JOG rate [counts/s]; sign = direction. Default -800")
    parser.add_argument("--duration", type=float, default=1.5, help="Each JOG [s]")
    parser.add_argument("--servo-turn", action="store_true",
                        help="Also check SERVO_TURN after a second power cycle")
    parser.add_argument("--echo", action="store_true", help="Print every firmware line.")
    args = parser.parse_args()

    fw = Firmware(args.port, args.baud, args.echo)
    checker = Checker()
    try:
        time.sleep(args.startup_delay)
        run(fw, checker, args.id, args.rate, args.duration, args.servo_turn)
    finally:
        fw.send("JOG 0 0")
        fw.close()
    return checker.summary()


if __name__ == "__main__":
    sys.exit(main())
