#!/usr/bin/env python3
"""Serial JOG / STOP acceptance check (CoreXY_UFO_Game HANDOFF_FIRMWARE_JOG §4).

JOG must be resent every 50 ms while held, which serial_send.py (one command,
wait, next) cannot do. This script streams JOG from a sender loop while a
reader thread collects the firmware log, then checks the expected lines.

The machine moves. Clear the work area; HOME runs unless --skip-home.
See tools/serial_tool/docs/jog-check.md.
"""

from __future__ import annotations

import argparse
import re
import sys
import threading
import time
from dataclasses import dataclass, field

JOG_PERIOD_S = 0.05
SERIAL_JOG_TIMEOUT_S = 0.25
X_MAX_MM = 390.0
Y_MAX_MM = 200.0
ALL_TESTS = ("8", "1", "2", "5", "3", "6", "7", "11", "4")

FLOAT = r"(-?\d+(?:\.\d+)?)"


@dataclass
class Line:
    t: float
    text: str


@dataclass
class Log:
    lines: list[Line] = field(default_factory=list)
    lock: threading.Lock = field(default_factory=threading.Lock)

    def add(self, text: str) -> None:
        with self.lock:
            self.lines.append(Line(time.monotonic(), text))

    def since(self, t0: float) -> list[Line]:
        with self.lock:
            return [line for line in self.lines if line.t >= t0]


class Firmware:
    def __init__(self, port: str, baud: int, echo: bool) -> None:
        import serial  # type: ignore[import-not-found]

        self.serial = serial.Serial(port, baud, timeout=0.05)
        self.log = Log()
        self.echo = echo
        self.running = True
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self) -> None:
        buffer = b""
        while self.running:
            buffer += self.serial.read(512)
            while b"\n" in buffer:
                raw, buffer = buffer.split(b"\n", 1)
                text = raw.decode(errors="replace").strip()
                if text:
                    self.log.add(text)
                    if self.echo:
                        print(f"  < {text}")

    def send(self, command: str) -> float:
        self.serial.write((command + "\n").encode())
        return time.monotonic()

    def wait_for(self, pattern: str, t0: float, timeout_s: float) -> Line | None:
        regex = re.compile(pattern)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for line in self.log.since(t0):
                if regex.search(line.text):
                    return line
            time.sleep(0.01)
        return None

    def jog(self, command: str, duration_s: float, extra=None) -> float:
        """Send command every 50 ms for duration_s. extra(i) may send more lines."""
        started = time.monotonic()
        index = 0
        while time.monotonic() - started < duration_s:
            self.send(command)
            if extra is not None:
                extra(index)
            index += 1
            time.sleep(JOG_PERIOD_S)
        return time.monotonic()

    def pos(self) -> dict[str, str]:
        t0 = self.send("POS")
        line = self.wait_for(r"^POS X=", t0, 2.0)
        if line is None:
            return {}
        return dict(re.findall(r"(\w+)=(\"[^\"]*\"|\S+)", line.text))

    def close(self) -> None:
        self.running = False
        self.reader.join(timeout=1.0)
        self.serial.close()


class Checker:
    def __init__(self) -> None:
        self.results: list[tuple[str, bool, str]] = []

    def check(self, name: str, ok: bool, detail: str = "") -> bool:
        self.results.append((name, ok, detail))
        print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}")
        return ok

    def summary(self) -> int:
        failed = [result for result in self.results if not result[1]]
        print(f"\n{len(self.results) - len(failed)}/{len(self.results)} checks passed")
        return 1 if failed else 0


def jog_end(fw: Firmware, t0: float, timeout_s: float = 2.0) -> Line | None:
    return fw.wait_for(r"^JOG end reason=", t0, timeout_s)


def field_value(text: str, name: str) -> float | None:
    match = re.search(rf"\b{name}={FLOAT}", text)
    return float(match.group(1)) if match else None


def move_to(fw: Firmware, x_mm: float, y_mm: float, feed: float = 3000.0) -> bool:
    t0 = fw.send(f"XY {x_mm:.3f} {y_mm:.3f} {feed:.0f}")
    return fw.wait_for(r"^ACK_XY|^NACK_XY", t0, 20.0) is not None


def test_not_homed(fw: Firmware, c: Checker) -> None:
    if fw.pos().get("HOMED") != "NO":
        c.check("8 not_homed", True, "(skipped: already HOMED; power-cycle to run)")
        return
    t0 = fw.send("JOG 50 0")
    c.check("8 not_homed", fw.wait_for(r"^NACK_JOG reason=not_homed", t0, 1.0) is not None)
    fw.send("JOG 0 0")


def test_jog_zero(fw: Firmware, c: Checker) -> None:
    start_x = float(fw.pos().get("X", "0"))
    t0 = time.monotonic()
    fw.jog("JOG 50 0", 2.0)
    fw.send("JOG 0 0")
    end = jog_end(fw, t0)
    ok = end is not None and "reason=zero" in end.text and "HOMED=YES" in end.text
    moved = (field_value(end.text, "X") or 0.0) - start_x if end else 0.0
    c.check("1 JOG 50 0 for 2 s, JOG 0 0", ok and 90.0 <= moved <= 110.0,
            f"dX={moved:.1f}mm line={end.text if end else None}")


def test_jog_timeout(fw: Firmware, c: Checker) -> None:
    t0 = time.monotonic()
    last = fw.jog("JOG -50 0", 1.0)
    end = jog_end(fw, t0)
    delay_s = end.t - last if end else 99.0
    c.check("2 deadman timeout", end is not None and "reason=timeout" in end.text and delay_s < 0.6,
            f"end {delay_s * 1000:.0f}ms after last JOG")


def test_reject_and_pos(fw: Firmware, c: Checker) -> None:
    t0 = time.monotonic()

    def extra(index: int) -> None:
        if index == 10:
            fw.send("XY 100 100")
        if index == 15:
            fw.send("POS")

    fw.jog("JOG 20 0", 1.5, extra)
    fw.send("JOG 0 0")
    nack = fw.wait_for(r"^NACK_XY .*reason=jog_active", t0, 0.1)
    pos = fw.wait_for(r"^POS X=.*MOTION=YES.*", t0, 0.1)
    end = jog_end(fw, t0)
    c.check("5 XY during jog -> NACK jog_active", nack is not None and end is not None and
            "reason=zero" in end.text)
    c.check("5b POS during jog", pos is not None and "HOMED=YES" in pos.text and
            (pos.t < end.t if end else False))
    time.sleep(1.0)
    c.check("5 XY not executed after jog", fw.wait_for(r"^ACK_XY target=\(100", t0, 0.1) is None)


def test_soft_limit(fw: Firmware, c: Checker) -> None:
    y_mm = min(Y_MAX_MM - 20.0, 100.0)
    if not move_to(fw, X_MAX_MM - 10.0, y_mm):
        c.check("3 soft limit", False, "setup XY failed")
        return
    t0 = time.monotonic()
    fw.jog("JOG 100 0", 1.5)
    fw.send("JOG 0 0")
    jog_end(fw, t0)
    xs = [field_value(line.text, "X") for line in fw.log.since(t0) if line.text.startswith("JOG_POS")]
    xs = [x for x in xs if x is not None]
    max_x = max(xs) if xs else 0.0
    c.check("3 soft limit X<=390", bool(xs) and max_x <= X_MAX_MM and max_x >= X_MAX_MM - 1.0,
            f"max X={max_x:.3f}")


def test_stop_xy(fw: Firmware, c: Checker) -> None:
    pos = fw.pos()
    y_mm = float(pos.get("Y", "100"))
    t0 = fw.send(f"XY 20 {y_mm:.3f} 3000")
    time.sleep(0.5)
    fw.send("STOP")
    nack = fw.wait_for(r"^NACK_XY .*reason=stopped", t0, 3.0)
    done = fw.wait_for(r"^STOP complete .*HOMED=YES", t0, 3.0)
    after = fw.pos()
    c.check("6 STOP during XY", nack is not None and done is not None and after.get("ALARM") == "NO",
            done.text if done else "")
    c.check("6 XY accepted after STOP", move_to(fw, 50.0, 50.0) and
            fw.wait_for(r"^ACK_XY target=\(50", t0, 0.1) is not None)


def test_log_flood(fw: Firmware, c: Checker) -> None:
    t0 = time.monotonic()
    pos_sent = 0

    def extra(index: int) -> None:
        nonlocal pos_sent
        if index % 5 == 0:
            fw.send("POS")
            pos_sent += 1

    fw.jog("JOG 10 10", 5.0, extra)
    fw.jog("JOG -10 -10", 5.0, extra)
    fw.send("JOG 0 0")
    end = jog_end(fw, t0)
    lines = fw.log.since(t0)
    jog_pos = [line.t for line in lines if line.text.startswith("JOG_POS")]
    pos = sum(line.text.startswith("POS X=") for line in lines)
    rate_hz = (len(jog_pos) - 1) / (jog_pos[-1] - jog_pos[0]) if len(jog_pos) > 1 else 0.0
    c.check("7 JOG_POS ~20 Hz", 18.0 <= rate_hz <= 22.0,
            f"JOG_POS={len(jog_pos)} rate={rate_hz:.1f}Hz")
    c.check("7 every POS answered", pos == pos_sent, f"POS={pos}/{pos_sent}")
    c.check("7 session not interrupted", end is not None and "reason=zero" in end.text)
    t1 = fw.send("XY 60 60 3000")
    c.check("7 ACK_XY after 10 s jog", fw.wait_for(r"^ACK_XY target=\(60", t1, 10.0) is not None)


def test_servo(fw: Firmware, c: Checker) -> None:
    t0 = time.monotonic()
    fw.jog("JOG 0 0 400 0", 1.0)
    fw.send("JOG 0 0")
    end = jog_end(fw, t0)
    failed = fw.wait_for(r"^JOG servo id=1 (disabled|stopped)", t0, 0.1)
    c.check("11 Z servo jog without TEST_MODE", end is not None and failed is None,
            failed.text if failed else "")


def test_abort(fw: Firmware, c: Checker) -> None:
    t0 = time.monotonic()

    def extra(index: int) -> None:
        if index == 10:
            fw.send("ABORT")

    fw.jog("JOG 20 0", 1.0, extra)
    end = jog_end(fw, t0)
    pos = fw.pos()
    c.check("4 ABORT during jog", end is not None and "reason=abort" in end.text and
            pos.get("ALARM") == "YES" and pos.get("HOMED") == "NO")
    fw.send("ALARM_CLEAR")


TESTS = {
    "8": test_not_homed,
    "1": test_jog_zero,
    "2": test_jog_timeout,
    "5": test_reject_and_pos,
    "3": test_soft_limit,
    "6": test_stop_xy,
    "7": test_log_flood,
    "11": test_servo,
    "4": test_abort,
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--startup-delay", type=float, default=4.0)
    parser.add_argument("--tests", default=",".join(ALL_TESTS),
                        help=f"Comma separated, run in this order. Default: {','.join(ALL_TESTS)}")
    parser.add_argument("--skip-home", action="store_true", help="Do not send HOME before test 1.")
    parser.add_argument("--echo", action="store_true", help="Print every firmware line.")
    args = parser.parse_args()

    selected = [name.strip() for name in args.tests.split(",") if name.strip()]
    unknown = [name for name in selected if name not in TESTS]
    if unknown:
        parser.error(f"unknown tests: {unknown}")

    fw = Firmware(args.port, args.baud, args.echo)
    checker = Checker()
    try:
        time.sleep(args.startup_delay)
        homed = False
        for name in selected:
            if name != "8" and not homed:
                if not args.skip_home:
                    t0 = fw.send("HOME")
                    if fw.wait_for(r"^HOME complete", t0, 60.0) is None:
                        checker.check("HOME", False, "no HOME complete")
                        break
                move_to(fw, 100.0, 100.0)
                homed = True
            print(f"--- test {name}")
            TESTS[name](fw, checker)
            time.sleep(0.5)
    finally:
        fw.send("JOG 0 0")
        fw.close()
    return checker.summary()


if __name__ == "__main__":
    sys.exit(main())
