import subprocess
import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))

import server  # noqa: E402


class ValidateTestCommandTest(unittest.TestCase):
    def test_allows_test_page_commands(self):
        for command in [
            "TEST_MODE ON",
            "TEST_MODE OFF",
            "TEST_JOG 5 0",
            "TEST_JOG -1.5 0 600",
            "TEST_A 400 600",
            "TEST_B -80",
            "SERVO_STATUS 1",
            "SERVO_OFF 2",
            "SERVO_ANGLE 1 90",
            "SERVO_ANGLE 2 359.5",
            "SERVO_TURN 1 -3",
            "SERVO_TURN 2 0.25",
            "POS",
            "ALARM_CLEAR",
        ]:
            with self.subTest(command=command):
                expect, timeout_s = server.validate_test_command(command)
                self.assertTrue(expect)
                self.assertGreater(timeout_s, 0)

    def test_rejects_other_commands(self):
        for command in [
            "XY 10 10 600",
            "HOME",
            "G1 X10",
            "SERVO_TURN 3 1",
            "SERVO_ANGLE 1",
            "TEST_MODE",
            "TEST_JOG 5",
            "POS; HOME",
            "JOB_BEGIN",
        ]:
            with self.subTest(command=command):
                with self.assertRaises(ValueError):
                    server.validate_test_command(command)

    def test_servo_move_waits_for_its_own_result(self):
        expect, timeout_s = server.validate_test_command("SERVO_TURN 1 3")
        self.assertEqual(expect, "SERVO_TURN id=")
        self.assertGreaterEqual(timeout_s, 30)


class ConsoleCommandExpectTest(unittest.TestCase):
    def test_xy_waits_for_ack_or_nack(self):
        self.assertEqual(server.console_command_expect("XY 11.000 10.000 900"), "_XY target=")
        self.assertIn(server.console_command_expect("XY 1 2"), "ACK_XY target=(1.000,2.000)")
        self.assertIn(server.console_command_expect("XY 1 2"), "NACK_XY target=(1.000,2.000) reason=rejected")

    def test_other_commands_keep_default(self):
        for command in ["POS", "HOME", "PENUP", "XYZ", "ALARM_CLEAR"]:
            with self.subTest(command=command):
                self.assertEqual(server.console_command_expect(command), "")


class SyncMotionStateTest(unittest.TestCase):
    ARGS = ["--csv", "x.csv", "--port", "/dev/null", "--baud", "115200"]

    def set_motion(self, motion):
        with server.state_lock:
            server.state["machine"] = dict(server.state["machine"], motion=motion)
            server.state["jobRunning"] = False

    def test_stuck_motion_is_cleared_by_pos(self):
        self.set_motion(True)
        pos = subprocess.CompletedProcess([], 0, stdout="POS X=1.000 Y=0.000 MOTION=NO\n", stderr="")
        with mock.patch.object(server.subprocess, "run", return_value=pos) as run:
            server.sync_motion_state(self.ARGS)
        run.assert_called_once()
        self.assertFalse(server.snapshot_state()["machine"]["motion"])

    def test_no_pos_when_motion_already_idle(self):
        self.set_motion(False)
        with mock.patch.object(server.subprocess, "run") as run:
            server.sync_motion_state(self.ARGS)
        run.assert_not_called()


class UpdateTestStateTest(unittest.TestCase):
    def setUp(self):
        with server.state_lock:
            server.state["machine"] = dict(server.state["machine"], testMode=False)
            server.state["lastTestResult"] = ""
            server.state["servos"] = {"1": {}, "2": {}}

    def test_pos_line_updates_test_mode_and_raw_limits(self):
        server.update_test_state_from_log(
            "POS X=0.000 Y=0.000 A=0 B=0 F=0.000 EN=HARDWIRED_ACTIVE HOMED=NO "
            "LIMIT_X=OPEN LIMIT_Y=ACTIVE LIMIT_X_RAW=OPEN LIMIT_Y_RAW=ACTIVE TEST_MODE=ON"
        )
        machine = server.snapshot_state()["machine"]
        self.assertTrue(machine["testMode"])
        self.assertEqual(machine["limitsRaw"], {"x": "OPEN", "y": "ACTIVE"})

    def test_test_mode_result_line(self):
        server.update_test_state_from_log("TEST_MODE result=ON (moves clear HOMED)")
        self.assertTrue(server.snapshot_state()["machine"]["testMode"])
        server.update_test_state_from_log("TEST_MODE result=OFF homed=NO")
        self.assertFalse(server.snapshot_state()["machine"]["testMode"])

    def test_jog_result_is_recorded(self):
        line = "TEST_JOG result=STOPPED_LIMIT dx=-5.000 dy=0.000 F=600 X=-2.1 Y=0.0 limit=X"
        server.update_test_state_from_log(line)
        self.assertEqual(server.snapshot_state()["lastTestResult"], line)

    def test_alarm_clear_is_recorded(self):
        server.update_test_state_from_log("ALARM_CLEAR complete")
        self.assertEqual(server.snapshot_state()["lastTestResult"], "ALARM_CLEAR complete")

    def test_servo_status_and_result(self):
        server.update_test_state_from_log(
            "SERVO_STATUS id=1 result=OK flags=0x00 mode=0 torque=1 raw=99 goal=100 "
            "limits=0/0 tracking=YES abs=4195 turns=1.02 voltage=7.9 temp=28 moving=0"
        )
        server.update_test_state_from_log(
            "SERVO_TURN id=2 value=1.00 result=PASS target=4196 actual=4195 error=-1 ms=5500"
        )
        servos = server.snapshot_state()["servos"]
        self.assertEqual(servos["1"]["raw"], "99")
        self.assertEqual(servos["1"]["tracking"], "YES")
        self.assertEqual(servos["2"]["lastResult"], "SERVO_TURN PASS")


if __name__ == "__main__":
    unittest.main()
