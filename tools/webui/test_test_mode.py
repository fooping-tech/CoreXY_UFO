import sys
import unittest
from pathlib import Path

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
