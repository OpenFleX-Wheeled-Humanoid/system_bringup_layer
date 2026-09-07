#!/usr/bin/env python3

import pathlib
import unittest


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
HEADER = (PACKAGE / "include/swerve_base_panel/swerve_base_panel.hpp").read_text()
SOURCE = (PACKAGE / "src/swerve_base_panel.cpp").read_text()


class KeyboardControlContractTest(unittest.TestCase):
    def test_complete_multi_key_state_and_s_curve_are_present(self):
        self.assertIn("std::set<int> pressed_keyboard_keys_;", HEADER)
        self.assertIn("SCurveState keyboard_vx_;", HEADER)
        self.assertIn("SCurveState keyboard_vy_;", HEADER)
        self.assertIn("SCurveState keyboard_wz_;", HEADER)
        for key in ("W", "S", "A", "D", "Up", "Down", "Left", "Right", "Q", "E"):
            self.assertIn(f"Qt::Key_{key}", SOURCE)

    def test_axes_are_composed_independently(self):
        self.assertIn("(forward - backward) * linear_speed_", SOURCE)
        self.assertIn("(left - right) * linear_speed_", SOURCE)
        self.assertIn("(rotate_left - rotate_right) * angular_speed_", SOURCE)
        self.assertIn("updateSCurve(\n    keyboard_vx_", SOURCE)
        self.assertIn("updateSCurve(\n    keyboard_vy_", SOURCE)
        self.assertIn("updateSCurve(\n    keyboard_wz_", SOURCE)

    def test_manual_speed_caps_are_one(self):
        self.assertIn("kMaxLinearSpeed = 1.00", HEADER)
        self.assertIn("kMaxAngularSpeed = 1.00", HEADER)
        self.assertIn("kMaxSpeedPercent = 500", HEADER)

    def test_stop_and_input_handover_clear_stale_state(self):
        self.assertIn("if (command_active_) {\n    return true;\n  }", SOURCE)
        self.assertGreaterEqual(SOURCE.count("clearKeyboardMotion();"), 3)
        self.assertGreaterEqual(SOURCE.count("resetKeyboardSmoothing();"), 3)
        self.assertIn(
            "stopActiveCommand(QStringLiteral(\"Emergency stop\"));",
            SOURCE,
        )


if __name__ == "__main__":
    unittest.main()
