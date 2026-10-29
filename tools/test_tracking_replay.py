"""Public CLI regression checks; no hardware, numpy, or test framework required."""

import csv
import io
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from telemetry_replay import convert

BINARY = str(Path(sys.argv.pop(1)).resolve())
TELEMETRY_BINARY = str(Path(sys.argv.pop(1)).resolve())
CONFIG = """[general]
log_level=error
[can_1]
camera=rgb
[tracking_1]
filter_enabled=true
filter_max_coast_ms=200
filter_meas_sigma_px=2
filter_q_jerk=0
kp=0.002
[rgb]
detection_width=1000
detection_height=600
focal_x_px=1000
focal_y_px=500
"""


class ReplayTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = self.root / "config.ini"
        self.config.write_text(CONFIG)

    def run_replay(self, events, repeats=1):
        path = self.root / "events.txt"
        path.write_text(events)
        return subprocess.run([BINARY, str(self.config), "rgb", "0", "0", str(path), str(repeats)],
                              cwd=self.root, text=True, capture_output=True)

    def test_stationary_target_camera_motion_coasting_and_reacquisition(self):
        # Target is at (+100,-50), then camera moves (+.01,+.02) rad.
        # Effective pan includes cos(elevation); tilt displacement is 10 pixels.
        pan_shift = 10 * math.cos(.02)
        events = f"""pose 1 0 0
aim 1
frame 1 1 1 1 1 1 590 245 20 10 1
aim 1
pose 1.1 0.01 0.02
frame 1.1 1.1 1.1 1.1 2 1 {590 - pan_shift:.17g} 235 20 10 1
aim 1.1
frame 1.11 1.1 1.1 1.1 3 1 5 5 20 10 1
frame 1.15 1.15 1.15 1.15 4 1 5 5 20 10 1
frame 1.29 1.29 1.29 1.29 5 0 0 0 0 0 0
frame 1.32 1.32 1.32 1.32 6 0 0 0 0 0 0
aim 1.32
frame 1.33 1.33 1.33 1.33 7 1 690 395 20 10 1
"""
        first = self.run_replay(events)
        self.assertEqual(first.returncode, 0, first.stderr)
        repeated = self.run_replay(events, 3)
        self.assertEqual(repeated.returncode, 0, repeated.stderr)
        self.assertEqual(first.stdout, repeated.stdout)
        rows = list(csv.DictReader(io.StringIO(first.stdout)))
        self.assertTrue(all(None not in row and None not in row.values() for row in rows))
        self.assertEqual(rows[0]["present"], "0")
        self.assertAlmostEqual(float(rows[1]["u"]), 100, places=4)
        self.assertAlmostEqual(float(rows[1]["v"]), -50, places=4)
        self.assertAlmostEqual(float(rows[2]["cmd_pan"]), .2, places=5)
        self.assertAlmostEqual(float(rows[2]["cmd_tilt"]), -.1, places=5)
        self.assertAlmostEqual(float(rows[3]["u"]), 100 - pan_shift, places=3)
        self.assertAlmostEqual(float(rows[3]["v"]), -60, places=3)
        self.assertAlmostEqual(float(rows[3]["vu"]), 0, places=3)
        self.assertAlmostEqual(float(rows[3]["vv"]), 0, places=3)
        self.assertEqual(rows[5]["measured"], "0")  # Duplicate capture timestamp.
        self.assertAlmostEqual(float(rows[5]["u"]), 100 - pan_shift, places=3)
        self.assertEqual(rows[6]["measured"], "0")  # Outlier gated out.
        self.assertEqual(rows[7]["present"], "1")   # 190 ms coast survives.
        self.assertEqual(rows[8]["present"], "0")   # 220 ms coast expires.
        self.assertEqual(rows[9]["present"], "0")
        self.assertEqual(rows[10]["measured"], "1")
        self.assertAlmostEqual(float(rows[10]["u"]), 200, places=3)
        self.assertAlmostEqual(float(rows[10]["v"]), 100, places=3)
        self.assertIn("filter_step n=21", repeated.stderr)
        # One accepted existing-track update and two seeds per pass; no warm-up,
        # duplicate, rejected outlier, coast or expired-track sample belongs here.
        self.assertIn("filter_successful_update n=3 ", repeated.stderr)
        self.assertIn("filter_seed n=6 ", repeated.stderr)
        self.assertIn("aim_and_control n=12", repeated.stderr)

    def test_telemetry_shutdown_flushes_and_preserves_timing_columns(self):
        # Repetition exercises both early and late writer-thread startup.
        for attempt in range(16):
            result = subprocess.run([TELEMETRY_BINARY, str(self.root / str(attempt)),
                                     str(self.config)], cwd=self.root, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            run = Path(result.stdout.strip())
            with (run / "ticks.csv").open() as source:
                ticks = list(csv.DictReader(source))
            with (run / "frames.csv").open() as source:
                frames = list(csv.DictReader(source))
            self.assertEqual(len(ticks), 2)
            self.assertEqual(len(frames), 1)
            self.assertTrue(all(None not in row and None not in row.values() for row in ticks))
            self.assertEqual(float(ticks[0]["cmd_pan"]), .25)
            self.assertEqual(float(ticks[0]["cmd_tilt"]), -.5)
            self.assertEqual(ticks[1]["cmd_pan"], "")
            self.assertEqual(ticks[1]["cmd_tilt"], "")
            self.assertEqual(ticks[1]["aim"], "0")
            self.assertEqual(float(ticks[1]["loop_start_s"]), 1.262)
            self.assertEqual(float(ticks[0]["filter_us"]), 2.5)
            self.assertEqual(float(ticks[0]["command_done_s"]), 1.2607)
            self.assertEqual(int(ticks[0]["control_elapsed_ns"]), 499999999)
            self.assertEqual(ticks[0]["raw_detection"], "1")
            self.assertEqual(ticks[1]["estimate_available"], "0")
            self.assertEqual(ticks[0]["pan_write_status"], "1")
            self.assertEqual(ticks[0]["tilt_write_status"], "2")
            self.assertEqual(ticks[0]["tilt_write_error"], "3")
            self.assertEqual(ticks[0]["pan_stop_status"], "1")
            self.assertEqual(ticks[0]["tilt_stop_status"], "2")
            self.assertEqual(float(ticks[0]["pan_read_s"]), 1.25)
            self.assertEqual(float(ticks[0]["tilt_read_s"]), 1.251)
            expected = dict(previous_submit_us=3.25, iteration_us=13000, control_us=.125,
                            aim_compute_us=1.75, follower_us=420, command_start_s=1.26000175,
                            observation_received_s=1.259, skipped_observations=7,
                            source_frame_id=123, source_capture_s=1.23, source_probe_s=1.258,
                            source_publish_s=1.2585, bus_lock_wait_us=5.25, sdk_read_us=900.5,
                            sdk_write_us=320.75, sdk_read_calls=6, sdk_write_calls=8)
            for key, value in expected.items():
                self.assertAlmostEqual(float(ticks[0][key]), value, places=9, msg=key)
            for key in ("observation_received_s", "skipped_observations", "sdk_read_calls",
                        "sdk_write_calls", "bus_lock_wait_us", "sdk_read_us", "sdk_write_us"):
                self.assertEqual(float(ticks[1][key]), 0, key)
            self.assertEqual(ticks[1]["source_frame_id"], "123")  # Retained on a no-frame tick.
            self.assertTrue(all(None not in row and None not in row.values() for row in frames))
            self.assertEqual(float(frames[0]["published_s"]), 1.2585)
            if attempt == 0:
                converted = self.root / "control-input"
                convert(run, converted, 0, control_only=True)
                replay = self.run_replay((converted / "events.txt").read_text())
                self.assertEqual(replay.returncode, 0, replay.stderr)
                decisions = list(csv.DictReader(io.StringIO(replay.stdout)))
                self.assertEqual([r["leader_action"] for r in decisions], ["1", "2"])
                self.assertEqual([r["next_phase"] for r in decisions], ["0", "0"])
                with self.assertRaisesRegex(ValueError, "pose-fault resets"):
                    convert(run, self.root / "invalid-filter-reconstruction", 0)
                # Missing ticks can change the state; never silently reconstruct them.
                log = run / "run.txt"
                log.write_text(log.read_text().replace("dropped_ticks 0", "dropped_ticks 1"))
                with self.assertRaisesRegex(ValueError, "no dropped ticks"):
                    convert(run, self.root / "incomplete-control", 0, control_only=True)

    def test_live_performance_report_excludes_missing_times_and_separates_failed_writes(self):
        # The analyzer needs numpy; replay itself deliberately does not.
        try:
            import numpy as np
        except ImportError:
            self.skipTest("analyzer requires numpy")
        from contextlib import redirect_stdout
        from telemetry_analyse import report_performance
        ticks = {key: np.array(values, dtype=float) for key, values in dict(
            loop_us=[2000, 5000, 0, 0], command_start_s=[1, 2, 0, 4],
            command_done_s=[1.001, 2.004, 3, 3.9], aiming=[1, 1, 1, 1],
            source_capture_s=[.99, 1.98, 0, float("nan")],
            source_probe_s=[.995, 1.99, 0, 4], source_publish_s=[.996, 1.991, 0, 4],
            observation_received_s=[.997, 0, 0, 0],
            pan_write_status=[1, 1, 0, 1], tilt_write_status=[1, 2, 0, 1],
            sdk_read_calls=[2, 6, 0, 0], skipped_observations=[0, 3, 0, 0]
        ).items()}
        output = io.StringIO()
        with redirect_stdout(output):
            report_performance(ticks)
            report_performance({"loop_us": np.array([10., 20.])})  # Older columns optional.
            report_performance({})
        report = output.getvalue()
        self.assertIn("leader command call (all attempts): n=2 p50=2500.000 p99=3970.000", report)
        self.assertIn("publication -> consumption: n=1 p50=1000.000", report)
        self.assertIn("source capture -> aim command return (all attempts): n=2 p50=17500.000", report)
        self.assertIn("source capture -> aim command return (SDK writes OK): n=1 p50=11000.000", report)
        self.assertIn("aim commands with failed/unavailable SDK writes: 1", report)
        self.assertIn("sdk_read_calls: total=8", report)
        self.assertIn("sdk_read_calls per tick (count): n=4 p50=1.000", report)
        self.assertIn("skipped_observations: total=3", report)

    def test_control_transitions_and_nanosecond_boundaries(self):
        # phase/next phase: tracking=0, returning=1, home=2.
        # action: none=0, aim=1, hold=2, return=3.
        cases = [
            ("0 1 1 1 0",               (0, 0, 1, 1, 1, 0)),
            ("449999999 0 1 1 0",       (0, 0, 1, 1, 1, 0)),
            ("450000000 0 1 1 0",       (0, 0, 2, 2, 0, 0)),
            ("499999999 0 0 1 0",       (0, 0, 2, 2, 0, 0)),
            ("500000000 0 0 1 1",       (0, 1, 2, 2, 0, 0)),
            ("510000000 0 0 1 0",       (1, 1, 3, 3, 0, 0)),
            # Raw detection interrupts return even with no accepted estimate.
            ("520000000 1 0 1 1",       (0, 0, 2, 2, 0, 0)),
            ("970000000 0 1 1 0",       (0, 0, 2, 2, 0, 0)),
            ("1020000000 0 0 1 0",      (0, 1, 2, 2, 0, 0)),
            ("1030000000 0 0 1 1",      (1, 2, 3, 3, 0, 1)),
            # Followers continue parking while the leader holds at home.
            ("1040000000 0 0 1 0",      (2, 2, 2, 3, 0, 0)),
            ("1050000000 1 1 1 0",      (0, 0, 1, 1, 1, 0)),
        ]
        events = "\n".join("control " + row for row, _ in cases)
        first = self.run_replay(events)
        self.assertEqual(first.returncode, 0, first.stderr)
        repeated = self.run_replay(events, 3)
        self.assertEqual(repeated.returncode, 0, repeated.stderr)
        self.assertEqual(first.stdout, repeated.stdout)
        fields = ("phase", "next_phase", "leader_action", "follower_action", "aiming", "stop_after_home")
        rows = list(csv.DictReader(io.StringIO(first.stdout)))
        self.assertEqual(len(rows), len(cases))
        for row, (event, expected) in zip(rows, cases):
            with self.subTest(event=event):
                self.assertNotIn(None, row)
                self.assertEqual(tuple(int(row[key]) for key in fields), expected)
        self.assertIn("control_state n=36", repeated.stderr)

    def test_control_overlay_raw_mode_and_disabled_home(self):
        result = self.run_replay("control 0 1 1 0 0\ncontrol 900000000 0 0 0 1")
        self.assertEqual(result.returncode, 0, result.stderr)
        rows = list(csv.DictReader(io.StringIO(result.stdout)))
        self.assertEqual([r["aiming"] for r in rows], ["1", "0"])
        self.assertTrue(all(r["next_phase"] == "0" and r["leader_action"] == "0"
                            and r["follower_action"] == "0" for r in rows))
        self.config.write_text(CONFIG.replace("filter_enabled=true", "filter_enabled=false\nhome_return_enabled=false"))
        result = self.run_replay("control 0 1 0 1 0\ncontrol 1 0 1 1 0\ncontrol 900000000 0 0 1 1")
        self.assertEqual(result.returncode, 0, result.stderr)
        rows = list(csv.DictReader(io.StringIO(result.stdout)))
        self.assertEqual([r["leader_action"] for r in rows], ["1", "2", "2"])
        self.assertTrue(all(r["next_phase"] == "0" and r["stop_after_home"] == "0" for r in rows))

    def test_control_pose_fault_and_recovery(self):
        # No motion or home completion while feedback is unusable, even with a target.
        result = self.run_replay("control 0 1 1 1 0 0\n"
                                 "control 10000000 1 1 1 0 1\n"
                                 "control 510000000 0 0 1 0 1\n"
                                 "control 520000000 0 0 1 1 0\n"
                                 "control 530000000 0 0 1 0 1\n"
                                 "control 540000000 0 0 1 1 1")
        self.assertEqual(result.returncode, 0, result.stderr)
        rows = list(csv.DictReader(io.StringIO(result.stdout)))
        self.assertEqual([r["leader_action"] for r in rows], ["2", "1", "2", "2", "3", "3"])
        self.assertEqual([r["next_phase"] for r in rows], ["0", "0", "1", "1", "1", "2"])
        self.assertEqual([r["stop_after_home"] for r in rows], ["0", "0", "0", "0", "0", "1"])
        self.assertEqual(rows[0]["follower_action"], "2")
        self.assertEqual(rows[0]["aiming"], "0")

    def test_invalid_input_fails_without_partial_output(self):
        for events in ("", "pose 2 0 0\naim 1\n", "frame 1 1\n", "aim nan\n", "aim 1 extra\n",
                       "control 0 2 0 1 0", "control -1 0 0 1 0", "control 0 1 0 1",
                       "control 0 1 1 1 0 2", "control 0 1 1 1 0 garbage",
                       "control 9007199254740993 0 0 1 0\ncontrol 9007199254740992 0 0 1 0"):
            with self.subTest(events=events):
                result = self.run_replay(events)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(result.stdout, "")

    def test_telemetry_conversion_preserves_camera_pose_and_missing_frames(self):
        run = self.root / "run"
        run.mkdir()
        recorded_config = CONFIG.replace("[tracking_1]", "# --- Control law (PD + lead) ---\n[tracking_1]")
        (run / "run.txt").write_text(
            f"--- config system.ini ---\n{recorded_config}\n--- end config ---\n"
            "gimbal 0 camera rgb home_tilt 0\n"
            "end steady_s 3 dropped_frames 0 dropped_ticks 0\n")
        (run / "ticks.csv").write_text(
            "gimbal,t_s,driving,pan,tilt,aim,aim_s\n"
            "0,1,1,0,0,1,1.02\n0,1.1,1,0.01,0.02,1,1.12\n"
            "1,1,1,10,10,1,1.02\n")
        (run / "frames.csv").write_text(
            "gimbal,step_s,timestamp_s,capture_s,captured_at_s,frame_id,det,det_cx,det_cy,det_w,det_h,det_conf\n"
            "0,1.01,1,1,1,1,1,600,250,20,10,1\n"
            "0,1.11,1.1,1.1,1.1,2,0,,,,,\n")
        output = self.root / "converted"
        convert(run, output, 0)
        self.assertEqual((output / "config.ini").read_text().strip(), recorded_config.strip())
        result = self.run_replay((output / "events.txt").read_text())
        self.assertEqual(result.returncode, 0, result.stderr)
        rows = list(csv.DictReader(io.StringIO(result.stdout)))
        self.assertEqual([row["event"] for row in rows], ["frame", "aim", "frame", "aim"])
        self.assertEqual(rows[2]["measured"], "0")
        # Aim is 20 ms beyond the newest pose: extrapolate its measured rate.
        self.assertAlmostEqual(float(rows[3]["u"]), 100 - 12 * math.cos(.02), places=3)
        self.assertAlmostEqual(float(rows[3]["v"]), -62, places=3)


if __name__ == "__main__":
    unittest.main()
