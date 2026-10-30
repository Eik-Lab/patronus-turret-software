"""Run only against the replay-only simulator, which has no SDK transport linked."""
import csv
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

BINARY = str(Path(sys.argv.pop(1)).resolve())
# Synthetic test values only; these are not a plan for any physical mechanism.
PLAN = """[calibration]
node_id=1
datarate_mbps=1
repetitions=1
min_position_rad=-1
max_position_rad=1
max_excursion_rad=0.4
stop_margin_rad=0.1
max_observed_velocity_rad_s=0.5
settle_velocity_rad_s=0.005
sample_ms=5
max_io_ms=20
max_sample_gap_ms=50
pulse_ms=50
settle_ms=40
settle_timeout_ms=1000
max_run_ms=4000
max_torque_nm=0.1
velocity_kp=1
velocity_ki=0
velocity_kd=0
velocity_integral_max=0
amplitude_count=2
amplitude_1_rad_s=0.03
amplitude_2_rad_s=0.07
"""


class CalibrationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.plan = self.root / "plan.ini"
        self.plan.write_text(PLAN)
        self.output = self.root / "run with spaces"

    def invoke(self, *flags):
        return subprocess.run([BINARY, "--plan", str(self.plan), "--output", str(self.output), *flags],
                              capture_output=True, text=True, timeout=10)

    def events(self):
        with (self.output / "events.csv").open() as stream:
            return list(csv.DictReader(stream))

    def test_default_is_dry_and_does_not_create_output(self):
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("DRY RUN", result.stdout)
        self.assertFalse(self.output.exists())

    def test_rejects_invalid_or_missing_bounds_before_execution(self):
        for before, after in (("min_position_rad=-1", "min_position_rad=1"),
                              ("max_excursion_rad=0.4", "max_excursion_rad=nan"),
                              ("stop_margin_rad=0.1", "stop_margin_rad=0"),
                              ("node_id=1", "node_id=1.5"),
                              ("amplitude_2_rad_s=0.07", "amplitude_2_rad_s=0.5"),
                              ("max_io_ms=20", "max_io_ms=200"),
                              ("max_torque_nm=0.1\n", "")):
            with self.subTest(after=after):
                self.plan.write_text(PLAN.replace(before, after))
                result = self.invoke("--simulate")
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(self.output.exists())

    def test_complete_pulses_settle_and_zero_then_disable(self):
        result = self.invoke("--simulate")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        rows = self.events()
        self.assertIn("mode=simulation", (self.output / "run.txt").read_text())
        self.assertIn("state=complete\n", (self.output / "run.txt").read_text())
        commands = [r for r in rows if r["operation"] == "command"]
        nonzero = [r for r in commands if float(r["value"]) != 0]
        expected = [0.03, -0.03, 0.07, -0.07]
        self.assertEqual(sorted(set(int(r["pulse"]) for r in nonzero)), list(range(4)))
        for index, value in enumerate(expected):
            pulse = [r for r in nonzero if int(r["pulse"]) == index]
            self.assertTrue(all(abs(float(r["value"]) - value) < 1e-8 for r in pulse))
            first = float(pulse[0]["begin_monotonic_s"])
            stop = next(r for r in commands if int(r["pulse"]) == index and float(r["value"]) == 0)
            self.assertGreaterEqual(float(stop["begin_monotonic_s"]) - first, 0.05)
            self.assertLess(float(stop["end_monotonic_s"]) - first, 0.11)
        for row in rows:
            self.assertLessEqual(float(row["begin_monotonic_s"]), float(row["end_monotonic_s"]))
            if row["operation"] == "position":
                self.assertLess(abs(float(row["value"])), 0.4)
        self.assertEqual(float(commands[-1]["value"]), 0)
        self.assertEqual(rows[-1]["operation"], "disable")
        original = (self.output / "events.csv").read_bytes()
        self.assertNotEqual(self.invoke("--simulate").returncode, 0)
        self.assertEqual((self.output / "events.csv").read_bytes(), original)

    def test_faults_abort_and_never_resume_pulses(self):
        for fault in ("read", "write", "nonfinite", "limit", "lower-limit", "excursion",
                      "stale", "overspeed", "initial-limit", "stop", "unsettled"):
            with self.subTest(fault=fault):
                self.output = self.root / fault
                result = self.invoke("--simulate", "--fault", fault)
                self.assertNotEqual(result.returncode, 0, result.stdout)
                rows = self.events()
                self.assertIn("state=aborted", (self.output / "run.txt").read_text())
                commands = [r for r in rows if r["operation"] == "command"]
                self.assertEqual(float(commands[-1]["value"]), 0)
                self.assertEqual(rows[-1]["operation"], "disable")
                self.assertTrue(all(int(r["pulse"]) <= 0 for r in commands))
                if fault == "initial-limit":
                    self.assertFalse(any(r["operation"] == "enable" for r in rows))
                    self.assertTrue(all(float(r["value"]) == 0 for r in commands))
                if fault == "stop":
                    self.assertIn("stop_confirmed_by_sdk=0", (self.output / "run.txt").read_text())

    def test_total_deadline_ends_a_run_that_cannot_settle(self):
        self.plan.write_text(PLAN.replace("max_run_ms=4000", "max_run_ms=600"))
        result = self.invoke("--simulate", "--fault", "unsettled")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Total run deadline exceeded", result.stdout)
        self.assertEqual(self.events()[-1]["operation"], "disable")

    def test_interrupt_aborts_simulator_and_records_stop(self):
        # Long initial settling ensures this test sends no simulated nonzero command.
        self.plan.write_text(PLAN.replace("settle_ms=40", "settle_ms=500")
                            .replace("max_run_ms=4000", "max_run_ms=10000"))
        process = subprocess.Popen([BINARY, "--plan", str(self.plan), "--output", str(self.output),
                                    "--simulate"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            deadline = time.monotonic() + 3
            while not (self.output / "run.txt").exists() and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertTrue((self.output / "run.txt").exists())
            time.sleep(.05)
            process.send_signal(signal.SIGINT)
            stdout, stderr = process.communicate(timeout=3)
            self.assertEqual(process.returncode, 1, stdout + stderr)
            self.assertIn("Interrupted", stdout)
            rows = self.events()
            self.assertEqual(rows[-1]["operation"], "disable")
            self.assertTrue(all(float(r["value"]) == 0 for r in rows if r["operation"] == "command"))
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate()


if __name__ == "__main__":
    unittest.main()
