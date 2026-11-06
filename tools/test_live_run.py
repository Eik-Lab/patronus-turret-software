"""Offline tests: python3 tools/test_live_run.py [--trace-driver /tmp/test-run-trace]."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from live_run import prepare, read_config, sample_resources, stop_process
from live_run_analyse import NONE, analyse, analyse_trace

TRACE_DRIVER = None
if "--trace-driver" in sys.argv:
    index = sys.argv.index("--trace-driver")
    TRACE_DRIVER = str(Path(sys.argv[index + 1]).resolve())
    del sys.argv[index:index + 2]


class RecordingTest(unittest.TestCase):
    def test_prepare_disables_actuation_and_snapshots_without_launching(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            engine = root / "existing.engine"
            engine.write_bytes(b"fixture, not an engine")
            infer = root / "infer.ini"
            infer.write_text("[property]\nmodel-engine-file=existing.engine\nonnx-file=absent.onnx\n")
            config = root / "input.ini"
            config.write_text(f"[general]\ntracking=true\n[sensor]\nenabled=true\n[commands]\nenabled=true\n"
                              f"[can_1]\ncalibrate_on_boot=true\n[rgb]\ninfer_config={infer}\n"
                              f"[mono]\ninfer_config={infer}\n")
            output = root / "run"
            original = config.read_bytes()
            manifest = prepare(config, Path(sys.executable), output, "both", 15000)
            effective = read_config(output / "system.ini")
            self.assertFalse(effective.getboolean("general", "tracking"))
            self.assertFalse(effective.getboolean("sensor", "enabled"))
            self.assertFalse(effective.getboolean("commands", "enabled"))
            self.assertFalse(effective.getboolean("can_1", "calibrate_on_boot"))
            self.assertEqual(effective["rgb"]["udp_host"], "127.0.0.1")
            self.assertEqual(effective["mono"]["udp_port"], "15001")
            self.assertEqual(manifest["application_command"][-1], "--record-only")
            self.assertEqual(manifest["state"], "prepared")
            copied = read_config(output / "rgb-infer.ini")
            self.assertNotIn("onnx-file", copied["property"])
            self.assertEqual(copied["property"]["model-engine-file"], str(engine))
            self.assertEqual(config.read_bytes(), original)
            with self.assertRaises(FileExistsError):
                prepare(config, Path(sys.executable), output, "both", 15000)

            # Omitting --mode must not silently restart RGB for a mono-only test.
            config.write_text(original.decode().replace("[general]", "[general]\nmode=mono")
                              .replace(f"[rgb]\ninfer_config={infer}",
                                       "[rgb]\nenabled=false\ninfer_config=missing-rgb.ini"))
            mono_output = root / "mono-run"
            manifest = prepare(config, Path(sys.executable), mono_output, None, 15000)
            effective = read_config(mono_output / "system.ini")
            self.assertEqual(manifest["cameras"], ["mono"])
            self.assertEqual(effective["general"]["mode"], "mono")
            self.assertFalse(effective.getboolean("rgb", "enabled"))
            self.assertTrue(effective.getboolean("mono", "enabled"))
            self.assertFalse((mono_output / "rgb-infer.ini").exists())

    def test_trace_rejects_duplicate_missing_and_unmatched_pts(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.csv"
            path.write_text("stage,monotonic_ns,pts_ns,offset,bytes\n"
                            "infer_in,1000000,10,0,4\ninfer_out,4000000,10,0,4\n"
                            "infer_in,5000000,20,1,4\ninfer_in,6000000,20,2,4\n"
                            "infer_out,7000000,20,1,4\ninfer_out,8000000,30,2,4\n"
                            f"infer_in,9000000,{NONE},3,4\n"
                            "# dropped=2 complete=0\ninfer_out,partial\n# dropped=2 comp")
            result = analyse_trace(path, 4_000_000_000)
            timing = result["matched_durations_ms"]["inference"]
            self.assertEqual(timing["samples"], 1)
            self.assertEqual(timing["p99"], 3.0)
            self.assertEqual(timing["unmatched_inputs"], 3)
            self.assertEqual(timing["unmatched_outputs"], 2)
            self.assertEqual(result["dropped_trace_records"], 2)
            self.assertFalse(result["writer_complete"])
            self.assertEqual(result["malformed_rows"], 2)
            self.assertGreater(result["stages"]["infer_out"]["tail_to_stop_s"], 3)

    def test_missing_stream_and_forced_shutdown_are_not_success(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "manifest.json").write_text(json.dumps({
                "cameras": ["rgb", "mono"], "trace": True, "state": "stopped",
                "shutdown": {"application": {"forced": True, "returncode": -9},
                             "tegrastats": {"forced": False, "returncode": -2, "signals": ["SIGINT"]}}}))
            result = analyse(root, video=False)
            self.assertIn("mono: missing pipeline trace; rebuild application", result["warnings"])
            self.assertIn("application: forced or nonzero exit; not clean shutdown", result["warnings"])
            self.assertFalse(any("tegrastats:" in warning for warning in result["warnings"]))
            self.assertEqual(result["recordings"], {})

    def test_unreadable_optional_sensor_preserves_other_samples(self):
        original = Path.read_text
        def read(path, *args, **kwargs):
            if str(path) == "/sys/class/thermal/thermal_zone1/temp":
                raise TypeError("can't concat NoneType to bytes")
            return original(path, *args, **kwargs)
        with tempfile.TemporaryDirectory() as directory:
            with patch("live_run.glob.glob", return_value=["/sys/class/thermal/thermal_zone1/temp"]):
                with patch.object(Path, "read_text", read):
                    sample = sample_resources([], Path(directory))
            self.assertIn("/proc/stat", sample)
            self.assertGreater(sample["disk_free_bytes"], 0)
            self.assertIn("TypeError", sample["read_errors"]["/sys/class/thermal/thermal_zone1/temp"])

    def test_shutdown_records_escalation_without_claiming_graceful_exit(self):
        with tempfile.TemporaryDirectory() as directory:
            ready = Path(directory) / "ready"
            code = ("import signal,time,pathlib;signal.signal(signal.SIGINT,signal.SIG_IGN);"
                    f"pathlib.Path({str(ready)!r}).touch();time.sleep(30)")
            process = subprocess.Popen([sys.executable, "-c", code], start_new_session=True)
            import time
            try:
                deadline = time.monotonic() + 5
                while not ready.exists() and time.monotonic() < deadline:
                    time.sleep(.01)
                self.assertTrue(ready.exists())
                result = stop_process(process, .05)
                self.assertTrue(result["forced"])
                self.assertEqual(result["signals"], ["SIGINT", "SIGTERM"])
                self.assertIsNotNone(result["returncode"])
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()

    @unittest.skipUnless(TRACE_DRIVER, "supply compiled --trace-driver for GStreamer test")
    def test_real_trace_drains_both_cameras_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run([TRACE_DRIVER, directory], check=True, timeout=15)
            for camera in ("rgb", "mono"):
                result = analyse_trace(Path(directory) / f"pipeline-{camera}.csv")
                self.assertTrue(result["writer_complete"])
                self.assertEqual(result["dropped_trace_records"], 0)
                self.assertEqual(len(result["stages"]), 12)
                self.assertTrue(all(s["buffers"] == 17 for s in result["stages"].values()))


if __name__ == "__main__":
    unittest.main()
