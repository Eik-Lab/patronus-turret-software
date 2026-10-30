"""Offline encoder regression: DRIVER [--thor]. Requires GStreamer plugins and ffprobe.

Build DRIVER with tools/test_video_encoder.cpp, src/pipeline/video_encoder.cpp,
src/core/config.cpp, -Isrc and pkg-config --cflags --libs gstreamer-1.0.
"""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

BINARY = str(Path(sys.argv.pop(1)).resolve())
THOR = "--thor" in sys.argv
if THOR:
    sys.argv.remove("--thor")


class EncoderTest(unittest.TestCase):
    def run_encoder(self, config, camera, output):
        return subprocess.run([BINARY, str(config), camera, str(output)],
                              cwd=config.parent, capture_output=True, text=True, timeout=40)

    def check_stream(self, backend, camera):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            config = root / "system.ini"
            output = root / "stream.h264"
            # An invalid other camera ensures selection is per-pipeline.
            other = "mono" if camera == "rgb" else "rgb"
            config.write_text(f"[{other}]\nencoder_backend=invalid\n[{camera}]\n"
                              f"encoder_bitrate={3000 if camera == 'rgb' else 5000}\n"
                              + (f"encoder_backend={backend}\n" if backend else ""))
            run = self.run_encoder(config, camera, output)
            self.assertEqual(run.returncode, 0, run.stderr)
            probe = subprocess.run(["ffprobe", "-v", "error", "-count_frames", "-show_streams",
                                    "-show_frames", "-of", "json", str(output)],
                                   capture_output=True, text=True, check=True, timeout=30)
            self.assertEqual(probe.stderr, "")
            data = json.loads(probe.stdout)
            stream = data["streams"][0]
            self.assertEqual(stream["codec_name"], "h264")
            self.assertEqual((stream["width"], stream["height"]), (1920, 1088))
            self.assertEqual(int(stream["nb_read_frames"]), 120)
            self.assertEqual(stream["has_b_frames"], 0)
            if backend == "thor":
                self.assertEqual(stream["profile"], "High")
                self.assertEqual([i for i, f in enumerate(data["frames"]) if f["key_frame"]], [0, 60])

    def test_default_software(self):
        self.check_stream("", "rgb")

    def test_explicit_software_mono(self):
        self.check_stream("x264", "mono")

    @unittest.skipUnless(THOR, "requires Thor hardware encoder; use --thor")
    def test_thor_both_cameras(self):
        for camera in ("rgb", "mono"):
            with self.subTest(camera=camera):
                self.check_stream("thor", camera)

    def test_invalid_configuration_rejected_without_fallback(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            config = root / "system.ini"
            for backend, bitrate in (("typo", 3000), ("thor", 0), ("thor", 4294968)):
                config.write_text(f"[rgb]\nencoder_backend={backend}\nencoder_bitrate={bitrate}\n")
                run = self.run_encoder(config, "rgb", root / "invalid.h264")
                self.assertEqual(run.returncode, 1, run.stderr)
                self.assertFalse((root / "invalid.h264").exists())


if __name__ == "__main__":
    unittest.main()
