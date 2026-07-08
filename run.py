#!/usr/bin/env python3
"""
Patronus — inference pipeline launcher.

Usage:
  ./run.py                         Build + run both pipelines
  ./run.py --rgb-only              Run RGB pipeline only
  ./run.py --mono-only             Run mono pipeline only
  ./run.py --validate              Check prerequisites, don't run
  ./run.py --list-cameras          Enumerate connected Basler cameras
  ./run.py --config <file>         Use custom config file
  ./run.py --dry-run               Show commands without executing
  ./run.py --no-build              Skip build step
  ./run.py --gst-debug=<level>     Pass GStreamer debug level (1-6)
  ./run.py --help                  Show this help and exit
"""

import argparse
import os
import subprocess
import sys
import signal
import shutil
import textwrap

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
BUILD_DIR = os.path.join(PROJECT_ROOT, "build")
INFERENCE_BIN = os.path.join(BUILD_DIR, "inference")
DEFAULT_CONFIG = os.path.join("config", "system.ini").replace(
    os.path.commonpath([PROJECT_ROOT, PROJECT_ROOT]), "", 1
).lstrip("/") or "config/system.ini"


def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)


def run_command(cmd, cwd=None, dry_run=False):
    print(f"  $ {' '.join(cmd)}")
    if dry_run:
        return 0
    result = subprocess.run(cmd, cwd=cwd or PROJECT_ROOT)
    return result.returncode


# ---------------------------------------------------------------------------
# Validation helpers
# ---------------------------------------------------------------------------

def check_model_files():
    """Ensure ONNX + TensorRT engine files exist."""
    models = [
        ("models/yolov8_p2.onnx", "RGB ONNX model"),
        ("models/yolov8_p2.onnx_b1_gpu0_fp16.engine", "RGB TensorRT engine"),
        ("models/yolov8_mono.onnx", "Mono ONNX model"),
        ("models/yolov8_mono.onnx_b1_gpu0_fp16.engine", "Mono TensorRT engine"),
    ]
    missing = []
    for rel_path, desc in models:
        full = os.path.join(PROJECT_ROOT, rel_path)
        if not os.path.isfile(full):
            missing.append(f"  {desc}: {rel_path}")
    return missing


def check_custom_lib():
    """Ensure the custom YOLOv8 bbox parser .so exists."""
    path = os.path.join(PROJECT_ROOT, "libs/deepstream/libnvdsinfer_custom_impl_Yolo.so")
    if not os.path.isfile(path):
        return [f"  Custom YOLO bbox parser: libs/deepstream/libnvdsinfer_custom_impl_Yolo.so"]
    return []


def check_deepstream():
    """Check if DeepStream libraries are reachable."""
    try:
        result = subprocess.run(
            ["ldconfig", "-p"],
            capture_output=True, text=True, timeout=5
        )
        libs_found = result.stdout
        needed = ["libnvdsgst_meta.so", "libnvds_meta.so", "libnvds_yml_parser.so"]
        missing = []
        for lib in needed:
            if lib not in libs_found:
                missing.append(f"  {lib}")
        return missing
    except Exception:
        return ["  (could not run ldconfig; check DeepStream installation)"]


def check_cuda():
    """Check CUDA availability."""
    try:
        result = subprocess.run(
            ["nvidia-smi"],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode != 0:
            return ["nvidia-smi returned non-zero"]
        return []
    except FileNotFoundError:
        return ["nvidia-smi not found (check CUDA installation)"]


def check_cameras():
    """Check if Basler cameras are visible via pylon tools."""
    # Try pylon-config first, then pylonCLI
    for tool, args, label in [
        ("pylon-config", ["--list-devices"], "pylon-config"),
        ("pylonCLI", ["camera", "list"], "pylonCLI"),
    ]:
        tool_path = shutil.which(tool)
        if tool_path:
            try:
                result = subprocess.run(
                    [tool_path] + args,
                    capture_output=True, text=True, timeout=5
                )
                if result.returncode == 0 and result.stdout.strip():
                    return [f"  Found via {tool}:", f"    {result.stdout.strip()}"]
                return [f"  {tool} found but no cameras detected"]
            except Exception:
                pass
    # Fallback: try gst-inspect for pylonsrc
    try:
        subprocess.run(
            ["gst-inspect-1.0", "pylonsrc"],
            capture_output=True, timeout=5
        )
        return ["  pylonsrc GStreamer plugin available (use --validate for details)"]
    except FileNotFoundError:
        return ["  gst-inspect-1.0 not found"]
    return ["  No camera detection tool found (install pylon SDK)"]


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

def cmd_validate(args):
    """Check prerequisites without running."""
    print("=== Patronus Environment Validation ===\n")

    checks = [
        ("CUDA", check_cuda),
        ("DeepStream libraries", check_deepstream),
        ("Model files", check_model_files),
        ("Custom YOLO bbox parser", check_custom_lib),
        ("Basler cameras", check_cameras),
    ]

    all_ok = True
    for name, check_fn in checks:
        print(f"[{name}]")
        issues = check_fn()
        if issues:
            all_ok = False
            for line in issues:
                print(f"  ✗ {line}")
        else:
            print(f"  ✓ OK")
        print()

    if os.path.isfile(INFERENCE_BIN):
        print(f"[Build]")
        print(f"  ✓ Binary at: {INFERENCE_BIN}")
    else:
        print(f"[Build]")
        print(f"  ~ Not yet built (run without --validate to build)")
    print()

    # Check config
    config_file = args.config or os.path.join(PROJECT_ROOT, DEFAULT_CONFIG)
    if os.path.isfile(config_file):
        print(f"[Config]")
        print(f"  ✓ {config_file}")
    else:
        print(f"[Config]")
        print(f"  ✗ Config not found: {config_file}")
        all_ok = False
    print()

    if all_ok:
        print("All checks passed. Ready to run.")
    else:
        print("Some checks failed — review issues above.")
    return 0 if all_ok else 1


def cmd_list_cameras(args):
    """Enumerate connected Basler cameras."""
    print("=== Basler Cameras ===\n")

    for tool, tool_args in [
        ("pylon-config", ["--list-devices"]),
        ("pylonCLI", ["camera", "list"]),
    ]:
        tool_path = shutil.which(tool)
        if tool_path:
            try:
                result = subprocess.run(
                    [tool_path] + tool_args,
                    capture_output=True, text=True, timeout=5
                )
                if result.returncode == 0 and result.stdout.strip():
                    print(result.stdout.strip())
                    return 0
                print(f"(no cameras detected by {tool})")
            except subprocess.TimeoutExpired:
                print(f"({tool} timed out)")
            except Exception as e:
                print(f"({tool} error: {e})")

    print("No pylon tools found. Install Basler pylon SDK or connect cameras.")
    return 1


def ensure_built(dry_run, args):
    """Build the project if the binary doesn't exist or if forced."""
    if args.no_build:
        if not os.path.isfile(INFERENCE_BIN):
            eprint("error: --no-build specified but binary not found")
            sys.exit(1)
        return

    config_marker = os.path.join(BUILD_DIR, "CMakeCache.txt")
    build_file = os.path.join(BUILD_DIR, "Makefile")
    if not os.path.isfile(build_file):
        build_file = os.path.join(BUILD_DIR, "build.ninja")
    needs_configure = not os.path.isdir(BUILD_DIR) or not os.path.isfile(config_marker) or not os.path.isfile(build_file)

    if needs_configure:
        print("Configuring CMake...")
        cmd = ["cmake", "-B", BUILD_DIR, "-DCMAKE_BUILD_TYPE=Release",
               "-DCMAKE_CUDA_ARCHITECTURES=native"]
        ret = run_command(cmd, dry_run=dry_run)
        if ret != 0:
            eprint("error: cmake configuration failed")
            sys.exit(1)

    if not os.path.isfile(INFERENCE_BIN):
        print("Building...")
        cmd = ["cmake", "--build", BUILD_DIR, "-j", str(os.cpu_count() or 4)]
        ret = run_command(cmd, dry_run=dry_run)
        if ret != 0:
            eprint("error: build failed")
            sys.exit(1)
        print("Build complete.\n")


def cmd_run(args, remaining_argv):
    """Build and run the inference pipeline(s)."""
    config_file = args.config or DEFAULT_CONFIG
    config_abs = config_file if os.path.isabs(config_file) else \
        os.path.join(PROJECT_ROOT, config_file)

    if not os.path.isfile(config_abs):
        eprint(f"error: config not found: {config_abs}")
        sys.exit(1)

    ensure_built(args.dry_run, args)

    # Build the inference command
    cmd = [INFERENCE_BIN]
    cmd.extend(["--config", config_abs])
    if args.rgb_only:
        cmd.append("--rgb-only")
    if args.mono_only:
        cmd.append("--mono-only")
    if remaining_argv:
        cmd.extend(remaining_argv)

    print(f"Launching: {' '.join(cmd)}\n")

    if args.dry_run:
        return 0

    # Launch and handle Ctrl+C gracefully
    process = subprocess.Popen(cmd, cwd=PROJECT_ROOT)

    def signal_handler(sig, frame):
        print("\nShutting down pipelines...")
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    ret = process.wait()
    return ret


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        prog="run.py",
        description="Patronus inference pipeline launcher",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent("""\
            Examples:
              ./run.py                          Build + run both pipelines
              ./run.py --rgb-only               Just the RGB pipeline
              ./run.py --validate               Check environment
              ./run.py --list-cameras           Show connected Basler cameras
              ./run.py -- --gst-debug=3         Pass flags to inference binary
        """),
        allow_abbrev=True,
    )
    parser.add_argument("--config", default=None,
                        help=f"Config file path (default: {DEFAULT_CONFIG})")
    parser.add_argument("--rgb-only", action="store_true",
                        help="Run RGB pipeline only")
    parser.add_argument("--mono-only", action="store_true",
                        help="Run mono pipeline only")
    parser.add_argument("--validate", action="store_true",
                        help="Check prerequisites and exit")
    parser.add_argument("--list-cameras", action="store_true",
                        help="Enumerate connected Basler cameras")
    parser.add_argument("--dry-run", action="store_true",
                        help="Show commands without executing")
    parser.add_argument("--no-build", action="store_true",
                        help="Skip build step")

    args, remaining_argv = parser.parse_known_args()

    os.chdir(PROJECT_ROOT)

    if args.validate:
        sys.exit(cmd_validate(args))
    if args.list_cameras:
        sys.exit(cmd_list_cameras(args))

    sys.exit(cmd_run(args, remaining_argv))


if __name__ == "__main__":
    main()
