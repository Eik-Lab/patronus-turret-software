#!/usr/bin/env python3
"""Fit focal lengths from a focal_calib run.

For every consecutive pair of frames, phase correlation measures how far the
image content moved; a least-squares fit against the encoder change gives the
2x2 map from (pan, tilt) in radians to (u, v) image motion in pixels. Its
diagonal, scaled into detection pixels, is focal_x_px / focal_y_px, and its sign
is the sign the filter assumes (content moves opposite to the camera).

Usage: python3 tools/focal_calib_analyse.py <out_dir>
Needs numpy.
"""

import csv
import sys

import numpy as np

# Downsampling before correlation: averages away sensor noise and fixed-pattern
# noise, and keeps the FFTs small. Sub-pixel refinement recovers the precision.
DOWNSAMPLE = 4
# Correlation peaks weaker than this many standard deviations are not trusted.
MIN_PEAK = 8.0


def read_pgm(path):
    with open(path, "rb") as f:
        if f.readline().strip() != b"P5":
            raise ValueError(f"{path}: not a binary PGM")
        w, h = map(int, f.readline().split())
        f.readline()
        return np.frombuffer(f.read(), np.uint8).reshape(h, w).astype(np.float32)


def downsample(a, k):
    h, w = a.shape
    return a[: h // k * k, : w // k * k].reshape(h // k, k, w // k, k).mean((1, 3))


def content_motion(before, after):
    """Image-content motion from `before` to `after`, pixels, plus peak strength."""
    win = np.outer(np.hanning(before.shape[0]), np.hanning(before.shape[1]))
    fa = np.fft.fft2((after - after.mean()) * win)
    fb = np.fft.fft2((before - before.mean()) * win)
    cross = fa * np.conj(fb)
    r = np.fft.ifft2(cross / (np.abs(cross) + 1e-9)).real
    y, x = np.unravel_index(np.argmax(r), r.shape)
    peak = r[y, x] / r.std()

    def parabola(m, c, p):
        den = m - 2 * c + p
        return 0.0 if den == 0 else 0.5 * (m - p) / den

    ny, nx = r.shape
    dy = parabola(r[(y - 1) % ny, x], r[y, x], r[(y + 1) % ny, x])
    dx = parabola(r[y, (x - 1) % nx], r[y, x], r[y, (x + 1) % nx])
    if y > ny // 2:
        y -= ny
    if x > nx // 2:
        x -= nx
    return x + dx, y + dy, peak


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    out = sys.argv[1]
    poses = list(csv.DictReader(open(f"{out}/poses.csv")))
    pose = np.array([[float(p["pan_rad"]), float(p["tilt_rad"])] for p in poses])
    cameras = list(csv.DictReader(open(f"{out}/cameras.csv")))

    results = {}
    for cam in cameras:
        name = cam["camera"]
        cw, ch = int(cam["capture_w"]), int(cam["capture_h"])
        sx = int(cam["detection_w"]) / cw
        sy = int(cam["detection_h"]) / ch
        imgs = [downsample(read_pgm(f"{out}/{name}_{i}.pgm"), DOWNSAMPLE) for i in range(len(pose))]

        d_pose, motion = [], []
        for i in range(1, len(pose)):
            mx, my, peak = content_motion(imgs[i - 1], imgs[i])
            if peak < MIN_PEAK:
                print(f"  {name}: frames {i - 1}->{i}: weak correlation ({peak:.1f}), skipped")
                continue
            d_pose.append(pose[i] - pose[i - 1])
            motion.append([mx * DOWNSAMPLE, my * DOWNSAMPLE])
        if len(d_pose) < 4:
            print(f"{name}: too few usable frame pairs ({len(d_pose)}); is the scene lit and textured?")
            continue
        d_pose, motion = np.array(d_pose), np.array(motion)
        # motion = d_pose @ m: row 0 is per rad of pan, row 1 per rad of tilt.
        m, *_ = np.linalg.lstsq(d_pose, motion, rcond=None)
        rms = np.sqrt(((motion - d_pose @ m) ** 2).mean(0))
        fx, fy = m[0, 0] * sx, m[1, 1] * sy

        # A defocused or featureless view correlates on sensor noise and fixed
        # pattern instead of the scene: the fit then reads (near) zero motion,
        # or scatters. Either way the numbers mean nothing.
        moved = np.abs(motion).max(0)
        unreliable = (np.abs(m[0, 0]) * np.abs(d_pose[:, 0]).max() < 20
                      or np.abs(m[1, 1]) * np.abs(d_pose[:, 1]).max() < 20
                      or (rms > 0.05 * np.maximum(moved, 1.0)).any())
        if not unreliable:
            results[name] = (fx, fy)

        print(f"{name}: {len(d_pose)} frame pairs, fit rms {rms[0]:.1f} / {rms[1]:.1f} capture px")
        print(f"  capture px/rad: pan->u {m[0, 0]:+.0f}  tilt->v {m[1, 1]:+.0f}"
              f"   cross: pan->v {m[0, 1]:+.0f}  tilt->u {m[1, 0]:+.0f}")
        print(f"  detection px/rad: pan->u {fx:+.1f}  tilt->v {fy:+.1f}"
              f"   (configured: -{float(cam['focal_x_px']):.1f} / -{float(cam['focal_y_px']):.1f})")
        if unreliable:
            print("  UNRELIABLE: the image did not track the gimbal (fit residuals too large or"
                  " no motion found). The scene is probably out of focus, too dark or too"
                  " plain for this camera; not reporting a value.")
            continue
        if fx > 0 or fy > 0:
            print("  WARNING: content moves WITH the camera on an axis. The filter assumes it moves"
                  " against it; check the motor direction and camera mounting.")
        if abs(m[0, 1]) > 0.05 * abs(m[0, 0]) or abs(m[1, 0]) > 0.05 * abs(m[1, 1]):
            print("  WARNING: >5% cross-coupling; the camera is rolled relative to the gimbal axes.")

    if results:
        print("\nIf the scene was far away (> 30 m), put these in config/system.ini:")
        for name, (fx, fy) in results.items():
            print(f"  [{name}] focal_x_px={abs(fx):.1f}  focal_y_px={abs(fy):.1f}")


if __name__ == "__main__":
    main()
