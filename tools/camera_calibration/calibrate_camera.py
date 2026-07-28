"""Calculate camera intrinsics from captured chessboard images."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calibrate a camera from chessboard PNG files."
    )
    parser.add_argument(
        "--square-mm",
        type=float,
        required=True,
        help="Measured physical side length of one displayed square in millimetres",
    )
    parser.add_argument("--cols", type=int, default=9, help="Inner-corner columns")
    parser.add_argument("--rows", type=int, default=6, help="Inner-corner rows")
    parser.add_argument(
        "--images",
        type=Path,
        default=Path(__file__).resolve().parent / "images",
        help="Directory containing calibration images",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parent
        / "output"
        / "camera_calibration.json",
        help="Output JSON path",
    )
    return parser.parse_args()


def detect_corners(
    gray: np.ndarray, pattern_size: tuple[int, int]
) -> np.ndarray | None:
    found, corners = cv2.findChessboardCorners(
        gray,
        pattern_size,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    if not found:
        return None
    return cv2.cornerSubPix(
        gray,
        corners,
        (11, 11),
        (-1, -1),
        (
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        ),
    )


def main() -> int:
    args = parse_args()
    if args.square_mm <= 0:
        print("ERROR: --square-mm must be greater than zero.")
        return 1

    image_paths = sorted(args.images.glob("calibration_*.png"))
    if len(image_paths) < 10:
        print(
            f"ERROR: Only {len(image_paths)} image(s) found. "
            "Capture at least 15, preferably 20-30."
        )
        return 2

    pattern_size = (args.cols, args.rows)
    square_m = args.square_mm / 1000.0
    object_template = np.zeros((args.rows * args.cols, 3), np.float32)
    object_template[:, :2] = (
        np.mgrid[0 : args.cols, 0 : args.rows].T.reshape(-1, 2) * square_m
    )

    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    used_paths: list[Path] = []
    image_size: tuple[int, int] | None = None

    for path in image_paths:
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            print(f"Skipped unreadable image: {path.name}")
            continue

        current_size = (gray.shape[1], gray.shape[0])
        if image_size is None:
            image_size = current_size
        elif current_size != image_size:
            print(
                f"Skipped different resolution: {path.name} "
                f"({current_size[0]}x{current_size[1]})"
            )
            continue

        corners = detect_corners(gray, pattern_size)
        if corners is None:
            print(f"Skipped, chessboard not found: {path.name}")
            continue

        object_points.append(object_template.copy())
        image_points.append(corners)
        used_paths.append(path)

    if image_size is None or len(used_paths) < 10:
        print(
            f"ERROR: Only {len(used_paths)} valid image(s). "
            "Capture more views with the complete chessboard visible."
        )
        return 3

    rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )

    per_image_errors = []
    for path, object_pts, detected, rvec, tvec in zip(
        used_paths, object_points, image_points, rvecs, tvecs
    ):
        projected, _ = cv2.projectPoints(
            object_pts, rvec, tvec, camera_matrix, distortion
        )
        residual = detected.reshape(-1, 2) - projected.reshape(-1, 2)
        error = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
        per_image_errors.append({"image": path.name, "rms_px": error})

    result = {
        "image_width": image_size[0],
        "image_height": image_size[1],
        "pattern_inner_corners": {"columns": args.cols, "rows": args.rows},
        "square_size_m": square_m,
        "rms_reprojection_error_px": float(rms),
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": distortion.reshape(-1).tolist(),
        "valid_image_count": len(used_paths),
        "per_image_errors": per_image_errors,
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    print(f"Valid images: {len(used_paths)}")
    print(f"Resolution: {image_size[0]}x{image_size[1]}")
    print(f"RMS reprojection error: {rms:.4f} px")
    print("Camera matrix:")
    print(camera_matrix)
    print("Distortion coefficients:")
    print(distortion.reshape(-1))
    print(f"Saved: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
