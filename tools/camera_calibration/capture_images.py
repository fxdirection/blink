"""Capture chessboard images from a USB camera for intrinsic calibration."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import cv2


WINDOW_NAME = "Camera calibration capture"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture 9x6-inner-corner chessboard images."
    )
    parser.add_argument("--camera", type=int, default=1, help="OpenCV camera index")
    parser.add_argument("--width", type=int, default=640, help="Requested width")
    parser.add_argument("--height", type=int, default=480, help="Requested height")
    parser.add_argument("--cols", type=int, default=9, help="Inner-corner columns")
    parser.add_argument("--rows", type=int, default=6, help="Inner-corner rows")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parent / "images",
        help="Directory used to save captured PNG files",
    )
    return parser.parse_args()


def open_camera(index: int, width: int, height: int) -> cv2.VideoCapture:
    backends = (
        (cv2.CAP_MSMF, cv2.CAP_DSHOW, cv2.CAP_ANY)
        if os.name == "nt"
        else (cv2.CAP_ANY,)
    )
    camera = None
    for backend in backends:
        candidate = cv2.VideoCapture(index, backend)
        if candidate.isOpened():
            camera = candidate
            break
        candidate.release()

    if camera is None:
        raise RuntimeError(
            f"Cannot open camera index {index}. Try --camera 0 or --camera 2."
        )

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return camera


def next_image_number(output_dir: Path) -> int:
    numbers = []
    for path in output_dir.glob("calibration_*.png"):
        try:
            numbers.append(int(path.stem.rsplit("_", 1)[1]))
        except (IndexError, ValueError):
            continue
    return max(numbers, default=0) + 1


def main() -> int:
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    pattern_size = (args.cols, args.rows)

    try:
        camera = open_camera(args.camera, args.width, args.height)
    except RuntimeError as exc:
        print(f"ERROR: {exc}")
        return 1

    actual_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
    image_number = next_image_number(args.output)
    saved_count = len(list(args.output.glob("calibration_*.png")))

    print(f"Camera index: {args.camera}")
    print(f"Actual resolution: {actual_width}x{actual_height}")
    print(f"Chessboard inner corners: {args.cols}x{args.rows}")
    print("SPACE: save a detected chessboard image")
    print("Q or ESC: quit")

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                print("ERROR: Failed to read a camera frame.")
                return 2

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(
                gray,
                pattern_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH
                | cv2.CALIB_CB_NORMALIZE_IMAGE
                | cv2.CALIB_CB_FAST_CHECK,
            )

            if found:
                corners = cv2.cornerSubPix(
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
                cv2.drawChessboardCorners(frame, pattern_size, corners, found)

            state = "FOUND - press SPACE" if found else "NOT FOUND"
            color = (0, 200, 0) if found else (0, 0, 255)
            cv2.putText(
                frame,
                state,
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                color,
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                f"Saved: {saved_count} | Resolution: {actual_width}x{actual_height}",
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(WINDOW_NAME, frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord(" ") and found:
                output_path = args.output / f"calibration_{image_number:03d}.png"
                if not cv2.imwrite(str(output_path), gray):
                    print(f"ERROR: Could not save {output_path}")
                    return 3
                print(f"Saved: {output_path}")
                image_number += 1
                saved_count += 1
    finally:
        camera.release()
        cv2.destroyAllWindows()

    print(f"Finished. Total saved images: {saved_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
