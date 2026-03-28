"""
preprocessing.py
----------------
Frame preprocessing pipeline for uParking drone scanner.

Adapted from:
    Peraza-Garzón et al. (2026) - "Intelligent Car Park Occupancy Monitoring
    System Based on Parking Slot and Vehicle Detection Using DJI Mini 3
    Aerial Imagery and YOLOv11"
    DOI: https://doi.org/10.3390/ai7020074

Pipeline:
    1. Laplacian variance blur detection  (discard blurry frames)
    2. Letterbox resize to 640x640        (preserve aspect ratio)
    3. RGB normalization to [0, 1]        (model input standard)
"""

import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple


# -- Tunable constants --------------------------------------------------------

# Minimum Laplacian variance to accept a frame as sharp enough.
# Frames below this are discarded before inference.
# Lower = more permissive, higher = stricter sharpness requirement.
LAPLACIAN_THRESHOLD: float = 100.0

# Target size expected by YOLOv11 (must match training resolution)
TARGET_SIZE: int = 640

# Padding colour used for letterboxing (neutral grey, standard for YOLO)
PAD_COLOR: Tuple[int, int, int] = (114, 114, 114)


# -- Data classes -------------------------------------------------------------

@dataclass
class PreprocessResult:
    """
    Container returned by preprocess_frame().

    Attributes
    ----------
    frame       : Preprocessed float32 numpy array, shape (640, 640, 3),
                  values in [0, 1].
    scale       : Scale factor applied when resizing the original frame.
                  Used to map bounding-box coordinates back to original space.
    pad_top     : Pixels of top padding added during letterboxing.
    pad_left    : Pixels of left padding added during letterboxing.
    laplacian   : Laplacian variance score of the raw frame.
                  Useful for logging / debugging frame quality.
    """
    frame: np.ndarray
    scale: float
    pad_top: int
    pad_left: int
    laplacian: float


# -- Core functions ------------------------------------------------------------

def compute_laplacian_variance(frame: np.ndarray) -> float:
    """
    Compute Laplacian variance as a sharpness metric.

    A low variance indicates a blurry image.  Paper 2 uses this to
    automatically discard frames extracted from drone video that did
    not meet quality standards.

    Parameters
    ----------
    frame : BGR numpy array (H, W, 3), uint8.

    Returns
    -------
    float : Variance of the Laplacian.  Higher = sharper.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def letterbox_resize(
    frame: np.ndarray,
    target: int = TARGET_SIZE,
    pad_color: Tuple[int, int, int] = PAD_COLOR
) -> Tuple[np.ndarray, float, int, int]:
    """
    Resize frame to (target x target) while preserving aspect ratio.

    Padding is added symmetrically on the shorter axis so the image is
    not distorted.  This is the standard letterbox approach used by
    Ultralytics YOLO models and followed by Peraza-Garzón et al.

    Parameters
    ----------
    frame      : BGR numpy array (H, W, 3).
    target     : Output square size in pixels.  Default 640.
    pad_color  : RGB tuple for the padding colour.  Default (114,114,114).

    Returns
    -------
    padded  : Resized + padded numpy array, shape (target, target, 3).
    scale   : Uniform scale factor applied to both dimensions.
    pad_top : Pixels of padding on the top edge.
    pad_left: Pixels of padding on the left edge.
    """
    h, w = frame.shape[:2]
    scale = target / max(h, w)

    new_h = int(round(h * scale))
    new_w = int(round(w * scale))

    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    pad_top  = (target - new_h) // 2
    pad_left = (target - new_w) // 2
    pad_bot  = target - new_h - pad_top
    pad_right = target - new_w - pad_left

    padded = cv2.copyMakeBorder(
        resized,
        pad_top, pad_bot, pad_left, pad_right,
        cv2.BORDER_CONSTANT,
        value=pad_color
    )

    return padded, scale, pad_top, pad_left


def preprocess_frame(
    frame: np.ndarray,
    blur_threshold: float = LAPLACIAN_THRESHOLD,
    target_size: int = TARGET_SIZE
) -> Optional[PreprocessResult]:
    """
    Full preprocessing pipeline for a single drone camera frame.

    Steps (Peraza-Garzón et al., 2026):
        1. Laplacian variance blur check  → reject if below threshold
        2. Letterbox resize to 640×640    → preserve aspect ratio
        3. BGR → RGB conversion           → YOLO expects RGB
        4. Normalize to float32 [0, 1]   → model input range

    Parameters
    ----------
    frame          : Raw BGR numpy array from QDrone2 camera (H, W, 3).
    blur_threshold : Minimum Laplacian variance to accept the frame.
    target_size    : Square output resolution (must match YOLO training).

    Returns
    -------
    PreprocessResult if the frame passes quality checks, else None.
    Callers should skip inference when None is returned.

    Example
    -------
    >>> result = preprocess_frame(raw_frame)
    >>> if result is None:
    ...     print("Frame too blurry, skipping")
    ... else:
    ...     vehicles, slots = detector.detect(result.frame)
    """
    if frame is None or frame.size == 0:
        return None

    # Step 1 — blur detection
    lap_var = compute_laplacian_variance(frame)
    if lap_var < blur_threshold:
        return None  # discard blurry frame

    # Step 2 — letterbox resize
    padded, scale, pad_top, pad_left = letterbox_resize(
        frame, target=target_size
    )

    # Step 3 — BGR → RGB (OpenCV loads BGR, YOLO expects RGB)
    rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)

    # Step 4 — normalize to [0, 1]
    normalized = rgb.astype(np.float32) / 255.0

    return PreprocessResult(
        frame=normalized,
        scale=scale,
        pad_top=pad_top,
        pad_left=pad_left,
        laplacian=lap_var
    )


def unscale_bbox(
    bbox: list,
    scale: float,
    pad_top: int,
    pad_left: int
) -> list:
    """
    Map a bounding box from the 640×640 preprocessed space back to the
    original frame resolution.

    Useful when you need to draw detections onto the original camera feed
    or compare slot positions with QLabs reference-frame coordinates.

    Parameters
    ----------
    bbox     : [x1, y1, x2, y2] in 640×640 space.
    scale    : Scale factor returned by preprocess_frame().
    pad_top  : Top padding returned by preprocess_frame().
    pad_left : Left padding returned by preprocess_frame().

    Returns
    -------
    [x1, y1, x2, y2] in original frame pixel coordinates.
    """
    x1, y1, x2, y2 = bbox
    x1 = (x1 - pad_left) / scale
    y1 = (y1 - pad_top)  / scale
    x2 = (x2 - pad_left) / scale
    y2 = (y2 - pad_top)  / scale
    return [x1, y1, x2, y2]
