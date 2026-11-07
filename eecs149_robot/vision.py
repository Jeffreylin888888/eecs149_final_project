"""Optional external-camera ArUco marker detector. Never imports OpenCV in simulation."""
from __future__ import annotations
from dataclasses import dataclass


@dataclass(frozen=True)
class Marker:
    id: int
    center_px: tuple[float, float]


def detect_aruco(frame, dictionary_name: str = "DICT_4X4_50") -> list[Marker]:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("Install optional vision dependencies: pip install -e '.[vision]'") from exc
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, dictionary_name))
    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(frame)
    else:
        corners, ids, _ = aruco.detectMarkers(frame, dictionary)
    if ids is None:
        return []
    output: list[Marker] = []
    for corners_one, marker_id in zip(corners, ids.flatten()):
        p = corners_one.reshape(-1, 2)
        output.append(Marker(int(marker_id), (float(p[:, 0].mean()), float(p[:, 1].mean()))))
    return output
