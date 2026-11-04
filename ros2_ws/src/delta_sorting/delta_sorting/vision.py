"""RGB-only cap segmentation. No simulator metadata is accepted by this module."""
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass
class Detection:
    u: float
    v: float
    radius: float
    color: str
    confidence: float


def detect_caps(rgb, cfg):
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError("expected uint8 RGB image")
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    masks, detections, rejected = {}, [], 0
    kernel = np.ones((3, 3), np.uint8)
    for color, ranges in cfg["hue_ranges"].items():
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for low, high in ranges:
            mask |= cv2.inRange(hsv, (low, cfg["min_saturation"], cfg["min_value"]), (high, 255, 255))
        masks[color] = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    for color, mask in masks.items():
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < cfg["min_area"]:
                continue
            perimeter = cv2.arcLength(contour, True)
            circularity = 4*np.pi*area / max(perimeter*perimeter, 1)
            solidity = area / max(cv2.contourArea(cv2.convexHull(contour)), 1)
            (u, v), radius = cv2.minEnclosingCircle(contour)
            fill = area / max(np.pi*radius*radius, 1)
            x, y, w, h = cv2.boundingRect(contour)
            # Reject border fragments, merged caps, narrow crescents and partial occlusions.
            if (area > cfg["max_area"] or circularity < cfg["min_circularity"] or
                solidity < cfg["min_solidity"] or fill < 0.78 or
                min(w, h) / max(w, h) < 0.82 or x < 2 or y < 2 or
                x+w >= rgb.shape[1]-2 or y+h >= rgb.shape[0]-2):
                rejected += 1
                continue
            interior = np.zeros(mask.shape, np.uint8)
            cv2.drawContours(interior, [contour], -1, 255, -1)
            purity = np.count_nonzero(mask & interior) / max(np.count_nonzero(interior), 1)
            confidence = float(min(1.0, circularity/0.9) * min(1.0, fill/0.9) * purity)
            # A visible round cap can sit on a rejected crescent of another cap.
            # Include ALL other-color pixels, not just accepted detections, in
            # the local clearance check so the top cap is rejected as well.
            halo = np.zeros(mask.shape, np.uint8)
            cv2.circle(halo, (round(u), round(v)), int(np.ceil(radius*1.18)), 255, -1)
            other = np.zeros(mask.shape, np.uint8)
            for other_color, other_mask in masks.items():
                if other_color != color:
                    other |= other_mask
            if np.count_nonzero(other & halo) > max(20, area*0.04):
                rejected += 1
                continue
            if confidence < cfg["min_confidence"]:
                rejected += 1
                continue
            m = cv2.moments(contour)
            detections.append(Detection(m["m10"]/m["m00"], m["m01"]/m["m00"], radius, color, confidence))
    # Even individually circular blobs near another coloured blob are unsafe for this cup.
    unsafe = set()
    for i, a in enumerate(detections):
        for j, b in enumerate(detections[i+1:], i+1):
            if np.hypot(a.u-b.u, a.v-b.v) < (a.radius+b.radius)*1.16:
                unsafe.update([i, j])
    return [d for i, d in enumerate(detections) if i not in unsafe], rejected + len(unsafe)


def pixel_to_plane(u, v, intrinsic, translation, rotation, plane_z):
    """Intersect a calibrated optical ray with a plane in the target frame."""
    k = np.asarray(intrinsic, dtype=float).reshape(3, 3)
    t = np.asarray(translation, dtype=float)
    ray = np.asarray(rotation) @ np.linalg.solve(k, [u, v, 1.0])
    if not np.isfinite(ray).all() or abs(ray[2]) < 1e-8:
        raise ValueError("camera ray is parallel to the conveyor")
    distance = (plane_z-t[2]) / ray[2]
    if distance <= 0:
        raise ValueError("conveyor plane is behind the camera")
    return t + distance*ray
