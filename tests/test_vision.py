import cv2
import numpy as np
import pytest
from delta_sorting.vision import detect_caps, pixel_to_plane


def canvas():
    return np.full((480, 640, 3), 30, np.uint8)


@pytest.mark.parametrize("color,rgb", [("red", (240, 20, 20)), ("blue", (20, 40, 240)),
                                     ("green", (20, 220, 30)), ("yellow", (240, 220, 15))])
def test_isolated_caps_multiple_positions(color, rgb, cfg):
    for x, y in [(80, 100), (320, 240), (520, 380)]:
        im = canvas()
        cv2.circle(im, (x, y), 23, rgb, -1)
        detections, _ = detect_caps(im, cfg["perception"])
        assert len(detections) == 1
        d = detections[0]
        assert d.color == color and d.confidence >= .8
        assert abs(d.u-x) < .5 and abs(d.v-y) < .5


@pytest.mark.parametrize("kind", ["empty", "gray", "purple", "same_overlap", "different_overlap", "ambiguous", "border"])
def test_uncertain_overlapping_and_empty_frames_do_not_pick(kind, cfg):
    im = canvas()
    if kind == "gray":
        cv2.circle(im, (250, 240), 23, (140, 140, 140), -1)
    elif kind == "purple":
        cv2.circle(im, (250, 240), 23, (220, 10, 220), -1)
    elif kind in ("same_overlap", "different_overlap"):
        cv2.circle(im, (250, 240), 23, (240, 20, 20), -1)
        cv2.circle(im, (282, 240), 23, (240, 20, 20) if kind == "same_overlap" else (20, 40, 240), -1)
    elif kind == "ambiguous":
        cv2.ellipse(im, (250, 240), (23, 23), 0, 0, 180, (240, 20, 20), -1)
        cv2.ellipse(im, (250, 240), (23, 23), 0, 180, 360, (20, 40, 240), -1)
    elif kind == "border":
        cv2.circle(im, (3, 240), 23, (240, 20, 20), -1)
    detections, _ = detect_caps(im, cfg["perception"])
    assert detections == []


def test_calibrated_plane_projection(cfg):
    c = cfg["camera"]
    f = c["width"]*c["focal_length_mm"]/c["horizontal_aperture_mm"]
    k = np.array([[f, 0, 320], [0, f, 240], [0, 0, 1]])
    r = np.diag([1, -1, -1])
    t = np.array(c["position"])-[0, 0, cfg["robot"]["base_height"]]
    z = cfg["conveyor"]["top"]+cfg["caps"]["height"]-cfg["robot"]["base_height"]
    for p in [[0, 0, z], [-.07, .02, z], [.08, -.025, z]]:
        optical = r.T @ (np.array(p)-t)
        uvw = k @ optical
        result = pixel_to_plane(uvw[0]/uvw[2], uvw[1]/uvw[2], k, t, r, z)
        np.testing.assert_allclose(result, p, atol=1e-12)


def test_projection_rejects_camera_facing_away():
    with pytest.raises(ValueError, match="behind"):
        pixel_to_plane(0, 0, np.eye(3), [0, 0, 1], np.eye(3), 0)
