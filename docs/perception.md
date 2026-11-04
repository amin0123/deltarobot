# RGB perception, coordinate frames and calibration

The perception node receives only `sensor_msgs/Image`. It converts RGB to HSV, thresholds the configured hue bands and saturation/value minima, applies a small morphological opening, and finds external contours. Area, circularity, solidity, aspect ratio, filled-circle fraction, image borders and confidence reject fragments and merged shapes. A clearance halo checks neighboring color pixels, including fragments that failed contour validation. Nearby accepted circles are also rejected.

Color is inferred from image pixels. Neither object names, assigned materials, semantic images, instance masks nor simulator cap poses are inputs. The simulator uses a single RGB annotator for this camera. Published confidence is a contour-quality/purity score, not a statistically calibrated probability.

## Frames

| Frame | Definition |
| --- | --- |
| `world` | Metres, Z up, floor at Z=0, X along conveyor travel |
| `base_link` | Shoulder-center plane; world translation `[0,0,1.17]`, same axis directions |
| `conveyor` | World translation `[0,0,0.80]`, top surface plane |
| `camera_optical` | World position `[0,0,1.10]`; optical X=world X, optical Y=-world Y, optical Z=-world Z |
| `tool0` | Cup contact point, measured from the dynamic platform and its current orientation |
| `upper_0..2` | Physical upper rigid-body frames at shoulder pivots |

The USD camera looks along -Z with +Y up. Its corresponding ROS optical transform is a 180-degree rotation about X, quaternion `[x,y,z,w]=[1,0,0,0]`. The projection node obtains this transform through TF at the **image timestamp**, rather than assuming a world pose independently.

## Pinhole calibration

The default camera is 640×480 pixels, with 12 mm focal length and 16 mm horizontal aperture. The vertical aperture is set to preserve square pixels. Thus:

```text
fx = fy = width * focal_length / horizontal_aperture = 480 pixels
cx = 320, cy = 240
K = [[480, 0, 320], [0, 480, 240], [0, 0, 1]]
```

The simulation camera has zero distortion; the projection node refuses nonzero `CameraInfo.d` because this baseline expects rectified images. An image point gives ray `R K⁻¹ [u,v,1]`. Starting at camera translation `t` in `base_link`, intersect this ray with the horizontal **cap-top plane**, not the belt surface:

```text
plane_z = conveyor.top + caps.height - robot.base_height = -0.358 m
lambda = (plane_z - t.z) / ray.z
target = t + lambda * ray
```

Rays parallel to the plane or pointing away are rejected. A second projected ray at the detected pixel radius checks that apparent cap radius is 10–19 mm. Centers outside the belt picking ROI (`x=±95 mm`, `y=±45 mm`) or outside the commissioned IK workspace are rejected. The approach target is above the same XY; the final commanded cup point is 1 mm above the estimated top plane to allow the ideal vacuum model to seal without pushing the cap into the belt.

The camera covers approximately 384×288 mm at cap-top height. The robot parks at `[0,-0.125,-0.300]` before using images, clearing the main picking region. Detection runs continuously, but the task accepts a picking target only from a fresh image captured after the belt has stopped and settled.

## Tuning and limits

Use saved `camera_*.png` images or `rqt_image_view` to tune HSV limits. Tune `min_area`/`max_area` when changing camera optics. Red uses two bands to cover hue wraparound. Add new colors consistently to `caps.colors`, `perception.hue_ranges`, and `bins`; the simulator supplies caps in a repeatable color cycle with randomized XY lane offset, yaw and small initial tilt.

This is a calibrated, flat-belt, isolated-cap baseline. A single RGB camera and assumed top plane cannot reliably distinguish fully hidden stacked caps, cap orientation with identical top/bottom appearance, or arbitrary heights. The task conservatively skips visible overlap and poor contours; it does not claim to solve every possible occlusion. Caps settle upright in the supplied test population. Depth, multi-view estimation, measured real-camera extrinsics, suction pressure sensing, and continuous moving-belt interception are future extensions.
