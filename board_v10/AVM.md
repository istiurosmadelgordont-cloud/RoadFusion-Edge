# 2D bird's-eye view

The upper-right panel shows the calibrated ground projection. It renders a
realistic transparent top-view vehicle sprite over the camera imagery and
marks areas without observed ground as blind zones. Press `V` (or the AVM
button) for the enlarged view. The original four video panels remain
independent. The BEV worker refreshes at most twice per second and does not
block replay presentation.

## NVIDIA real-video group 11

`11_side_vehicle_bsd` has the original front, rear, left and right video plus
downloaded rear-left, rear-right and front-tele views from the **same clip**. The latter
are aligned by source timestamps and copied into its board replay directory
as `rear_left.mp4`, `rear_right.mp4` and `front_tele.mp4`. The scene's `avm.yml` comes from
NVIDIA's camera intrinsics, camera-to-ego extrinsics, ego masks and vehicle
dimensions for clip `66b5fa4b-de68-4339-9d18-669da3c6ce80`. The board AVM
uses all seven cameras when the three extra files exist. Only four are shown as
normal video tiles.

Generate the calibrated map on the PC with the Python environment containing
OpenCV, pandas, pyarrow and scipy:

```text
python tools/build_nvidia_avm_calibration.py GROUP_DIR SCENE_DIR/avm.yml
```

`GROUP_DIR` must contain `additional_views/manifest.json` and the downloaded
`additional_views/calibration` parquet files. Maps are specific to that clip
and rig. The live four-camera input needs its own measured calibration;
playing a different scene with this map gives an incorrect picture.

The projection assumes a flat ground plane. Cars, curbs and buildings above
the plane stretch at the seams. The narrow rear telecamera leaves limited
ground coverage behind the vehicle, and unobserved areas stay hatched. The
display is an approximate 2D bird's-eye view, not a full 360-degree surround
measurement or a rotatable 3D view.

For other rigs, measured ground correspondences can be converted with
`tests/make_avm_calibration.py`. The earlier experimental
`config/avm/66b5fa4b_approximate.yml` uses assumed camera geometry and is
retained only for comparison; use the real clip calibration above for the
demonstration.

## Replay timing and validation

Replay follows the source clock. If the board cannot display every source
frame, it skips late frames consistently across the four normal streams.
`fps_1s` is the actual display FPS, `playback_dropped` counts skipped source
frames, and recognition FPS is reported separately. This keeps a 20-second
clip near its recorded duration even when rendering is slower than 30 FPS.

`avm_regression` covers projection, overlap, missing coverage, resize and
reset. A board screenshot should additionally be inspected to verify the
central vehicle, seam positions and orientation for the chosen footage.
