# 2D bird's-eye view

The upper-right panel shows the calibrated ground projection. It renders a
realistic transparent top-view vehicle sprite over the camera imagery and
marks areas without observed ground as blind zones. Press `V` (or the AVM
button) for the enlarged view. The original four video panels remain
independent. The BEV worker refreshes at most twice per second and does not
block replay presentation.

Replay decode operations and decoder helper threads use CPU 0/1. The caller's
CPU affinity is restored after each decode operation, so UI and inference
remain on CPU 2/3. PCIe capture and live affinity assignments are unchanged.
For a recorded four-quadrant composite, YOLO consumes the existing composite
directly and scales returned coordinates to the canonical layout. This avoids
an unnecessary split/enlarge/recombine before detector preprocessing.

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

The GLES presenter uploads only UI pixels visible outside the active camera
tiles. Camera textures still update every frame; when the enlarged AVM is
shown, the complete UI texture is uploaded. Strided uploads use
`GL_EXT_unpack_subimage` when available and reusable packed buffers otherwise.
`ui_upload_regression` checks all camera presence combinations, clipping and
overlap so visible UI pixels are covered exactly once.

On 2026-09-30, two before/after replays of `66b5fa4b_30fps` with front-only
1600x320 UFLDv2 produced 23/22 periodic log samples. Combined median front
result age was 472.7/381.2 ms, YOLO result age 278.7/321.0 ms, lane task time
209.3/205.8 ms and display rate 17.7/16.7 FPS. Both recognition rates remained
2.5 FPS. These are sparse samples from this clip, not a guarantee across rigs
or footage. The user accepted a lower display rate to prioritize lane latency.

## PC visual review

`tools/pc_avm_preview.py` replays the synchronized seven-camera sample with
the original projected vehicles, a 1.55x central vehicle display model, and
bounded presentation texture extension. Calibration masks are unchanged;
extended texture is not additional observed ground. `B` compares against
strict coverage and `V` enlarges the camera BEV.

The separate `--driving-view` inset renders a violet perspective road, UFLDv2
lane curves and schematic vehicle symbols from YOLO detections. It does not
replace vehicles in the camera BEV. `D` toggles this inset. The PC sample uses
synchronized cached perception at 2.5 Hz, not live RKNN inference or measured
depth. Generate its cache with `tools/pc_driving_scene.py`; its YOLO input is
the matching sample's cached detections. Scene identity, frame count and FPS
are checked before replay. The Windows entry point is
`scripts/run_pc_avm_preview.cmd`. This inset has not yet been integrated into
the RK3568 real-time dashboard. It is currently disabled in the Windows
launcher at the user's request because the PC rendering felt slow. The
default launcher only runs the previous camera AVM preview; the optional
perception renderer and its cache are not loaded.

## Seven-camera PC coverage review (2026-10-11)

`tools/pc_surround_review.py` reads original 1080p NVIDIA camera frames and
matches each camera to front exposure timestamps. Export is restricted to
the common time window, so an ended stream is never held to fake coverage.
The existing group 11 highway clip and group 22 street/parked-car clip have
been reviewed at 3, 8 and 14 seconds. Group 22 uses its own intrinsics,
extrinsics and ego masks, rather than group 11 calibration.

Within the full calibrated ground extent, excluding the measured vehicle
footprint, observed coverage for group 11 is 60.3% with four cameras and
92.4% with seven; group 22 is 64.3% and 95.7%. These fractions are geometric
mask coverage, not completeness or accuracy of visible objects. Rear-corner
views remove much of the rear missing region; close side gaps remain.

Group 22's PC presentation crops the central 80% of the ground view and adds
static cyan sprite guides. These guides do not measure clearance or predict
motion. Display fill stays separate from strict coverage comparisons.
Elevated nearby vehicles, people and poles still stretch under planar
projection; this review does not establish production-quality 360 AVM.

`tools/download_pc_surround_groups.py` fetches the three additional views and
calibration for groups 01, 02, 05 and 26 using the existing Hugging Face login.
It extracts individual members from remote ZIPs and retains original exposure
timestamps. It does not download whole chunk archives or change the four
original streams. Its default download budget is 2 GB.

`tools/build_pc_surround_groups.py` exports eight rigs: groups 01, 02, 03,
04, 05, 11, 22 and 26. Each gets its own calibration and timestamp review.
`--groups` permits rebuilding selected clips without dropping other exports
from the index. `scripts/run_pc_surround_review.cmd` opens these exports with
clickable numbered scene buttons and PREVIOUS/NEXT controls. Number keys,
left/right arrows, A/D and `[` / `]` also switch scenes when the window has
keyboard focus. Reports and a group index live in `output/pc_surround_groups`.
It contains 1080p source imagery in a 1440x840 dashboard exported at 30 FPS,
with approximately 20 seconds of source footage. Its FPS is presentation
only, with no live YOLO/UFLD inference. SPACE pauses, V enlarges and B shows
the four/seven-camera comparison. Generated videos, maps and reports stay
under `output/`. A board deployment still needs compilation and screenshot
verification; the board was unavailable for the October 11 PC review.

The PC group exporter enables `--seam-refinement`: a static primary-camera
selection uses the minimum singular value of the calibrated projection's
Jacobian, preferring better sampled ground. Seven-pixel feathering blends
only near selection boundaries instead of averaging entire overlaps.
Low-gradient overlap pixels estimate bounded per-camera color offsets
(up to 12 intensity levels) with temporal smoothing. Valid ground masks and
reported coverage do not change. This reduces overlap ghosts and some exposure
steps; it cannot correct nonplanar vehicles, moving-object parallax, curb
height, or missing source coverage. It is a PC experiment; the board renderer
has not received this selection and exposure algorithm.

For the current Windows PC, double-click the repository-root
`启动PC鸟瞰.cmd`; `PC_AVM_START.md` describes controls and Python selection.
The launcher checks OpenCV/NumPy and prefers the configured Python or the
current PC's `D:\python\python.exe` before falling back to PATH. Reopening
brings an existing player forward. The player redraws only on new frames or
control changes; it does not manufacture a higher video or inference FPS.
Enlarged BEV uses a taller canvas so the scene toolbar does not cover ground.

The initial eight exports were 10 FPS. All eight have subsequently been rebuilt
from the original seven streams at 30 FPS to reduce motion steps while
retaining 1x replay. Each group's `previous_10fps` folder retains its old video
and report for comparison.
`build_pc_surround_groups.py --video-fps 30` selects a higher source sampling
rate (now the builder default); this is not interpolation or inference FPS.

All eight clips now use `--antialias-ground` after a PC recording
showed texture flicker. Ground samples integrate a 4x4 subpixel footprint
using calibrated-map interpolation, pyrDown filtering, and an area average.
Invalid sub-samples are excluded, with a point-sample fallback at empty edge
footprints. Ground validity masks and camera tiles do not change. Subsequent
group rebuilds enable this sampling, and the group index records this flag
alongside export FPS. There is no temporal averaging or ground translation.

The player waits according to the next frame deadline and requests 1 ms
Windows timer resolution only for its lifetime, paired with timeEndPeriod
on exit. Optional `--diagnostics` reports GUI submission intervals and skipped
video frames to stdout, not hardware presentation or inference latency.

## PC alignment audit (2026-10-11)

The calibration builder now respects each FTheta model's reference polynomial,
its `linear_cde` scale/shear, and bivariate windshield distortion. Previously it
always inverted the backward polynomial and omitted the last two terms. This
affected groups 05, 22 and 26; their exports have been rebuilt. Group 02's maps
were unchanged by this fix. Model equations follow
[NVIDIA NCore sensor models](https://nvidia.github.io/ncore/data/sensor_models.html).
`tests/nvidia_calibration_regression.py` checks analytic forward/backward,
linear scale/shear, windshield and invalid-domain cases. New maps record clip
identity, and exports reject a mismatched calibration.

Press S in the PC player to mark the actual static camera boundaries; V enlarges
the BEV and SPACE pauses it. The overlay uses the renderer's own camera weights
and does not warp images or add inference. Check road markings at these seams;
vehicle roofs and curb heights are not coplanar with road texture.

Group 02's camera timestamps still differ by up to 12.7 ms. A test using matching
ego-motion showed mixed correspondence changes, so unvalidated compensation is
not enabled. Fixed ground projection also does not compensate rolling shutter,
road grade/bank or camera pitch/roll changes. The alignment audit does not
establish fully aligned, stable 360-degree AVM. Pre-fix videos and maps for the
three affected groups are retained in `previous_incomplete_intrinsics`.
