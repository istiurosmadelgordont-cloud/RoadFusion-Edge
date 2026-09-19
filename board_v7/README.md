# RK3568 V7 single-model demo

This board program uses the preserved 21-class V7 P2 model. Classes 7–14 are
red/green circle, left, right and straight traffic lights in the exact order
defined by `adas_training/configs/unified21_names.yaml`. It is not a 23-class
model. The old 17-class board program remains in the sibling `board` folder.

The PC pipeline exports `unified21_p2_v7_domestic_preserved.pt` to a 640-pixel
Rockchip ONNX model, selects 400 class-balanced training images for INT8
calibration, and converts that graph with RKNN Toolkit2 1.5.0 for `rk3568`.
The installed RKNN model is `models/unified21_p2_v7_640_int8.rknn`.

On the board, build and run from `/home/cat/rk3568_adas/board_v7`:

```sh
./scripts/build.sh
./scripts/run.sh --source samples/project_video.mp4
./scripts/run.sh --source test_inputs/ccf7_full_01470.jpg --headless --dump-detections
./scripts/run_four_view.sh
```

The four-view demo first builds the same 1920x1080 four-quadrant frame that
the production FPGA will output: front and rear on the top row, left and right
on the bottom row. That full frame is letterboxed to the fixed 640x640 RKNN
input and sent through one NPU call. The video area in the UI is 1280x720
(four 640x360 tiles). `--legacy-mosaic` retains the earlier direct 640x640
composition for A/B checks. Press `C` to calibrate the front lane or
`R` for the rear lane. Playback freezes while four points on the two lane
markings are selected, then resumes after the ROI is validated and saved in
`config/front_lane_roi.txt` or `config/rear_lane_roi.txt`. The green ROI and
`CALIBRATED / LANE FOUND` status stay visible so loading the calibration can
be distinguished from a temporary lane miss. The bottom bar can be clicked
or dragged to seek all four clips to the same frame.

Each view needs four clicks. For both front and rear, use the same clockwise
order: **near left (bottom-left) -> far left (top-left) -> far right
(top-right) -> near right (bottom-right)**. The program sorts the four points
into `top-left, top-right, bottom-right, bottom-left`, so click order is not
strict, but the recommended order makes mistakes easy to see. Put every point
on the corresponding lane marking and keep the upper pair on roughly the same
row. Front and rear therefore use eight points in total.

Use the `PREV SCENE` and `NEXT SCENE` buttons, or the `[` and `]` keys, to
cycle through the bundled synchronized scenes. Switching resets decoding,
the timeline, ByteTrack, collision state and lane history. Lane geometry uses
quadratic fitting on both the bird-eye path and the Hough fallback; display
points interpolate between lane updates to avoid six-frame jumps.

The right sidebar includes a lightweight surround-location display. It keeps
the ego vehicle at the center, draws the current front/rear lane corridors,
and places the largest tracked vehicles and pedestrians around it by camera
direction and apparent image distance. Track numbers come from ByteTrack. This
is a low-cost ADAS visualization rather than metric BEV; real-world positions
require camera intrinsics, extrinsics and ground-plane calibration.

To keep the Cortex-A55 UI responsive, the camera textures and safety overlays
still refresh every displayed frame, lane extraction alternates front/rear at
one update every two displayed frames, and dashboard text refreshes every
third frame. On the test board the FPGA-style composite path measured roughly
12-14 FPS with the GLES UI and about 19 FPS headless, depending on the scene.

`run.sh` uses `taskset -c 2,3` and two OpenCV worker threads. Linux CPU IDs
2 and 3 are the third and fourth Cortex-A55 cores. RKNN Runtime invokes the
NPU independently; CPU affinity does not assign NPU cores on RK3568.
Inference defaults to every second video frame to keep the UI responsive.
The V7 P2 decoder checks for four 21-class score outputs before running.

## Warning logic used by the four-view demo

- Lane departure is measured relative to the calibrated ROI center. A warning
  needs three reliable lane updates above the entry threshold and clears only
  after four updates below the lower exit threshold. A lane inferred from one
  visible marking may be drawn, but cannot start a warning.
- Front/rear collision selection uses the detected lane polygon and keeps the
  same ByteTrack ID across detection intervals. Distance, closing speed and TTC
  update only on fresh NPU measurements; target changes reset the speed history.
- The front wide-angle and rear telephoto NVIDIA cameras use separate focal
  scales. Collision alerts and blind-spot alerts use consecutive-update
  confirmation and hysteresis instead of a one-frame trigger.
- The 1920x1080 dashboard is rendered in Chinese. Technical labels such as
  FPS, NPU, TTC, YOLOv8 and detector class names remain in English.

The eight light classes are displayed. Forward driving prompts use only
circle and straight detections because left/right applicability requires lane
or navigation context. These prompts are for demonstration, not vehicle
control. V7 still has some arrow/circle confusion: on CCF frame 01470 the
right-hand red arrow is labeled `traffic_red_circle`; the original PyTorch
weight makes the same error. CCF frame 01445 has labeled small green arrows
that both the PT and RKNN models miss at confidence 0.25. Quantization did not
create those two failures.

Smoke checks on the board (RKNN Runtime 1.5.0, driver 0.8.2): one source image
gave a car box at `(1233.64, 976.32, 1270.49, 1008.03)` with confidence 0.385;
the PT model gave `(1233.7, 974.7, 1274.3, 1011.6)` with confidence 0.358.
On a 1080p sample video, headless 30-frame every-frame inference ran at about
5.6 FPS, while 60 frames with every-second-frame inference ran at about
11.1 FPS. Display FPS may be lower.
