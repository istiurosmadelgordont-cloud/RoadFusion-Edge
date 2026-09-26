# RoadFusion-Edge ARM software architecture

The board application follows the nine-layer architecture in the project
diagram.  The diagram is a target architecture, so every module is also marked
with its current implementation status below.  A module is not considered
complete merely because a UI card exists.

| Layer | Runtime responsibility | Current board status |
| --- | --- | --- |
| 1. Input and state | FPGA-style 1920x1080 four-view composite; vehicle state | Four-view video simulation is active. FPGA live input and CAN are adapters still to be connected. |
| 2. Hardware abstraction and sync | MPP decode, RGA conversion, frame identity, calibration, vehicle-state freshness | RGA NV12 conversion and synchronized file playback exist. The current launch script deliberately uses software decode; FPGA DMA and CAN are not active. |
| 3. Scheduling | Serialize YOLO/UFLD work on the RK3568 NPU and keep UI independent | Active. Object, front-lane and rear-lane jobs share one NPU queue. |
| 4. Perception | YOLOv8n-P2, UFLD V2, ByteTrack | Active. Outputs remain image coordinates and carry source-frame timing. |
| 5. Semantic fusion | Track association, lane marking/crossing/TLC, ROI filtering and directional light matching | Logic V2 is active as a conservative demo. Weak evidence produces `UNKNOWN`. |
| 6. ADAS logic | FCW/RCW/BSD/LDW, lane-change FSM, intersection logic, global ADAS state | Warning functions are demo-grade. Lane-change/intersection state machines provide advice only and never command steering or braking. |
| 7. Decision management | Priority, conflict suppression, primary warning and event output | Logic V2 warning priority is active. Persistent event logging is pending. |
| 8. Output | 1080p UI, warnings, playback, screenshots and logs | UI/playback/log output is active. Audio/LED and production recording are pending. |
| 9. Extensions | Traffic-light ROI model, guide-arrow subclassification, AVM, Lanelet2 rules, OTA | Planned interfaces; not production functionality. |

## Safety and validity rules

- Missing or stale vehicle, lane, side-view, route or traffic-light data is
  represented as unknown. Zero is never used as a substitute for missing CAN.
- A lane boundary permits a lane-change hint only after repeated dashed-line
  evidence. Solid or unknown boundaries block the hint.
- The target-side blind spot must remain clear for 500 ms before an allow hint.
- Crosswalk, guide-arrow or traffic-light evidence blocks lane-change
  confirmation because no HD-map distance is available yet.
- Directional traffic lights are matched to the current route/turn intent.
  Unmatched and unstable lights produce `WAIT` or `UNKNOWN`, never `GO`.
- FCW has the highest warning priority, followed by RCW, lane-change abort,
  BSD, LDW, intersection stop and advisory messages.

## Open-source design references

The implementation is original and intentionally smaller than the referenced
projects.  It adopts these design ideas:

- openpilot: field-valid vehicle state, same-side blind-spot lane-change gate,
  explicit events and alert selection.
- Autoware Universe: separate rule and safety checks, directional traffic-light
  matching, and restrictions near intersections/crosswalks/traffic lights.
- Lanelet2: dashed boundaries permit crossing; solid and unknown boundaries do
  not.

The current system is an ADAS demonstration. It does not output steering,
throttle or brake commands.

Implementation and review requirements for all later modules are defined in
[`ENGINEERING.md`](ENGINEERING.md).
