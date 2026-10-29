# Board software engineering rules

These rules apply to the current RK3568 application and to later FPGA, CAN,
perception and ADAS extensions. A feature is complete only after it satisfies
the relevant rules and passes the board regression gate.

## Architecture and extension points

- Keep input/HAL, scheduling, perception, semantic fusion, ADAS state,
  decision management and presentation as separate layers. Pass typed results
  between layers; do not make UI code call model internals.
- Every perception result carries camera identity, source frame or timestamp,
  validity and freshness. New sensors enter through adapters and must not
  change the meaning of existing result types.
- Keep NPU work serialized behind the scheduler. Use bounded latest-frame
  work, never an unbounded inference queue. A slower model must not create an
  ever-growing delay.
- Put tunable thresholds and periods in named configuration structures. Avoid
  adding unexplained constants in the frame loop.

## Robustness and safe behavior

- Treat missing, malformed, contradictory and stale data as `UNKNOWN` or
  invalid. ADAS decisions fail closed: unknown lane rules cannot authorize a
  lane change, and an unmatched signal cannot produce `GO`.
- Bound file sizes, image dimensions, tensor shapes, queue lengths and time
  windows before allocating memory or indexing buffers. Verify RKNN tensor
  schemas at model load time.
- Own resources with RAII. Do not detach worker threads. Drain asynchronous
  jobs on exit, seek, scene change, calibration and model reload.
- Keep the board demo advisory. Perception or demo state must never directly
  command steering, throttle or braking.

## Security and external inputs

- Consider model files, video paths, FPGA buffers, CAN frames and configuration
  files untrusted inputs. Validate ranges and complete reads before use.
- Avoid shell commands for data paths. If a platform tool has no library API,
  quote every variable and keep the command fixed; never concatenate arbitrary
  options from configuration or network input.
- Do not commit credentials, private keys, board passwords, datasets, generated
  weights or runtime logs. Record upstream source, revision and license for
  imported code or models.
- A future OTA/configuration path must authenticate updates, verify integrity,
  use an atomic rollback-capable install, and reject incompatible schemas.

## Readability and verification

- Prefer small named functions, explicit enums and value objects over integer
  flags. Comments explain constraints and reasons rather than restating code.
- Keep warnings enabled (`-Wall -Wextra`) and add focused regression coverage
  for each decision rule, stale-data path and bug fix.
- Before pushing a board change, run:

  ```sh
  cmake --build build --target adas_four_view --parallel 2
  cmake --build build --target perception_regression --parallel 2
  ./build/perception_regression
  ```

- For scheduling or performance changes, also record real display FPS, model
  execution time, result age, dropped stale results and the exact model/profile.
  FPS alone is not a latency measurement.
