## Repo overview

- Language: C++ (C++11/C++14 style). Build system: CMake. Primary binary target: `Armor` (defined in `main/CMakeLists.txt`).
- Layout: source in `main/`, public headers in `include/`, build output in `build/`, calibration/data in `src/`.

## Big-picture architecture

- The project detects and pairs light bars to form "armors" (see [main/Armors.cpp](main/Armors.cpp)).
- Key responsibilities:
  - Image preprocessing & contour detection: `ArmorsDetector::preprocessImage` and `detectLights` in `main/Armors.cpp`.
  - Pairing & filtering heuristics: `matchArmors` (distance/angle/ratio checks) in the same file.
  - Pose solving lives under `PoseSlove.hpp` / `PoseSlove.cpp` and is used to estimate 3D pose of detected armors.

## Important files to inspect first

- [CMakeLists.txt](CMakeLists.txt) — top-level CMake, finds OpenCV and sets `HIK_MVS_PATH` include/link dirs.
- [main/CMakeLists.txt](main/CMakeLists.txt) — defines `add_executable(Armor Main.cpp draw.cpp Armors.cpp PoseSlove.cpp)` and links `MvCameraControl` and `pthread`.
- [main/Armors.cpp](main/Armors.cpp) — main detection/matching logic and many in-code heuristics (ratio/angle/AREA thresholds).
- `include/Congfig.hpp` and `include/Struct.hpp` — hold project-specific constants and data structures (thresholds, macros, config parsing).
- `src/*.yml` — calibration and camera parameters used by pose solving.

## Build / run / debug workflows

- Standard CMake build (preferred):

  ```bash
  cmake -S . -B build
  cmake --build build -j
  cmake --build build --target Armor
  ./build/main/Armor   # or run the produced executable under build/
  ```

- If you need quick one-file compile (VS Code task provided): the workspace contains a `C/C++: g++` task that compiles the active file with OpenCV flags (not the canonical project build).
- To enable debug symbols, set debug flags in `main/CMakeLists.txt`:

  ```cmake
  set(CMAKE_BUILD_TYPE Debug)
  set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
  ```

## Project-specific conventions and patterns

- Heuristics are tuned in headers (e.g. `Congfig.hpp`): constants like `RATIO_MIN`, `RATIO_MAX`, `ARMOR_RATIO_MIN`, `AREA`, `ANGLE`, etc., control filtering. Changes go there, not scattered across files.
- Debugging is done via `std::cout` and `cv::imshow` in the detection pipeline — expect GUI windows during runtime; headless CI runs will need these removed or gated.
- Defensive checks and explicit initialization are used (e.g. checks for empty `cv::Mat`, explicit `eps` to avoid div-by-zero). Follow that style: prefer explicit initialization and small epsilon guards.
- Code uses OpenCV `findContours`, `minAreaRect`, and a mix of `fitEllipse` fallback — watch commented fallbacks in `Armors.cpp` for robustness patterns.

## External dependencies & integration points

- OpenCV (found via `find_package(OpenCV REQUIRED)` in top-level CMake).
- HIK/MVS SDK at `/opt/MVS` — `HIK_MVS_PATH` is referenced in `CMakeLists.txt`; runtime requires the SDK libs and the `MvCameraControl` link target.
- Camera/calibration files under `src/` (e.g. `calib_result.yml`) are used by pose solving; ensure paths are correct when running the binary.

## Typical agent tasks & examples

- When tuning detection thresholds: update `include/Congfig.hpp`, rebuild, and run `./build/main/Armor` to visually verify `cv::imshow` masks (see `ArmorsDetector::preprocessImage`).
- When adding a new source file: add it to `main/CMakeLists.txt` in the `add_executable(Armor ...)` list.
- When debugging linking errors for the camera SDK: confirm `HIK_MVS_PATH` folder exists and contains `lib/64` libs; runtime LD_LIBRARY_PATH may need `/opt/MVS/lib/64`.

## Caution / gotchas

- GUI calls (`cv::imshow`) block/require `cv::waitKey` elsewhere — headless runs may hang.
- Build artifacts live in `build/`; cleaning `build/` is safe to force a full rebuild.
- Many magic constants drive behavior — always search `include/` for definitions before editing numeric thresholds.

## When in doubt — quick file pointers

- Detection & pairing: [main/Armors.cpp](main/Armors.cpp)
- Build targets: [main/CMakeLists.txt](main/CMakeLists.txt)
- Global config & constants: [include/Congfig.hpp](include/Congfig.hpp)

---
If you want, I can (1) run a quick build here, (2) add a short README section for running the binary, or (3) expand examples for modifying config thresholds. 哪个优先？
