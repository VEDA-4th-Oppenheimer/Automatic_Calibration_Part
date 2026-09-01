# Manual Projection Reference Refiner MVP

- Date: 2026-08-30
- Branch: exp-manual-reference-refiner
- Scope: operator-assisted projection inspection and manual RT candidate capture
- Status: diagnostic/manual candidate only

## 1. Purpose and non-goals

This OpenCV HighGUI utility lets a human inspect a LiDAR-to-camera
projection and adjust the pose while viewing the camera image. It does not
search keys automatically, optimize RT, score alignment, declare a correct
alignment, or promote a product/reference RT.

Workflow:

1. Start from an existing RT candidate.
2. Display the camera image with LiDAR points projected on top.
3. Adjust rotation and, only when explicitly unlocked, translation.
4. Save the operator candidate and adjustment history.
5. Use the result as diagnostic evidence when evaluating automatic calibration.

The tool is separate from Calibration Core, the optimizer, analyzer, K/D
estimation, and the product execution path.

## 2. OpenCalib relationship

The interaction concept is inspired by the manual_calib workflow in OpenCalib:
show a projection, expose small pose increments, and let an operator inspect
the result. This implementation is standalone. It does not copy OpenCalib
source, link against it, or add it as a dependency. It reuses the existing
project camera model loader and JSON transform parser.

## 3. Coordinate and data contracts

### 3.1 RT convention

The existing project convention is:

    p_camera = R_camera_lidar * p_lidar + t_camera_lidar

The parent frame is camera_optical and the child frame is lidar_scan.
Rotation is a proper 3x3 matrix and translation is metres.

The initial RT loader accepts the R1 JSON shape with selected_rt and reads the
nested rotation_matrix and translation_m through the existing manual-marker
JSON API. A plain transform JSON is also accepted.

### 3.2 LiDAR reconstruction

The input is raw organized JSON, not PCD. For every valid measurement:

    range_m = distance_m + sensor.range_offset_m
    x = range_m * cos(tilt_rad) * sin(pan_rad)
    y = -range_m * sin(tilt_rad)
    z = range_m * cos(tilt_rad) * cos(pan_rad)

The range offset is applied exactly once. Invalid, non-finite, and
non-positive measurements are skipped. Row/column metadata is retained for
provenance; the current renderer does not resample the organized grid.

### 3.3 Intrinsics and distortion

The image and K/D must be in the same distortion domain. The tool loads K/D
from the supplied camera_intrinsic.json and calls OpenCV projectPoints. It does
not undistort the image and does not estimate or modify K/D.

Verified build51 clean18 profile:

- resolution: 2592 x 1520
- fx: 2033.9019520107618
- fy: 2037.7796376946073
- cx: 1337.029701465088
- cy: 745.3700555812936
- D: [-0.5653174394854492, 0.34459385610316223,
  -0.0039145365522611315, 0.0008182748566205869,
  -0.10809412486837452]

The values are loaded from the supplied file, not compiled into the refiner,
and are not a product approval decision.

## 4. Baseline lock

Default mode is baseline locked. The physical camera-center contract is:

    C_lidar = [0, -0.08105, 0] m

For every rotation change:

    t_camera_lidar = -R_camera_lidar * C_lidar

Thus the translation norm remains 0.08105 m while locked. Translation keys are
recorded as unchanged no-op attempts and do not change the pose.

The lock preserves the known physical camera-center contract while the
operator inspects orientation. It is not an automatic constraint solver and
does not prove that the measured center is an independent optical-center
survey.

Press l to unlock translation. In that mode translation keys directly modify
the camera-frame t vector and the UI shows UNCONSTRAINED DIAGNOSTIC. Saved
output has baseline_locked=false, constrained=false, and status
MANUAL_UNCONSTRAINED_DIAGNOSTIC. Do not use an unlocked result as product RT
or ground truth.

## 5. Build and run

The executable is manual_projection_refiner in
manual_calibration/CMakeLists.txt.

### 5.1 Native WSL/Ubuntu build

    cd /mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop
    cmake -S manual_calibration -B /tmp/manual-refiner-build
    cmake --build /tmp/manual-refiner-build -j2
    /tmp/manual-refiner-build/bin/manual_projection_refiner --help

### 5.2 Verified build51 command

    /tmp/manual-refiner-build/bin/manual_projection_refiner       --image /mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop/data/jenkins-capture/scene0/calib_dataset_build51_20260830_010816/20260830_005806_CH1.jpg       --scan-json /mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop/data/jenkins-capture/scene0/calib_dataset_build51_20260830_010816/calib-20260830-095830_sweep-000001_pan_tilt_lidar.json       --intrinsic /mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json       --initial-rt /mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/analyzer_experiments/v3_hybrid/automatic_calibration/generated/deadline_recovery_r1_20260830/TARGET_CONSTRAINED_DEMO_RT.json       --output /mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop/manual_calibration/output/manual_projection_refiner_build51_20260830       --camera-center-x-m 0 --camera-center-y-m -0.08105 --camera-center-z-m 0       --non-interactive

The non-interactive option renders the initial baseline-locked projection and
metadata, then exits. The smoke alias has the same behavior.

### 5.3 Windows, WSL, Ubuntu, and Docker GUI

The GUI uses OpenCV HighGUI. Native Ubuntu with a desktop, or WSLg with the
Linux GUI environment available, is the simplest interactive setup. Native
WSLg normally exposes DISPLAY, WAYLAND_DISPLAY, and XDG_RUNTIME_DIR to Linux
programs.

A Docker container does not automatically inherit a GUI display. The verified
Docker path is non-interactive smoke/CI. For interactive Docker use, the
operator must explicitly forward the host X11 or WSLg socket and display
environment, and mount the input/output paths. If that forwarding is absent,
a black/no-window result is a display setup issue, not a calibration algorithm
result.

## 6. Interactive controls

Rotation is displayed as camera_axis_rot_x/y/z_deg relative to the loaded
initial orientation. The UI uses mathematical X/Y/Z labels, not
project-specific yaw/down/roll names.

| Key | Action |
|---|---|
| q / a | rotation X plus / minus |
| w / s | rotation Y plus / minus |
| e / d | rotation Z plus / minus |
| r / f | translation X plus / minus |
| t / g | translation Y plus / minus |
| y / h | translation Z plus / minus |
| 1 | rotation step 5 degrees |
| 2 | rotation step 1 degree |
| 3 | rotation step 0.1 degree |
| 4 | rotation step 0.05 degree |
| 5 | translation step 10 mm |
| 6 | translation step 1 mm |
| 7 | translation step 0.1 mm |
| z | raw points / nearest-per-pixel z-buffer |
| c | distance color / signal-strength color |
| + / - | increase / decrease point radius |
| l | toggle baseline lock |
| u | undo last state-changing action |
| 0 | reset to loaded orientation and current lock policy |
| v | save current candidate |
| Esc | exit without saving |

Rotation is implemented as R_new = R_delta * R_current, so the adjustment axes
are the camera-parent frame axes used by the UI. A locked rotation is followed
immediately by the physical-center translation recomputation.

The UI reports mode, color source, local rotation vector, translation and
norm, camera-center contract, steps, point radius, projected/front count,
in-frame/visible count, behind-camera count, lock mode, action count, and
selected image/scan paths.

### 6.1 Raw and z-buffer rendering

Raw mode draws every projected point in front of the camera and inside the
image. Z-buffer mode uses a point-radius neighborhood: with the default
radius 1, each projected point competes in a 3 x 3 pixel window and the
nearest camera-depth source wins each overlapping window cell. A source point
is retained if it wins at least one cell. This replaces the old single
integer-pixel comparison. It is a minimal visibility aid, not surface
reconstruction or an occlusion solver. Behind-camera and out-of-frame points
are omitted and counted. The UI and JSON report depth-filtered points; this
can be zero or small when projected points are spatially separated.

Distance is the default color. If c is pressed and visible points have finite
signal_strength, signal coloring is used; otherwise the renderer falls back to
distance.

## 7. Saved candidate

Pressing v creates:

    output/manual_reference_candidate/
      manual_reference_rt.json
      manual_projection_raw.png
      manual_projection_zbuffer.png
      adjustment_history.csv
      session_metadata.json
      MANUAL_REFERENCE_REFINEMENT_REPORT.md

The RT JSON stores parent/child frames, exact transform convention, R,
quaternion, t, camera-center contract, translation norm, lock/constrained
state, status, all input paths, SHA-256 for initial RT and intrinsic, the
range-offset-applied-once flag, adjustments, action count, creation time, and
the following safety fields:

    reference_ground_truth=false
    product_rt=false
    allowed_use=MANUAL_REFERENCE_CANDIDATE_ONLY

The adjustment history records every rotation/translation attempt, whether it
changed the state, active steps, lock state, and resulting translation. A
locked translation attempt is recorded with changed=false.

Non-interactive output contains initial raw/z-buffer overlays and
session_metadata.json. It records zbuffer_window_radius_px,
zbuffer_window_size_px, and initial_zbuffer_removed_points. The same
information is included in manual_reference_rt.json after v. The manual
candidate report is created only after v.

## 8. Operator procedure

1. Confirm the selected image, raw JSON, intrinsic profile, and initial RT in
   startup text and UI.
2. Start with raw mode and distance coloring.
3. Toggle z-buffer with z when dense regions obscure each other.
4. Use 5 degree steps to approach the expected basin, then 1, 0.1, and
   0.05 degree steps for visual refinement.
5. Keep baseline locked unless a separate diagnostic authorizes translation.
6. Inspect both raw and z-buffer views, then press v to save.
7. Treat the result as a manual reference candidate for comparison and
   debugging, never as ground truth or automatic/product RT.
8. Keep generated output outside Git unless an intentional small audit artifact
   is requested.

## 9. Verification for this MVP

Run in the Ubuntu Docker build environment:

- CMake configure and build with OpenCV core, imgcodecs, imgproc, calib3d,
  objdetect, aruco, and highgui
- manual_projection_refiner --self-test
- existing manual_marker_tests through CTest
- build51 non-interactive smoke
- metadata finite-value check
- SHA-256 equality for image, scan JSON, intrinsic, and initial RT
- PNG output format and 2592 x 1520 dimensions

Verified build51 facts:

- source measurements: 40400
- valid LiDAR points: 40189
- organized scan: 101 rows x 400 columns
- range offset: 0.084 m, applied once
- initial raw: 17550 front, 2703 in-frame
- initial z-buffer: 2701 visible
- initial z-buffer window: 3 x 3 (radius 1), 2 points depth-filtered
- behind-camera: 22639

The refiner did not change Calibration Core or any automatic result.

## 10. Known limits

- No target detection or automatic pose selection.
- No K/D estimation or focus/zoom correction.
- No project-specific pan/tilt/yaw/down/roll inference.
- No surface reconstruction or image-edge score.
- Manual visual alignment remains subjective; preserve history and input hashes.
- The physical camera-center contract is applied by the lock but is not itself
  independent survey evidence.
