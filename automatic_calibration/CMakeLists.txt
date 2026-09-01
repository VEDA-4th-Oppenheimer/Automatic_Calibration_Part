cmake_minimum_required(VERSION 3.20)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(automatic_calibration LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
  enable_testing()
endif()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS calib3d core imgcodecs imgproc)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(automatic_calibration_core src/synthetic_lidar.cpp src/calibration_core.cpp)
target_include_directories(automatic_calibration_core PUBLIC include)
target_link_libraries(automatic_calibration_core PUBLIC Eigen3::Eigen opencv_core opencv_imgcodecs opencv_imgproc Ceres::ceres yaml-cpp nlohmann_json::nlohmann_json)
target_compile_options(automatic_calibration_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(generate_synthetic_scan apps/generate_synthetic_scan.cpp)
target_link_libraries(generate_synthetic_scan PRIVATE automatic_calibration_core)
target_compile_options(generate_synthetic_scan PRIVATE -Wall -Wextra -Wpedantic)
add_executable(run_synthetic_calibration apps/run_synthetic_calibration.cpp)
target_link_libraries(run_synthetic_calibration PRIVATE automatic_calibration_core)
add_executable(run_multi_synthetic_calibration apps/run_multi_synthetic_calibration.cpp)
target_link_libraries(run_multi_synthetic_calibration PRIVATE automatic_calibration_core)
target_compile_options(run_multi_synthetic_calibration PRIVATE -Wall -Wextra -Wpedantic)
add_executable(render_calibration_visualization apps/render_calibration_visualization.cpp)
target_link_libraries(render_calibration_visualization PRIVATE automatic_calibration_core)
target_compile_options(render_calibration_visualization PRIVATE -Wall -Wextra -Wpedantic)
add_executable(run_real_calibration apps/run_real_calibration.cpp)
target_link_libraries(run_real_calibration PRIVATE automatic_calibration_core opencv_calib3d)
target_compile_options(run_real_calibration PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(run_synthetic_calibration PRIVATE -Wall -Wextra -Wpedantic)

add_executable(synthetic_lidar_tests tests/synthetic_lidar_tests.cpp)
target_link_libraries(synthetic_lidar_tests PRIVATE automatic_calibration_core)
target_compile_options(synthetic_lidar_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME automatic_synthetic_lidar_tests COMMAND synthetic_lidar_tests)
add_executable(calibration_core_tests tests/calibration_core_tests.cpp)
target_link_libraries(calibration_core_tests PRIVATE automatic_calibration_core)
target_compile_options(calibration_core_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME automatic_calibration_core_tests COMMAND calibration_core_tests)

add_executable(challenger_m1_2_stress_tests tests/challenger_m1_2_stress_tests.cpp)
target_link_libraries(challenger_m1_2_stress_tests PRIVATE automatic_calibration_core)
target_compile_options(challenger_m1_2_stress_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME challenger_m1_2_stress_tests COMMAND challenger_m1_2_stress_tests)

add_executable(challenger_m2_1_ambiguity_tests tests/challenger_m2_1_ambiguity_tests.cpp)
target_link_libraries(challenger_m2_1_ambiguity_tests PRIVATE automatic_calibration_core)
target_compile_options(challenger_m2_1_ambiguity_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME challenger_m2_1_ambiguity_tests COMMAND challenger_m2_1_ambiguity_tests)

add_executable(challenger_m2_2_stress_tests tests/challenger_m2_2_stress_tests.cpp)
target_link_libraries(challenger_m2_2_stress_tests PRIVATE automatic_calibration_core)
target_compile_options(challenger_m2_2_stress_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME challenger_m2_2_stress_tests COMMAND challenger_m2_2_stress_tests)

add_executable(challenger_m3_stress_tests tests/challenger_m3_stress_tests.cpp)
target_link_libraries(challenger_m3_stress_tests PRIVATE automatic_calibration_core)
target_compile_options(challenger_m3_stress_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME challenger_m3_stress_tests COMMAND challenger_m3_stress_tests)

add_test(NAME verify_20260818_staged COMMAND ${CMAKE_COMMAND}
  -DRUNNER=$<TARGET_FILE:run_real_calibration>
  -DINPUT_DIR=/workspace/data/real_calibration/session-const-env/repeat_test_sample/20260818
  -DOUTPUT_DIR=/workspace/automatic_calibration/generated/verify_20260818
  -DINTRINSIC=/workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json
  -DEXPECTED_EXIT=3
  -DEXPECTED_STATUS=FAIL
  -DEXPECTED_REASON=FINALIST_AMBIGUOUS
  -DEXPECTED_CANDIDATE_STATUS=NOT_CANDIDATE_RT
  -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/verify_real_calibration_result.cmake
)
set_tests_properties(verify_20260818_staged PROPERTIES
  LABELS "weekly;real_data;expected_rejection"
)
add_test(NAME verify_20260819_staged COMMAND ${CMAKE_COMMAND}
  -DRUNNER=$<TARGET_FILE:run_real_calibration>
  -DINPUT_DIR=/workspace/automatic_calibration/generated/repeat_test_sample_20260819_install_shift_ch1_v3/input
  -DOUTPUT_DIR=/workspace/automatic_calibration/generated/verify_20260819
  -DINTRINSIC=/workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json
  -DEXPECTED_EXIT=0
  -DEXPECTED_STATUS=CANDIDATE_RT
  -DEXPECTED_REASON=PASS
  -DEXPECTED_CANDIDATE_STATUS=CANDIDATE_RT
  -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/verify_real_calibration_result.cmake
)
set_tests_properties(verify_20260819_staged PROPERTIES
  LABELS "weekly;real_data;candidate_regression"
)
