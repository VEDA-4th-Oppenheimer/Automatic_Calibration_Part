foreach(required RUNNER INPUT_DIR OUTPUT_DIR INTRINSIC EXPECTED_EXIT
                 EXPECTED_STATUS EXPECTED_REASON EXPECTED_CANDIDATE_STATUS)
  if(NOT DEFINED ${required})
    message(FATAL_ERROR "Missing -D${required}=...")
  endif()
endforeach()

set(result_file "${OUTPUT_DIR}/calibration_result.json")
file(REMOVE "${result_file}")
execute_process(
  COMMAND "${RUNNER}"
    --input-dir "${INPUT_DIR}"
    --output "${OUTPUT_DIR}"
    --manual-intrinsic-json "${INTRINSIC}"
    --image-distortion-state raw
    --ldc-enabled false
    --camera-channel 1
    --camera-center-x-m 0.05928
    --camera-center-y-m -0.08105
    --camera-center-z-m 0.0
    --search-strategy staged
    --holdout-count 1
  RESULT_VARIABLE runner_result
  OUTPUT_VARIABLE runner_output
  ERROR_VARIABLE runner_error
)

if(NOT "${runner_result}" STREQUAL "${EXPECTED_EXIT}")
  message(FATAL_ERROR
    "Expected exit ${EXPECTED_EXIT}, got ${runner_result}.\n"
    "${runner_error}\n${runner_output}")
endif()
if(NOT EXISTS "${result_file}")
  message(FATAL_ERROR "Expected result JSON was not created: ${result_file}")
endif()

file(READ "${result_file}" result_json)
function(assert_json_value expected)
  string(JSON actual ERROR_VARIABLE json_error
    GET "${result_json}" ${ARGN})
  if(NOT json_error STREQUAL "NOTFOUND")
    message(FATAL_ERROR "Cannot read JSON path ${ARGN}: ${json_error}")
  endif()
  if(NOT "${actual}" STREQUAL "${expected}")
    message(FATAL_ERROR
      "Expected JSON path ${ARGN}=${expected}, got ${actual}")
  endif()
endfunction()

assert_json_value("${EXPECTED_STATUS}" status)
assert_json_value("${EXPECTED_REASON}" reason_code)
assert_json_value("${EXPECTED_CANDIDATE_STATUS}" candidate_rt_status)
assert_json_value("NOT_PRODUCT_APPROVED_RT" product_approved_rt_status)
assert_json_value("OFF" activation_allowed)
assert_json_value(
  "fixed_per_finalist_training_seed_prior_reused_for_training_and_holdout"
  algorithm manhattan_image_feature_prior_policy)

message(STATUS
  "Real calibration contract verified: exit=${runner_result}, "
  "status=${EXPECTED_STATUS}, reason=${EXPECTED_REASON}")
