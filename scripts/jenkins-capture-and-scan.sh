#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage: jenkins-capture-and-scan.sh

Jenkins environment:
  CCTV_BASE_URL              Camera base URL (default: http://172.20.32.43)
  CCTV_USER                  Camera user (default: admin)
  CCTV_PASSWORD              Camera password (required; Jenkins credential)
  CCTV_CHANNELS              Space-separated channels (default: "1 2 3 4")
  LIDAR_URL                  LiDAR web service (default: http://172.20.26.191:8080)
  LIDAR_SCAN_TIMEOUT_SECONDS Maximum wait for scan completion (default: 3600)
  LIDAR_POLL_SECONDS         Event polling interval (default: 5)
  LIDAR_WAIT_FOR_RESULT      Download PCD after scan (default: 1; set 0 to return after request)
  JENKINS_DATA_DIR            Output root (default: $WORKSPACE/data/jenkins-capture)
EOF
}

[[ "${1:-}" != "--help" ]] || { usage; exit 0; }

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "${script_dir}/.." && pwd)"
capture_script="${CAPTURE_SCRIPT:-${script_dir}/capture-cctv-snapshot.sh}"
base_url="${LIDAR_URL:-http://172.20.26.191:8080}"
if [[ "${base_url}" != http://* && "${base_url}" != https://* ]]; then
    base_url="http://${base_url}"
fi
channels="${CCTV_CHANNELS:-1 2 3 4}"
timeout_seconds="${LIDAR_SCAN_TIMEOUT_SECONDS:-3600}"
poll_seconds="${LIDAR_POLL_SECONDS:-5}"
wait_for_result="${LIDAR_WAIT_FOR_RESULT:-1}"
output_root="${JENKINS_DATA_DIR:-${WORKSPACE:-${workspace_dir}}/data/jenkins-capture}"
session_id="$(date '+%Y%m%d-%H%M%S')"
run_dir="${output_root}/${session_id}"
camera_dir="${run_dir}/camera"
lidar_dir="${run_dir}/lidar"
event_file="${run_dir}/lidar-events.ndjson"
mkdir -p -- "${camera_dir}" "${lidar_dir}"

[[ -x "${capture_script}" ]] || { echo "Capture script is not executable: ${capture_script}" >&2; exit 2; }
[[ -n "${CCTV_PASSWORD:-}" ]] || { echo 'CCTV_PASSWORD is required for Jenkins; bind it from Credentials.' >&2; exit 2; }
[[ "${timeout_seconds}" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid LIDAR_SCAN_TIMEOUT_SECONDS: ${timeout_seconds}" >&2; exit 2; }
[[ "${poll_seconds}" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid LIDAR_POLL_SECONDS: ${poll_seconds}" >&2; exit 2; }

curl_json() {
    curl --fail --silent --show-error --connect-timeout 10 "$@"
}

cleanup() {
    if [[ -n "${events_pid:-}" ]] && kill -0 "${events_pid}" 2>/dev/null; then
        kill "${events_pid}" 2>/dev/null || true
        wait "${events_pid}" 2>/dev/null || true
    fi
}
trap cleanup EXIT

printf 'session=%s\noutput=%s\n' "${session_id}" "${run_dir}"
echo 'Checking LiDAR HTTP service'
curl_json "${base_url%/}/" >/dev/null

captured_files=()
for channel in ${channels}; do
    echo "Capturing camera channel ${channel}"
    captured="$(
        CCTV_BASE_URL="${CCTV_BASE_URL:-172.20.32.43}" \
        CCTV_USER="${CCTV_USER:-admin}" \
        CCTV_PASSWORD="${CCTV_PASSWORD}" \
        CCTV_PROFILE="${CCTV_PROFILE:-1}" \
        CCTV_COUNT=1 \
        "${capture_script}" "${channel}" "${camera_dir}"
    )"
    printf '%s\n' "${captured}"
    captured_files+=("${captured}")
done

: > "${event_file}"
echo 'Opening LiDAR event stream'
curl --fail --silent --show-error --no-buffer \
    --connect-timeout 10 \
    "${base_url%/}/api/events" > "${event_file}" 2>"${run_dir}/events-curl.log" &
events_pid=$!
sleep 1

echo 'Requesting LiDAR scan'
scan_response="$(curl_json --request POST --header 'Accept: application/json' "${base_url%/}/api/cmd/scan")"
printf '%s\n' "${scan_response}" > "${run_dir}/scan-request.json"
request_id="$(printf '%s' "${scan_response}" | sed -n 's/.*"req_id"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p')"
[[ -n "${request_id}" ]] || { echo "LiDAR response has no req_id: ${scan_response}" >&2; exit 1; }
printf 'scan request id=%s\n' "${request_id}"

if [[ "${wait_for_result}" == 1 ]]; then
    deadline=$((SECONDS + timeout_seconds))
    scan_event=''
    while (( SECONDS < deadline )); do
        if grep -F '"adts/state/scan"' "${event_file}" | grep -Fq "\"req_id\": \"${request_id}\""; then
            scan_event="$(grep -F '"adts/state/scan"' "${event_file}" | grep -F "\"req_id\": \"${request_id}\"" | tail -n 1)"
            break
        fi
        if grep -F '"adts/event/error"' "${event_file}" | grep -Fq "\"req_id\": \"${request_id}\""; then
            error_event="$(grep -F '"adts/event/error"' "${event_file}" | grep -F "\"req_id\": \"${request_id}\"" | tail -n 1)"
            echo "LiDAR scan failed: ${error_event}" >&2
            exit 1
        fi
        sleep "${poll_seconds}"
    done
    [[ -n "${scan_event}" ]] || { echo "LiDAR scan timed out after ${timeout_seconds}s" >&2; exit 1; }
    printf '%s\n' "${scan_event}" > "${run_dir}/scan-result-event.json"
    if printf '%s' "${scan_event}" | grep -Fq '"ok": false'; then
        echo "LiDAR scan completed with failure: ${scan_event}" >&2
        exit 1
    fi
    pcd_path="$(printf '%s' "${scan_event}" | sed -n 's/.*"pcd"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p')"
    pcd_name="${pcd_path##*/}"
    [[ -n "${pcd_name}" && "${pcd_name}" == *.pcd ]] || { echo "Scan event has no PCD path: ${scan_event}" >&2; exit 1; }
    echo "Downloading ${pcd_name}"
    curl_json "${base_url%/}/api/scan/${pcd_name}" > "${lidar_dir}/${pcd_name}"
    [[ -s "${lidar_dir}/${pcd_name}" ]] || { echo 'Downloaded PCD is empty.' >&2; exit 1; }
fi

{
    printf '{\n'
    printf '  "session_id": "%s",\n' "${session_id}"
    printf '  "jenkins_build": "%s",\n' "${BUILD_TAG:-unknown}"
    printf '  "lidar_url": "%s",\n' "${base_url}"
    printf '  "camera_channels": "%s",\n' "${channels}"
    printf '  "camera_files": ['
    for i in "${!captured_files[@]}"; do
        (( i > 0 )) && printf ', '
        printf '"%s"' "${captured_files[$i]}"
    done
    printf ']'
    if [[ -n "${pcd_name:-}" ]]; then
        printf ',\n  "lidar_file": "%s"\n' "${pcd_name}"
    else
        printf '\n'
    fi
    printf '}\n'
} > "${run_dir}/manifest.json"

echo "Completed: ${run_dir}"
