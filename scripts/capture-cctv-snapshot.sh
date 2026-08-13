#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" == "--help" ]]; then
    cat <<'EOF'
Usage: capture-cctv-snapshot.sh [channel] [output-directory]

Environment:
  CCTV_BASE_URL          Camera base URL (required)
  CCTV_USER              Login ID (default: admin)
  CCTV_PASSWORD          Password; prompted when omitted
  CCTV_PROFILE           SUNAPI video profile (default: 1)
  CCTV_COUNT             Number of captures (default: 1)
  CCTV_INTERVAL_SECONDS  Delay between captures (default: 0)

Camera channels are numbered 1 through 4.
EOF
    exit 0
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "${script_dir}/.." && pwd)"
base_url="${CCTV_BASE_URL:-}"
username="${CCTV_USER:-admin}"
channel="${1:-${CCTV_CHANNEL:-1}}"
profile="${CCTV_PROFILE:-1}"
count="${CCTV_COUNT:-1}"
interval="${CCTV_INTERVAL_SECONDS:-0}"
output_dir="${2:-${CCTV_OUTPUT_DIR:-${workspace_dir}/data/camera_capture}}"

CCTV_PASSWORD="${CCTV_PASSWORD:-}"

[[ -n "${base_url}" ]] || {
    echo "Set CCTV_BASE_URL to the camera base URL." >&2
    exit 2
}
[[ "${channel}" =~ ^[1-4]$ ]] || { echo "Invalid channel: ${channel}" >&2; exit 2; }
[[ "${profile}" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid profile: ${profile}" >&2; exit 2; }
[[ "${count}" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid capture count: ${count}" >&2; exit 2; }
[[ "${interval}" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "Invalid interval: ${interval}" >&2; exit 2; }

if [[ -z "${CCTV_PASSWORD:-}" ]]; then
    read -rsp "CCTV password: " CCTV_PASSWORD
    echo
fi

umask 077
mkdir -p -- "${output_dir}"
sunapi_channel=$((channel - 1))
snapshot_url="${base_url%/}/stw-cgi/video.cgi?msubmenu=snapshot&action=view&Profile=${profile}&Channel=${sunapi_channel}"

for ((index = 1; index <= count; ++index)); do
    while :; do
        timestamp="$(date '+%Y%m%d-%H%M%S')"
        output="${output_dir}/${timestamp}-CH${channel}.jpg"
        [[ ! -e "${output}" ]] && break
        sleep 0.1
    done
    temporary="$(mktemp "${output_dir}/.capture-XXXXXX.jpg")"
    trap 'rm -f -- "${temporary}"' EXIT

    content_type="$(curl --digest --fail --silent --show-error --max-time 30 \
        --user "${username}:${CCTV_PASSWORD}" --output "${temporary}" \
        --write-out '%{content_type}' "${snapshot_url}")"
    [[ "${content_type}" == image/jpeg* ]] || {
        echo "Unexpected response type: ${content_type:-unknown}" >&2
        exit 1
    }
    [[ -s "${temporary}" ]] || { echo "Empty snapshot response" >&2; exit 1; }
    mv -- "${temporary}" "${output}"
    trap - EXIT
    echo "${output}"
    ((index == count)) || sleep "${interval}"
done
