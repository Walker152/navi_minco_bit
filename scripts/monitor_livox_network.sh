#!/usr/bin/env bash
set -Eeuo pipefail

# Passive MID-360 Ethernet diagnostics recorder. This script only reads system
# counters and, when explicitly requested, starts a header-only packet capture.

LIDAR_IP=""
INTERFACE=""
INTERVAL_SEC="1"
DURATION_SEC=0
MAX_SAMPLES=0
OUTPUT_ROOT="/tmp/livox_network_monitor"
LIDAR_POINT_PORT=56300
HOST_POINT_PORT=56301
ENABLE_PCAP=0
ENABLE_KERNEL_LOG=1
PCAP_FILTER=""

SESSION_DIR=""
METRICS_FILE=""
EVENTS_FILE=""
PCAP_PID=""
JOURNAL_PID=""
STOP_REQUESTED=0
CLEANED_UP=0
SAMPLE_COUNT=0
ORIGINAL_ARGS=("$@")

usage() {
  cat <<'HELP'
Usage:
  monitor_livox_network.sh --lidar-ip IP [options]

Required:
  --lidar-ip IP             MID-360 IPv4 address

Options:
  --interface IFACE         Network interface; auto-detected from the route by default
  --interval SEC            Sampling interval, default: 1
  --duration SEC            Stop after this many seconds; 0 means no time limit
  --samples COUNT           Stop after this many samples; 0 means no sample limit
  --output DIR              Output root, default: /tmp/livox_network_monitor
  --lidar-point-port PORT   LiDAR point-cloud source port, default: 56300
  --host-point-port PORT    Host point-cloud destination port, default: 56301
  --pcap                    Capture packet headers to mid360_headers.pcap (requires tcpdump)
  --no-kernel-log           Do not follow the kernel journal
  -h, --help                Show this help

Examples:
  sudo ./scripts/monitor_livox_network.sh \
    --lidar-ip 192.168.1.122 --duration 3600

  sudo ./scripts/monitor_livox_network.sh \
    --lidar-ip 192.168.1.122 --interface enp86s0 --pcap

The script never changes network settings or resets counters. Press Ctrl+C to
stop an unlimited run cleanly. Optional tools that are not requested are
recorded as unavailable instead of causing the monitor to stop.
HELP
}

die_usage() {
  echo "[ERROR] $*" >&2
  usage >&2
  exit 2
}

require_value() {
  local option="$1"
  local value="${2-}"
  [[ -n "$value" ]] || die_usage "$option requires a value"
}

is_positive_number() {
  [[ "$1" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] &&
    [[ -n "${1//[0.]/}" ]]
}

is_nonnegative_integer() {
  [[ "$1" =~ ^[0-9]+$ ]]
}

is_valid_port() {
  is_nonnegative_integer "$1" && ((10#$1 >= 1 && 10#$1 <= 65535))
}

is_valid_ipv4() {
  local ip="$1"
  local -a octets
  local octet

  IFS='.' read -r -a octets <<<"$ip"
  [[ ${#octets[@]} -eq 4 ]] || return 1
  for octet in "${octets[@]}"; do
    [[ "$octet" =~ ^[0-9]{1,3}$ ]] || return 1
    ((10#$octet <= 255)) || return 1
  done
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --lidar-ip)
      require_value "$1" "${2-}"
      LIDAR_IP="$2"
      shift 2
      ;;
    --interface)
      require_value "$1" "${2-}"
      INTERFACE="$2"
      shift 2
      ;;
    --interval)
      require_value "$1" "${2-}"
      INTERVAL_SEC="$2"
      shift 2
      ;;
    --duration)
      require_value "$1" "${2-}"
      DURATION_SEC="$2"
      shift 2
      ;;
    --samples)
      require_value "$1" "${2-}"
      MAX_SAMPLES="$2"
      shift 2
      ;;
    --output)
      require_value "$1" "${2-}"
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    --lidar-point-port)
      require_value "$1" "${2-}"
      LIDAR_POINT_PORT="$2"
      shift 2
      ;;
    --host-point-port)
      require_value "$1" "${2-}"
      HOST_POINT_PORT="$2"
      shift 2
      ;;
    --pcap)
      ENABLE_PCAP=1
      shift
      ;;
    --no-kernel-log)
      ENABLE_KERNEL_LOG=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die_usage "unknown argument: $1"
      ;;
  esac
done

[[ -n "$LIDAR_IP" ]] || die_usage "--lidar-ip is required"
is_valid_ipv4 "$LIDAR_IP" || die_usage "invalid IPv4 address: $LIDAR_IP"
is_positive_number "$INTERVAL_SEC" || die_usage "interval must be a positive number"
is_nonnegative_integer "$DURATION_SEC" || die_usage "duration must be a non-negative integer"
is_nonnegative_integer "$MAX_SAMPLES" || die_usage "samples must be a non-negative integer"
is_valid_port "$LIDAR_POINT_PORT" || die_usage "invalid LiDAR point port: $LIDAR_POINT_PORT"
is_valid_port "$HOST_POINT_PORT" || die_usage "invalid host point port: $HOST_POINT_PORT"
PCAP_FILTER="src host ${LIDAR_IP} and udp src port ${LIDAR_POINT_PORT} and dst port ${HOST_POINT_PORT}"

if [[ -z "$INTERFACE" ]]; then
  command -v ip >/dev/null 2>&1 || {
    echo "[ERROR] ip is required to auto-detect the interface" >&2
    exit 1
  }
  INTERFACE="$(
    ip route get "$LIDAR_IP" 2>/dev/null |
      awk '{for (i = 1; i <= NF; ++i) if ($i == "dev") {print $(i + 1); exit}}'
  )"
  [[ -n "$INTERFACE" ]] || {
    echo "[ERROR] unable to determine the interface for $LIDAR_IP" >&2
    exit 1
  }
fi

[[ -d "/sys/class/net/${INTERFACE}" ]] || {
  echo "[ERROR] network interface does not exist: $INTERFACE" >&2
  exit 1
}

if ((ENABLE_PCAP)); then
  command -v tcpdump >/dev/null 2>&1 || {
    echo "[ERROR] --pcap requires tcpdump" >&2
    exit 1
  }
fi

session_stamp="$(date '+%Y%m%d-%H%M%S')"
safe_ip="${LIDAR_IP//./_}"
session_base="${OUTPUT_ROOT%/}/${session_stamp}_${safe_ip}_${INTERFACE}"
SESSION_DIR="$session_base"
session_suffix=0
while [[ -e "$SESSION_DIR" ]]; do
  session_suffix=$((session_suffix + 1))
  SESSION_DIR="${session_base}_${session_suffix}"
done

mkdir -p "$SESSION_DIR"
METRICS_FILE="${SESSION_DIR}/metrics.csv"
EVENTS_FILE="${SESSION_DIR}/events.log"
touch "$EVENTS_FILE"

log() {
  local level="$1"
  shift
  printf '%s [%s] %s\n' "$(date --iso-8601=seconds)" "$level" "$*" |
    tee -a "$EVENTS_FILE"
}

capture_command() {
  local output_file="$1"
  local command_name="$2"
  shift 2

  if ! command -v "$command_name" >/dev/null 2>&1; then
    printf 'UNAVAILABLE: %s\n' "$command_name" >"$output_file"
    return 0
  fi
  if ! "$command_name" "$@" >"$output_file" 2>&1; then
    printf '\nCOMMAND_EXITED_NONZERO: %s' "$command_name" >>"$output_file"
    printf ' %q' "$@" >>"$output_file"
    printf '\n' >>"$output_file"
  fi
}

read_sysfs_counter() {
  local name="$1"
  local path="/sys/class/net/${INTERFACE}/statistics/${name}"

  if [[ -r "$path" ]]; then
    tr -d '\n' <"$path"
  else
    printf 'NA'
  fi
}

read_sysfs_value() {
  local name="$1"
  local path="/sys/class/net/${INTERFACE}/${name}"

  if [[ -r "$path" ]]; then
    tr -d '\n' <"$path"
  else
    printf 'NA'
  fi
}

extract_named_counter() {
  local input="$1"
  shift
  local key value

  for key in "$@"; do
    value="$(
      awk -F: -v wanted="$key" '
        {
          name = $1
          gsub(/^[[:space:]]+|[[:space:]]+$/, "", name)
          if (name == wanted) {
            number = $2
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", number)
            print number
            exit
          }
        }
      ' <<<"$input"
    )"
    if [[ -n "$value" ]]; then
      printf '%s' "$value"
      return 0
    fi
  done
  printf 'NA'
}

extract_nstat_counter() {
  local input="$1"
  local key="$2"
  local value

  value="$(awk -v wanted="$key" '$1 == wanted {print $2; exit}' <<<"$input")"
  if [[ "$value" =~ ^[0-9]+$ ]]; then
    printf '%s' "$value"
  else
    printf 'NA'
  fi
}

read_nstat_output() {
  if command -v nstat >/dev/null 2>&1; then
    nstat -az 2>/dev/null || true
  fi
}

read_ethtool_stats() {
  if command -v ethtool >/dev/null 2>&1; then
    ethtool -S "$INTERFACE" 2>/dev/null || true
  fi
}

read_softnet_totals() {
  local processed dropped squeezed remainder
  local dropped_total=0
  local squeezed_total=0

  if [[ ! -r /proc/net/softnet_stat ]]; then
    printf 'NA,NA'
    return
  fi

  while read -r processed dropped squeezed remainder; do
    [[ "$dropped" =~ ^[[:xdigit:]]+$ ]] || continue
    [[ "$squeezed" =~ ^[[:xdigit:]]+$ ]] || continue
    dropped_total=$((dropped_total + 16#$dropped))
    squeezed_total=$((squeezed_total + 16#$squeezed))
  done </proc/net/softnet_stat

  printf '%s,%s' "$dropped_total" "$squeezed_total"
}

read_livox_socket_totals() {
  local port_hex socket_file local_address queue drops
  local rx_hex
  local socket_count=0
  local rx_queue_total=0
  local drop_total=0

  printf -v port_hex '%04X' "$HOST_POINT_PORT"
  for socket_file in /proc/net/udp /proc/net/udp6; do
    [[ -r "$socket_file" ]] || continue
    while read -r local_address queue drops; do
      [[ "${local_address##*:}" == "$port_hex" ]] || continue
      rx_hex="${queue#*:}"
      [[ "$rx_hex" =~ ^[[:xdigit:]]+$ ]] || rx_hex=0
      [[ "$drops" =~ ^[0-9]+$ ]] || drops=0
      socket_count=$((socket_count + 1))
      rx_queue_total=$((rx_queue_total + 16#$rx_hex))
      drop_total=$((drop_total + drops))
    done < <(awk 'NR > 1 {print $2, $5, $NF}' "$socket_file")
  done

  printf '%s,%s,%s' "$socket_count" "$rx_queue_total" "$drop_total"
}

write_metadata() {
  local metadata_file="${SESSION_DIR}/metadata.txt"

  {
    printf 'started_at=%s\n' "$(date --iso-8601=seconds)"
    printf 'hostname=%s\n' "$(hostname 2>/dev/null || printf 'unknown')"
    printf 'kernel=%s\n' "$(uname -srmo 2>/dev/null || printf 'unknown')"
    printf 'lidar_ip=%s\n' "$LIDAR_IP"
    printf 'interface=%s\n' "$INTERFACE"
    printf 'interval_sec=%s\n' "$INTERVAL_SEC"
    printf 'duration_sec=%s\n' "$DURATION_SEC"
    printf 'max_samples=%s\n' "$MAX_SAMPLES"
    printf 'lidar_point_port=%s\n' "$LIDAR_POINT_PORT"
    printf 'host_point_port=%s\n' "$HOST_POINT_PORT"
    printf 'pcap_filter=%s\n' "$PCAP_FILTER"
    printf 'pcap_enabled=%s\n' "$ENABLE_PCAP"
    printf 'kernel_log_enabled=%s\n' "$ENABLE_KERNEL_LOG"
    printf 'effective_user=%s\n' "$(id -un 2>/dev/null || printf 'unknown')"
    printf 'command='
    printf '%q ' "$0" "${ORIGINAL_ARGS[@]}"
    printf '\n'
  } >"$metadata_file"
}

capture_snapshot() {
  local phase="$1"

  capture_command "${SESSION_DIR}/ip_link_${phase}.txt" ip -s -s link show dev "$INTERFACE"
  capture_command "${SESSION_DIR}/ip_addr_${phase}.txt" ip address show dev "$INTERFACE"
  capture_command "${SESSION_DIR}/route_${phase}.txt" ip route get "$LIDAR_IP"
  capture_command "${SESSION_DIR}/ethtool_${phase}.txt" ethtool "$INTERFACE"
  capture_command "${SESSION_DIR}/ethtool_stats_${phase}.txt" ethtool -S "$INTERFACE"
  capture_command "${SESSION_DIR}/nstat_${phase}.txt" nstat -az
  capture_command "${SESSION_DIR}/udp_sockets_${phase}.txt" ss -u -a -n -m
  if [[ -r /proc/net/softnet_stat ]]; then
    cp /proc/net/softnet_stat "${SESSION_DIR}/softnet_${phase}.txt"
  else
    printf 'UNAVAILABLE: /proc/net/softnet_stat\n' >"${SESSION_DIR}/softnet_${phase}.txt"
  fi
}

start_kernel_log() {
  ((ENABLE_KERNEL_LOG)) || return 0
  if ! command -v journalctl >/dev/null 2>&1; then
    printf 'UNAVAILABLE: journalctl\n' >"${SESSION_DIR}/kernel.log"
    log WARN "journalctl unavailable; kernel events will not be recorded"
    return 0
  fi

  journalctl --dmesg --follow --since now --output short-iso \
    >"${SESSION_DIR}/kernel.log" 2>&1 &
  JOURNAL_PID=$!
  log INFO "kernel journal follower started pid=$JOURNAL_PID"
}

start_pcap() {
  ((ENABLE_PCAP)) || return 0

  tcpdump -i "$INTERFACE" -nn -B 16384 -s 96 \
    -w "${SESSION_DIR}/mid360_headers.pcap" \
    "$PCAP_FILTER" \
    >"${SESSION_DIR}/tcpdump.log" 2>&1 &
  PCAP_PID=$!

  sleep 0.1
  if ! kill -0 "$PCAP_PID" 2>/dev/null; then
    wait "$PCAP_PID" || true
    PCAP_PID=""
    log ERROR "tcpdump failed to start; see ${SESSION_DIR}/tcpdump.log"
    return 1
  fi
  log INFO "header-only packet capture started pid=$PCAP_PID"
}

stop_background_process() {
  local pid="$1"
  local name="$2"

  [[ -n "$pid" ]] || return 0
  if kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
  log INFO "$name stopped pid=$pid"
}

fix_output_owner() {
  if [[ -n "${SUDO_USER:-}" ]] && command -v chown >/dev/null 2>&1 &&
    id "$SUDO_USER" >/dev/null 2>&1; then
    chown -R "$SUDO_USER":"$(id -gn "$SUDO_USER")" "$SESSION_DIR" 2>/dev/null || true
  fi
}

cleanup() {
  local exit_status=$?
  ((CLEANED_UP == 0)) || return "$exit_status"
  CLEANED_UP=1

  stop_background_process "$PCAP_PID" tcpdump
  stop_background_process "$JOURNAL_PID" journalctl
  capture_snapshot end
  printf 'finished_at=%s\n' "$(date --iso-8601=seconds)" >>"${SESSION_DIR}/metadata.txt"
  printf 'samples_recorded=%s\n' "$SAMPLE_COUNT" >>"${SESSION_DIR}/metadata.txt"
  log INFO "monitor stopped samples=$SAMPLE_COUNT exit_status=$exit_status"
  fix_output_owner
  return "$exit_status"
}

handle_signal() {
  STOP_REQUESTED=1
  log INFO "stop signal received"
}

collect_sample() {
  local timestamp_epoch timestamp_iso
  local ethtool_stats nstat_output softnet_totals socket_totals
  local rx_crc_errors rx_missed_errors rx_no_buffer link_down_events
  local udp_in_errors udp_rcvbuf_errors ip_in_discards

  timestamp_epoch="$(date '+%s.%N')"
  # Keep this field comma-free regardless of the active locale so the CSV
  # column layout remains stable. timestamp_epoch retains nanosecond precision.
  timestamp_iso="$(date --iso-8601=seconds)"
  ethtool_stats="$(read_ethtool_stats)"
  nstat_output="$(read_nstat_output)"
  softnet_totals="$(read_softnet_totals)"
  socket_totals="$(read_livox_socket_totals)"

  rx_crc_errors="$(extract_named_counter "$ethtool_stats" rx_crc_errors rx_crc_error rx_crc_err)"
  rx_missed_errors="$(extract_named_counter "$ethtool_stats" rx_missed_errors rx_missed)"
  rx_no_buffer="$(extract_named_counter "$ethtool_stats" rx_no_buffer_count rx_no_buffer rx_buf_alloc_fail)"
  link_down_events="$(extract_named_counter "$ethtool_stats" link_down_events link_down_event)"
  udp_in_errors="$(extract_nstat_counter "$nstat_output" UdpInErrors)"
  udp_rcvbuf_errors="$(extract_nstat_counter "$nstat_output" UdpRcvbufErrors)"
  ip_in_discards="$(extract_nstat_counter "$nstat_output" IpInDiscards)"

  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "$timestamp_epoch" \
    "$timestamp_iso" \
    "$(read_sysfs_value carrier)" \
    "$(read_sysfs_value operstate)" \
    "$(read_sysfs_counter rx_bytes)" \
    "$(read_sysfs_counter rx_packets)" \
    "$(read_sysfs_counter rx_errors)" \
    "$(read_sysfs_counter rx_dropped)" \
    "$(read_sysfs_counter tx_bytes)" \
    "$(read_sysfs_counter tx_packets)" \
    "$(read_sysfs_counter tx_errors)" \
    "$(read_sysfs_counter tx_dropped)" \
    "$rx_crc_errors" \
    "$rx_missed_errors" \
    "$rx_no_buffer" \
    "$link_down_events" \
    "$udp_in_errors" \
    "$udp_rcvbuf_errors" \
    "$ip_in_discards" \
    "${softnet_totals%%,*}" \
    "${softnet_totals#*,}" \
    "${socket_totals%%,*}" \
    "$(cut -d, -f2 <<<"$socket_totals")" \
    "${socket_totals##*,}" >>"$METRICS_FILE"
}

printf '%s\n' \
  'timestamp_epoch,timestamp_iso,carrier,operstate,rx_bytes,rx_packets,rx_errors,rx_dropped,tx_bytes,tx_packets,tx_errors,tx_dropped,rx_crc_errors,rx_missed_errors,rx_no_buffer_count,link_down_events,udp_in_errors,udp_rcvbuf_errors,ip_in_discards,softnet_dropped,softnet_time_squeeze,livox_socket_count,livox_rx_queue_bytes,livox_socket_drops' \
  >"$METRICS_FILE"

write_metadata
capture_snapshot start
trap cleanup EXIT
trap handle_signal INT TERM

log INFO "monitor started lidar=$LIDAR_IP interface=$INTERFACE interval=${INTERVAL_SEC}s"
echo "Session directory: $SESSION_DIR"
start_kernel_log
start_pcap

start_seconds=$SECONDS
while ((STOP_REQUESTED == 0)); do
  collect_sample
  SAMPLE_COUNT=$((SAMPLE_COUNT + 1))

  if ((MAX_SAMPLES > 0 && SAMPLE_COUNT >= MAX_SAMPLES)); then
    break
  fi
  if ((DURATION_SEC > 0 && SECONDS - start_seconds >= DURATION_SEC)); then
    break
  fi
  sleep "$INTERVAL_SEC" &
  wait $! || true
done
