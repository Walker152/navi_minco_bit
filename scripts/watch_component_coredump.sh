#!/usr/bin/env bash
set -Eeuo pipefail

# 监控目标 component container 的崩溃报告，并自动导出元数据、core 与 GDB 调用栈。
# 支持 Ubuntu 默认的 Apport，以及安装了 systemd-coredump 的系统。

EXE_PATTERN="component_container_mt"
CMD_PATTERN="livox_pointlio_container"
OUTPUT_ROOT="/tmp/nav_core_watch"
BACKEND="auto"
APPORT_DIR="/var/crash"
FOLLOW_EXISTING=0
RUN_ONCE=0
POLL_INTERVAL_SEC=1

usage() {
  cat <<'HELP'
Usage:
  watch_component_coredump.sh [options]

Options:
  --exe PATTERN       匹配 executable，默认: component_container_mt
  --cmd PATTERN       匹配 command line，默认: livox_pointlio_container
                      传空字符串可关闭命令行过滤，例如: --cmd ""
  --output DIR        输出根目录，默认: /tmp/nav_core_watch
  --backend TYPE      auto、systemd 或 apport，默认: auto
  --apport-dir DIR    Apport 崩溃报告目录，默认: /var/crash
  --include-existing  启动时也处理已有的匹配崩溃报告
  --once              扫描一次后退出，主要用于检查配置
  -h, --help          显示帮助

Examples:
  sudo ./scripts/watch_component_coredump.sh

  sudo ./scripts/watch_component_coredump.sh \
    --backend apport \
    --exe component_container_mt \
    --cmd livox_pointlio_container \
    --output /tmp/nav_core_watch
HELP
}

require_option_value() {
  local option="$1"
  local value="${2-}"
  if [[ -z "$value" ]]; then
    echo "[ERROR] $option requires a non-empty value" >&2
    exit 2
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --exe)
      require_option_value "$1" "${2-}"
      EXE_PATTERN="$2"
      shift 2
      ;;
    --cmd)
      [[ $# -ge 2 ]] || {
        echo "[ERROR] --cmd requires a value (use --cmd \"\" to disable it)" >&2
        exit 2
      }
      CMD_PATTERN="$2"
      shift 2
      ;;
    --output)
      require_option_value "$1" "${2-}"
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    --backend)
      require_option_value "$1" "${2-}"
      BACKEND="$2"
      shift 2
      ;;
    --apport-dir)
      require_option_value "$1" "${2-}"
      APPORT_DIR="$2"
      shift 2
      ;;
    --include-existing)
      FOLLOW_EXISTING=1
      shift
      ;;
    --once)
      RUN_ONCE=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "$BACKEND" in
  auto)
    if command -v coredumpctl >/dev/null 2>&1; then
      BACKEND="systemd"
    elif command -v apport-unpack >/dev/null 2>&1; then
      BACKEND="apport"
    else
      echo "[ERROR] Neither coredumpctl nor apport-unpack is available." >&2
      exit 1
    fi
    ;;
  systemd|apport)
    ;;
  *)
    echo "[ERROR] Unsupported backend: $BACKEND (expected auto, systemd, or apport)" >&2
    exit 2
    ;;
esac

required_commands=(python3)
if [[ "$BACKEND" == "systemd" ]]; then
  required_commands+=(journalctl coredumpctl)
else
  required_commands+=(apport-unpack find stat)
fi

for command_name in "${required_commands[@]}"; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "[ERROR] Missing required command for $BACKEND backend: $command_name" >&2
    exit 1
  fi
done

mkdir -p "$OUTPUT_ROOT"
WATCH_LOG="${OUTPUT_ROOT}/watcher.log"
touch "$WATCH_LOG"

log() {
  local level="$1"
  shift
  printf '%s [%s] %s\n' "$(date --iso-8601=seconds)" "$level" "$*" | tee -a "$WATCH_LOG"
}

fix_owner() {
  local target="$1"
  if [[ -n "${SUDO_USER:-}" ]] && id "$SUDO_USER" >/dev/null 2>&1; then
    chown -R "$SUDO_USER":"$(id -gn "$SUDO_USER")" "$target" 2>/dev/null || true
  fi
}

sanitize() {
  tr -cs '[:alnum:]_.-' '_' <<<"$1" | sed 's/^_*//; s/_*$//'
}

new_report_dir() {
  local exe="$1"
  local pid="$2"
  local timestamp exe_name report_dir suffix
  timestamp="$(date '+%Y%m%d-%H%M%S')"
  exe_name="$(sanitize "$(basename "${exe:-unknown}")")"
  [[ -n "$exe_name" ]] || exe_name="unknown"
  [[ "$pid" =~ ^[0-9]+$ ]] || pid="unknown"

  report_dir="${OUTPUT_ROOT}/${timestamp}_${exe_name}_pid${pid}"
  suffix=0
  while [[ -e "$report_dir" ]]; do
    suffix=$((suffix + 1))
    report_dir="${OUTPUT_ROOT}/${timestamp}_${exe_name}_pid${pid}_${suffix}"
  done
  mkdir -p "$report_dir"
  printf '%s\n' "$report_dir"
}

write_gdb_commands() {
  local path="$1"
  cat > "$path" <<'GDB'
set pagination off
set print pretty on
set print frame-arguments all
set logging enabled off
echo \n===== CORE SUMMARY =====\n
info program
info files
echo \n===== THREADS =====\n
info threads
echo \n===== ALL THREAD BACKTRACES =====\n
thread apply all bt full
echo \n===== SHARED LIBRARIES =====\n
info sharedlibrary
echo \n===== REGISTERS OF SELECTED THREAD =====\n
info registers
quit
GDB
}

capture_gdb_backtrace() {
  local exe="$1"
  local core_path="$2"
  local report_dir="$3"

  if [[ ! -s "$core_path" ]]; then
    log WARN "core_missing_or_empty path=$core_path"
    return 0
  fi
  if [[ -z "$exe" || ! -x "$exe" ]]; then
    log WARN "executable_unavailable path=${exe:-<empty>} core=$core_path"
    return 0
  fi
  if ! command -v gdb >/dev/null 2>&1; then
    log WARN "gdb_unavailable core_saved=$core_path"
    return 0
  fi

  local gdb_cmd="${report_dir}/gdb_commands.txt"
  local gdb_output="${report_dir}/gdb_backtrace.txt"
  write_gdb_commands "$gdb_cmd"
  log INFO "gdb_started exe=$exe core=$core_path"
  if ! gdb -q -batch -x "$gdb_cmd" "$exe" "$core_path" > "$gdb_output" 2>&1; then
    log WARN "gdb_nonzero output=$gdb_output"
  fi

  {
    echo "Target executable: $exe"
    echo "Core: $core_path"
    echo
    grep -E '^(Core was generated by|Program terminated with signal|Thread [0-9]+|#0 |#1 |#2 |#3 |#4 |#5 |#6 |#7 |#8 |#9 )' \
      "$gdb_output" || true
  } > "${report_dir}/gdb_backtrace_brief.txt"
  log INFO "gdb_complete output=$gdb_output"
}

process_systemd_event() {
  local event_json="$1"
  local parsed
  parsed="$(
    EVENT_JSON="$event_json" EXE_PATTERN="$EXE_PATTERN" CMD_PATTERN="$CMD_PATTERN" python3 - <<'PY'
import json
import os
import sys

try:
    data = json.loads(os.environ.get("EVENT_JSON", ""))
except Exception:
    sys.exit(10)

exe = str(data.get("COREDUMP_EXE", ""))
cmd = str(data.get("COREDUMP_CMDLINE", ""))
pid = str(data.get("COREDUMP_PID", data.get("_PID", "")))
signal = str(data.get("COREDUMP_SIGNAL_NAME", data.get("COREDUMP_SIGNAL", "")))
message = str(data.get("MESSAGE", ""))
timestamp = str(data.get("__REALTIME_TIMESTAMP", ""))

if os.environ.get("EXE_PATTERN", "") not in exe and os.environ.get("EXE_PATTERN", "") not in message:
    sys.exit(20)
cmd_pattern = os.environ.get("CMD_PATTERN", "")
if cmd_pattern and cmd_pattern not in cmd and cmd_pattern not in message:
    sys.exit(21)
if not pid.isdigit():
    sys.exit(22)
print("\t".join([pid, exe, cmd, signal, timestamp]))
PY
  )" || {
    local rc=$?
    if [[ $rc -ne 20 && $rc -ne 21 && $rc -ne 22 ]]; then
      log WARN "systemd_event_parse_failed rc=$rc"
    fi
    return 0
  }

  local pid exe cmd signal realtime_us report_dir core_path
  IFS=$'\t' read -r pid exe cmd signal realtime_us <<<"$parsed"
  report_dir="$(new_report_dir "$exe" "$pid")"
  core_path="${report_dir}/core"
  printf '%s\n' "$event_json" > "${report_dir}/journal_event.json"
  {
    echo "backend=systemd"
    echo "capture_time=$(date --iso-8601=seconds)"
    echo "pid=$pid"
    echo "exe=$exe"
    echo "cmdline=$cmd"
    echo "signal=$signal"
    echo "journal_realtime_us=$realtime_us"
    echo "hostname=$(hostname)"
    echo "kernel=$(uname -a)"
  } > "${report_dir}/summary.txt"
  log INFO "target_detected backend=systemd pid=$pid exe=$exe output=$report_dir"

  if ! coredumpctl --no-pager info "$pid" > "${report_dir}/coredumpctl_info.txt" 2>&1; then
    log WARN "coredumpctl_info_failed pid=$pid"
  fi
  if coredumpctl --no-pager dump "$pid" --output="$core_path" \
      > "${report_dir}/coredumpctl_dump.txt" 2>&1; then
    capture_gdb_backtrace "$exe" "$core_path" "$report_dir"
  else
    log WARN "coredumpctl_dump_failed pid=$pid output=${report_dir}/coredumpctl_dump.txt"
  fi
  fix_owner "$report_dir"
  log INFO "capture_complete backend=systemd pid=$pid output=$report_dir"
}

parse_apport_metadata() {
  local report_path="$1"
  REPORT_PATH="$report_path" python3 - <<'PY'
import os
import re
import sys

try:
    import problem_report
    report = problem_report.ProblemReport()
    with open(os.environ["REPORT_PATH"], "rb") as stream:
        report.load(stream, binary=False)
except Exception as exc:
    print(str(exc), file=sys.stderr)
    sys.exit(10)

def text(key):
    value = report.get(key, "")
    if isinstance(value, bytes):
        return value.decode("utf-8", "replace")
    return str(value)

status = text("ProcStatus")
pid_match = re.search(r"^Pid:\s*(\d+)\s*$", status, re.MULTILINE)
pid = text("Pid") or (pid_match.group(1) if pid_match else "")
values = [pid, text("ExecutablePath"), text("ProcCmdline"), text("Signal"), text("Date")]
print("\t".join(value.replace("\t", " ").replace("\n", " ") for value in values))
PY
}

process_apport_report() {
  local crash_file="$1"
  local parsed
  if ! parsed="$(parse_apport_metadata "$crash_file" 2>> "$WATCH_LOG")"; then
    log WARN "apport_report_parse_failed file=$crash_file"
    return 1
  fi

  local pid exe cmd signal crash_date
  IFS=$'\t' read -r pid exe cmd signal crash_date <<<"$parsed"
  if [[ -n "$EXE_PATTERN" && "$exe" != *"$EXE_PATTERN"* ]]; then
    return 0
  fi
  if [[ -n "$CMD_PATTERN" && "$cmd" != *"$CMD_PATTERN"* ]]; then
    return 0
  fi

  local report_dir unpack_dir core_path
  report_dir="$(new_report_dir "$exe" "$pid")"
  unpack_dir="${report_dir}/apport"
  core_path="${unpack_dir}/CoreDump"
  {
    echo "backend=apport"
    echo "capture_time=$(date --iso-8601=seconds)"
    echo "source_report=$crash_file"
    echo "crash_date=$crash_date"
    echo "pid=$pid"
    echo "exe=$exe"
    echo "cmdline=$cmd"
    echo "signal=$signal"
    echo "hostname=$(hostname)"
    echo "kernel=$(uname -a)"
  } > "${report_dir}/summary.txt"
  log INFO "target_detected backend=apport pid=${pid:-unknown} exe=$exe file=$crash_file output=$report_dir"

  if apport-unpack "$crash_file" "$unpack_dir" > "${report_dir}/apport_unpack.txt" 2>&1; then
    capture_gdb_backtrace "$exe" "$core_path" "$report_dir"
  else
    log WARN "apport_unpack_failed file=$crash_file output=${report_dir}/apport_unpack.txt"
  fi
  fix_owner "$report_dir"
  log INFO "capture_complete backend=apport pid=${pid:-unknown} output=$report_dir"
  return 0
}

declare -A SEEN_APPORT_REPORTS=()

apport_signature() {
  stat -c '%i:%s:%Y' "$1" 2>/dev/null
}

mark_existing_apport_reports() {
  local crash_file signature
  while IFS= read -r -d '' crash_file; do
    signature="$(apport_signature "$crash_file")" || continue
    SEEN_APPORT_REPORTS["$signature"]=1
  done < <(find "$APPORT_DIR" -maxdepth 1 -type f -name '*.crash' -print0 2>> "$WATCH_LOG")
}

scan_apport_reports() {
  local crash_file signature size_before size_after
  while IFS= read -r -d '' crash_file; do
    signature="$(apport_signature "$crash_file")" || continue
    [[ -z "${SEEN_APPORT_REPORTS[$signature]+x}" ]] || continue

    size_before="$(stat -c '%s' "$crash_file" 2>/dev/null)" || continue
    sleep 0.1
    size_after="$(stat -c '%s' "$crash_file" 2>/dev/null)" || continue
    if [[ "$size_before" != "$size_after" ]]; then
      continue
    fi

    if process_apport_report "$crash_file"; then
      SEEN_APPORT_REPORTS["$signature"]=1
    fi
  done < <(find "$APPORT_DIR" -maxdepth 1 -type f -name '*.crash' -print0 2>> "$WATCH_LOG")
}

log INFO "monitor_started backend=$BACKEND exe_pattern=${EXE_PATTERN:-<disabled>} cmd_pattern=${CMD_PATTERN:-<disabled>} output=$OUTPUT_ROOT"
if [[ "$BACKEND" == "apport" ]]; then
  log INFO "apport_directory=$APPORT_DIR include_existing=$FOLLOW_EXISTING"
else
  log INFO "systemd_journal include_existing=$FOLLOW_EXISTING"
fi

if [[ "$BACKEND" == "systemd" ]]; then
  journal_args=(--output=json --no-pager SYSLOG_IDENTIFIER=systemd-coredump)
  if [[ "$RUN_ONCE" -eq 0 ]]; then
    journal_args+=(--follow)
  fi
  if [[ "$FOLLOW_EXISTING" -eq 1 ]]; then
    journal_args+=(--lines=1)
  else
    journal_args+=(--lines=0)
  fi

  while IFS= read -r event_json; do
    [[ -n "$event_json" ]] || continue
    process_systemd_event "$event_json"
  done < <(journalctl "${journal_args[@]}")
else
  if [[ ! -d "$APPORT_DIR" ]]; then
    log ERROR "apport_directory_missing path=$APPORT_DIR"
    exit 1
  fi
  if [[ "$FOLLOW_EXISTING" -eq 0 ]]; then
    mark_existing_apport_reports
  fi

  while true; do
    scan_apport_reports
    [[ "$RUN_ONCE" -eq 0 ]] || break
    sleep "$POLL_INTERVAL_SEC"
  done
fi

log INFO "monitor_stopped backend=$BACKEND"
