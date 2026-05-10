#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

usage() {
  cat <<'USAGE'
Usage: ./deploy.sh [--env ENV] [--force-build] [--no-build] [--monitor|--no-monitor] [--monitor-baud BAUD] [--ip-timeout SECONDS]

Interactively build and flash this ESP32 PlatformIO project.

Options:
  -e, --env ENV       PlatformIO environment to build/upload.
  --force-build       Always build before scanning devices.
  --no-build          Do not build, even if firmware appears stale.
  --monitor           Open the ESP32 serial monitor after flashing. This is the default.
  --no-monitor        Do not open the ESP32 serial monitor after flashing.
  --monitor-baud BAUD Serial monitor baud rate. Defaults to platformio.ini.
  --ip-timeout SECONDS
                      Wait this long for ESP32APRS_STA_IP on serial after flashing. Default: 60.
  --no-ip-wait        Do not wait for or print the station IP.
  -h, --help          Show this help.

Environment:
  PIO                 Path to the PlatformIO executable.
USAGE
}

info() {
  printf '\033[1;34m==>\033[0m %s\n' "$*" >&2
}

warn() {
  printf '\033[1;33mwarning:\033[0m %s\n' "$*" >&2
}

die() {
  printf '\033[1;31merror:\033[0m %s\n' "$*" >&2
  exit 1
}

ask_yes_no() {
  local prompt="$1"
  local default="${2:-y}"
  local suffix
  local answer

  case "$default" in
    y|Y) suffix='[Y/n]' ;;
    n|N) suffix='[y/N]' ;;
    *) suffix='[y/n]' ;;
  esac

  while true; do
    printf '%s %s ' "$prompt" "$suffix" >&2
    read -r answer
    if [[ -z "$answer" ]]; then
      answer="$default"
    fi
    case "$answer" in
      y|Y|yes|YES) return 0 ;;
      n|N|no|NO) return 1 ;;
      *) printf 'Please answer y or n.\n' >&2 ;;
    esac
  done
}

find_pio() {
  if [[ -n "${PIO:-}" ]]; then
    [[ -x "$PIO" ]] || die "PIO is set but is not executable: $PIO"
    printf '%s\n' "$PIO"
    return
  fi

  if command -v pio >/dev/null 2>&1; then
    command -v pio
    return
  fi

  if [[ -x "$HOME/.platformio/penv/bin/pio" ]]; then
    printf '%s\n' "$HOME/.platformio/penv/bin/pio"
    return
  fi

  die "PlatformIO was not found. Install PlatformIO or set PIO=/path/to/pio."
}

find_python_with_serial() {
  local pio="$1"
  local pio_dir
  local candidate
  local resolved

  pio_dir="$(cd "$(dirname "$pio")" && pwd)"
  for candidate in "$pio_dir/python" "$pio_dir/python3" python3 python; do
    resolved=''
    if [[ "$candidate" == */* ]]; then
      [[ -x "$candidate" ]] && resolved="$candidate"
    else
      resolved="$(command -v "$candidate" 2>/dev/null || true)"
    fi

    if [[ -n "$resolved" ]] && "$resolved" -c 'import serial' >/dev/null 2>&1; then
      printf '%s\n' "$resolved"
      return 0
    fi
  done

  return 1
}

default_env() {
  awk '
    BEGIN { in_platformio = 0 }
    /^\[platformio\]/ { in_platformio = 1; next }
    /^\[/ { in_platformio = 0 }
    in_platformio && /^[[:space:]]*default_envs[[:space:]]*=/ {
      sub(/^[^=]*=/, "", $0)
      gsub(/[[:space:]]/, "", $0)
      split($0, envs, ",")
      print envs[1]
      exit
    }
  ' platformio.ini
}

list_envs() {
  awk '
    /^\[env:[^]]+\]/ {
      gsub(/^\[env:/, "")
      gsub(/\]$/, "")
      print
    }
  ' platformio.ini
}

monitor_speed() {
  local env="$1"

  awk -v want="[env:${env}]" '
    /^\[env\]/ {
      in_global = 1
      in_want = 0
      next
    }
    $0 == want {
      in_global = 0
      in_want = 1
      next
    }
    /^\[/ {
      in_global = 0
      in_want = 0
    }
    (in_global || in_want) && /^[[:space:]]*monitor_speed[[:space:]]*=/ {
      sub(/^[^=]*=/, "", $0)
      gsub(/[[:space:]]/, "", $0)
      if (in_want) {
        print
        found = 1
        exit
      }
      if (in_global) {
        global = $0
      }
    }
    END {
      if (!found && global != "") {
        print global
      }
    }
  ' platformio.ini
}

monitor_filters() {
  local env="$1"

  awk -v want="[env:${env}]" '
    /^\[env\]/ {
      in_global = 1
      in_want = 0
      next
    }
    $0 == want {
      in_global = 0
      in_want = 1
      next
    }
    /^\[/ {
      in_global = 0
      in_want = 0
    }
    (in_global || in_want) && /^[[:space:]]*monitor_filters[[:space:]]*=/ {
      sub(/^[^=]*=/, "", $0)
      gsub(/,/, " ", $0)
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", $0)
      if (in_want) {
        print
        found = 1
        exit
      }
      if (in_global) {
        global = $0
      }
    }
    END {
      if (!found && global != "") {
        print global
      }
    }
  ' platformio.ini
}

dotenv_value() {
  local key="$1"
  [[ -f .env ]] || return 1

  awk -v want="$key" '
    /^[[:space:]]*(#|$)/ { next }
    {
      line = $0
      pos = index(line, "=")
      if (pos == 0) {
        next
      }
      key = substr(line, 1, pos - 1)
      val = substr(line, pos + 1)
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", key)
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", val)
      if (key != want) {
        next
      }
      if (length(val) >= 2 && substr(val, 1, 1) == "\"" && substr(val, length(val), 1) == "\"") {
        val = substr(val, 2, length(val) - 2)
      }
      print val
      exit
    }
  ' .env
}

dotenv_first_value() {
  local value
  local key

  for key in "$@"; do
    value="$(dotenv_value "$key" || true)"
    if [[ -n "$value" ]]; then
      printf '%s\n' "$value"
      return 0
    fi
  done
  return 1
}

extract_ip_from_serial_line() {
  local line="$1"
  local ip=''

  ip="$(printf '%s\n' "$line" | sed -nE 's/.*ESP32APRS_STA_IP=([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+).*/\1/p' | head -n 1)"
  if [[ -z "$ip" ]]; then
    ip="$(printf '%s\n' "$line" | sed -nE 's/.*ESP32APRS_WEB_URL=http:\/\/([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+)\/.*/\1/p' | head -n 1)"
  fi

  [[ -n "$ip" ]] && printf '%s\n' "$ip"
}

wait_for_serial_ip() {
  local pio="$1"
  local port="$2"
  local baud="$3"
  local timeout="$4"
  local python=''
  local fifo
  local monitor_pid
  local started="$SECONDS"
  local chunk=''
  local serial_buffer=''
  local ip=''
  local monitor_args=(device monitor --port "$port" --baud "$baud")
  local MONITOR_FILTER_ARRAY=()
  local filter

  if python="$(find_python_with_serial "$pio" 2>/dev/null)"; then
    "$python" - "$port" "$baud" "$timeout" <<'PY'
import re
import sys
import time

import serial

port = sys.argv[1]
baud = int(sys.argv[2])
timeout = float(sys.argv[3])
patterns = (
    re.compile(r"ESP32APRS_STA_IP=([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+)"),
    re.compile(r"ESP32APRS_WEB_URL=http://([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+)/"),
)
deadline = time.monotonic() + timeout
buffer = ""

try:
    ser = serial.Serial(port, baud, timeout=0.25)
except Exception as exc:
    print(f"serial open failed: {exc}", file=sys.stderr)
    sys.exit(2)

try:
    while time.monotonic() < deadline:
        data = ser.read(512)
        if not data:
            continue
        buffer = (buffer + data.decode("utf-8", "ignore"))[-4096:]
        for pattern in patterns:
            match = pattern.search(buffer)
            if match:
                print(match.group(1))
                sys.exit(0)
finally:
    ser.close()

sys.exit(1)
PY
    return $?
  fi

  if [[ -n "$MONITOR_FILTERS" ]]; then
    read -r -a MONITOR_FILTER_ARRAY <<< "$MONITOR_FILTERS"
    for filter in "${MONITOR_FILTER_ARRAY[@]}"; do
      monitor_args+=(--filter "$filter")
    done
  fi

  fifo="$(mktemp -u "${TMPDIR:-/tmp}/esp32aprs-serial.XXXXXX")"
  mkfifo "$fifo"
  "$pio" "${monitor_args[@]}" > "$fifo" 2>&1 &
  monitor_pid=$!
  exec 3< "$fifo"
  rm -f "$fifo"

  while ((SECONDS - started < timeout)); do
    if IFS= read -r -t 1 -n 512 chunk <&3; then
      serial_buffer+="$chunk"
      if ((${#serial_buffer} > 4096)); then
        serial_buffer="${serial_buffer: -4096}"
      fi
      ip="$(extract_ip_from_serial_line "$serial_buffer")"
      if [[ -n "$ip" ]]; then
        kill "$monitor_pid" 2>/dev/null || true
        wait "$monitor_pid" 2>/dev/null || true
        exec 3<&-
        printf '%s\n' "$ip"
        return 0
      fi
    elif ! kill -0 "$monitor_pid" 2>/dev/null; then
      break
    fi
  done

  kill "$monitor_pid" 2>/dev/null || true
  wait "$monitor_pid" 2>/dev/null || true
  exec 3<&-
  return 1
}

choose_env() {
  local selected="$1"
  local fallback
  local answer

  fallback="$(default_env)"
  if [[ -z "$selected" ]]; then
    selected="$fallback"
  fi
  [[ -n "$selected" ]] || die "Could not infer a PlatformIO environment from platformio.ini."

  printf 'PlatformIO environment [%s]. Press Enter to use it, type ? to list, or enter another env: ' "$selected" >&2
  read -r answer
  case "$answer" in
    '') printf '%s\n' "$selected" ;;
    '?')
      info "Available environments:"
      list_envs | sed 's/^/  - /' >&2
      printf 'Environment [%s]: ' "$selected" >&2
      read -r answer
      if [[ -z "$answer" ]]; then
        printf '%s\n' "$selected"
      else
        printf '%s\n' "$answer"
      fi
      ;;
    *) printf '%s\n' "$answer" ;;
  esac
}

env_exists() {
  local env="$1"
  list_envs | grep -Fxq "$env"
}

any_newer_than() {
  local target="$1"
  shift

  [[ -e "$target" ]] || return 0
  find "$@" -type f -newer "$target" -print -quit 2>/dev/null | grep -q .
}

build_reason() {
  local env="$1"
  local firmware=".pio/build/$env/firmware.bin"
  local paths=()

  [[ -f "$firmware" ]] || {
    printf 'missing firmware: %s\n' "$firmware"
    return
  }

  [[ -d src ]] && paths+=(src)
  [[ -d include ]] && paths+=(include)
  [[ -d lib ]] && paths+=(lib)
  [[ -d lib_extra ]] && paths+=(lib_extra)
  [[ -f platformio.ini ]] && paths+=(platformio.ini)
  [[ -f flash_8MB.csv ]] && paths+=(flash_8MB.csv)
  [[ -f flash_16MB.csv ]] && paths+=(flash_16MB.csv)
  [[ -f flash_NOOTA.csv ]] && paths+=(flash_NOOTA.csv)

  if ((${#paths[@]} > 0)) && any_newer_than "$firmware" "${paths[@]}"; then
    printf 'source files are newer than %s\n' "$firmware"
  fi
}

filesystem_reason() {
  local env="$1"
  local image=''

  [[ -d data ]] || return

  for candidate in ".pio/build/$env/littlefs.bin" ".pio/build/$env/spiffs.bin"; do
    if [[ -f "$candidate" ]]; then
      image="$candidate"
      break
    fi
  done

  if [[ -z "$image" ]]; then
    printf 'no filesystem image found for data/\n'
    return
  fi

  if any_newer_than "$image" data; then
    printf 'data/ is newer than %s\n' "$image"
  fi
}

scan_ports() {
  local candidates=()
  local path
  local resolved
  local seen='|'

  shopt -s nullglob
  candidates+=(
    /dev/cu.usbserial*
    /dev/cu.usbmodem*
    /dev/cu.SLAB_USBtoUART*
    /dev/cu.wchusbserial*
    /dev/ttyUSB*
    /dev/ttyACM*
    /dev/serial/by-id/*
  )
  shopt -u nullglob

  ((${#candidates[@]} > 0)) || return 0

  for path in "${candidates[@]}"; do
    [[ -e "$path" ]] || continue
    resolved="$(cd "$(dirname "$path")" && pwd -P)/$(basename "$path")"
    if [[ "$seen" != *"|$resolved|"* ]]; then
      printf '%s\n' "$path"
      seen+="$resolved|"
    fi
  done
}

show_pio_devices() {
  local pio="$1"
  if "$pio" device list >/tmp/deploy-pio-devices.txt 2>/dev/null; then
    if [[ -s /tmp/deploy-pio-devices.txt ]]; then
      info "PlatformIO device list:"
      sed 's/^/  /' /tmp/deploy-pio-devices.txt >&2
    fi
  fi
  rm -f /tmp/deploy-pio-devices.txt
}

choose_port() {
  local pio="$1"
  local ports=()
  local answer
  local i

  while true; do
    ports=()
    while IFS= read -r port; do
      ports+=("$port")
    done < <(scan_ports)

    if ((${#ports[@]} == 0)); then
      warn "No obvious ESP32 serial devices were found."
      show_pio_devices "$pio"
      printf 'Enter an upload port manually, r to rescan, or q to quit: ' >&2
      read -r answer
      case "$answer" in
        r|R) continue ;;
        q|Q|'') exit 1 ;;
        *) printf '%s\n' "$answer"; return ;;
      esac
    fi

    info "Candidate flash devices:"
    for i in "${!ports[@]}"; do
      printf '  %2d) %s\n' "$((i + 1))" "${ports[$i]}" >&2
    done
    show_pio_devices "$pio"

    if ((${#ports[@]} == 1)); then
      if ask_yes_no "Use ${ports[0]}?" y; then
        printf '%s\n' "${ports[0]}"
        return
      fi
      printf 'Enter another upload port, r to rescan, or q to quit: ' >&2
      read -r answer
    else
      printf 'Select device number, enter a port path, r to rescan, or q to quit: ' >&2
      read -r answer
    fi

    case "$answer" in
      r|R) continue ;;
      q|Q|'') exit 1 ;;
      ''|*[!0-9]*)
        printf '%s\n' "$answer"
        return
        ;;
      *)
        if ((answer >= 1 && answer <= ${#ports[@]})); then
          printf '%s\n' "${ports[$((answer - 1))]}"
          return
        fi
        printf 'Invalid selection.\n' >&2
        ;;
    esac
  done
}

ENV_NAME=''
FORCE_BUILD=0
NO_BUILD=0
MONITOR_MODE='yes'
MONITOR_BAUD=''
MONITOR_FILTERS=''
IP_WAIT=1
IP_TIMEOUT=60

while (($#)); do
  case "$1" in
    -e|--env)
      shift
      [[ $# -gt 0 ]] || die "--env requires a value"
      ENV_NAME="$1"
      ;;
    --force-build)
      FORCE_BUILD=1
      ;;
    --no-build)
      NO_BUILD=1
      ;;
    --monitor)
      MONITOR_MODE='yes'
      ;;
    --no-monitor)
      MONITOR_MODE='no'
      ;;
    --monitor-baud)
      shift
      [[ $# -gt 0 ]] || die "--monitor-baud requires a value"
      MONITOR_BAUD="$1"
      ;;
    --ip-timeout)
      shift
      [[ $# -gt 0 ]] || die "--ip-timeout requires a value"
      [[ "$1" =~ ^[0-9]+$ ]] || die "--ip-timeout must be a number of seconds"
      IP_TIMEOUT="$1"
      ;;
    --no-ip-wait)
      IP_WAIT=0
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown argument: $1"
      ;;
  esac
  shift
done

PIO_BIN="$(find_pio)"
ENV_NAME="$(choose_env "$ENV_NAME")"
env_exists "$ENV_NAME" || die "PlatformIO environment not found: $ENV_NAME"
if [[ -z "$MONITOR_BAUD" ]]; then
  MONITOR_BAUD="$(monitor_speed "$ENV_NAME")"
fi
[[ -n "$MONITOR_BAUD" ]] || MONITOR_BAUD=115200
MONITOR_FILTERS="$(monitor_filters "$ENV_NAME")"

info "Using PlatformIO: $PIO_BIN"
info "Using environment: $ENV_NAME"

reason="$(build_reason "$ENV_NAME")"
if ((FORCE_BUILD)); then
  reason='forced by --force-build'
fi

if [[ -n "$reason" ]]; then
  info "Build needed: $reason"
  if ((NO_BUILD)); then
    warn "Skipping build because --no-build was set."
  elif ask_yes_no "Build firmware now?" y; then
    "$PIO_BIN" run -e "$ENV_NAME"
  else
    warn "Continuing without rebuilding firmware."
  fi
else
  info "Firmware appears up to date."
  if ((NO_BUILD == 0)) && ask_yes_no "Rebuild anyway?" n; then
    "$PIO_BIN" run -e "$ENV_NAME"
  fi
fi

fs_reason="$(filesystem_reason "$ENV_NAME")"
UPLOAD_FS=0
if [[ -n "$fs_reason" ]]; then
  info "Filesystem upload may be needed: $fs_reason"
  if ask_yes_no "Upload data/ filesystem after firmware?" y; then
    UPLOAD_FS=1
  fi
elif [[ -d data ]] && ask_yes_no "Upload data/ filesystem too?" n; then
  UPLOAD_FS=1
fi

PORT="$(choose_port "$PIO_BIN")"
[[ -n "$PORT" ]] || die "No upload port selected."

info "Flashing firmware to $PORT"
"$PIO_BIN" run -e "$ENV_NAME" -t upload --upload-port "$PORT"

if ((UPLOAD_FS)); then
  info "Uploading data/ filesystem to $PORT"
  "$PIO_BIN" run -e "$ENV_NAME" -t uploadfs --upload-port "$PORT"
fi

if ((IP_WAIT)); then
  info "Waiting up to ${IP_TIMEOUT}s for ESP32 station IP from serial"
  if ESP32_IP="$(wait_for_serial_ip "$PIO_BIN" "$PORT" "$MONITOR_BAUD" "$IP_TIMEOUT")"; then
    info "ESP32 station IP: $ESP32_IP"
    info "Web UI: http://${ESP32_IP}/"
  else
    warn "Could not read ESP32APRS_STA_IP from serial. It may still be joining Wi-Fi, or UART0 may be carrying non-debug data."
  fi
fi

OPEN_MONITOR=0
case "$MONITOR_MODE" in
  yes)
    OPEN_MONITOR=1
    ;;
  ask)
    if ask_yes_no "Open serial monitor on $PORT at ${MONITOR_BAUD} baud?" y; then
      OPEN_MONITOR=1
    fi
    ;;
esac

if ((OPEN_MONITOR)); then
  MONITOR_ARGS=(device monitor --port "$PORT" --baud "$MONITOR_BAUD")
  if [[ -n "$MONITOR_FILTERS" ]]; then
    read -r -a MONITOR_FILTER_ARRAY <<< "$MONITOR_FILTERS"
    for filter in "${MONITOR_FILTER_ARRAY[@]}"; do
      MONITOR_ARGS+=(--filter "$filter")
    done
    info "Using monitor filter(s): $MONITOR_FILTERS"
  fi

  info "Deploy complete. Starting serial monitor on $PORT at ${MONITOR_BAUD} baud."
  info "Press Ctrl+C to stop reading ESP32 debug output."
  sleep 2
  "$PIO_BIN" "${MONITOR_ARGS[@]}"
else
  info "Deploy complete."
fi
