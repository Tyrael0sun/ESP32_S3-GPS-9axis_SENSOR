#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
IDF_EXPORT_SCRIPT=${1:-"$HOME/esp/esp-idf/export.sh"}

if [ ! -f "$IDF_EXPORT_SCRIPT" ]; then
    echo "ESP-IDF export script not found at: $IDF_EXPORT_SCRIPT" >&2
    echo "Pass the path as the first argument, e.g. ./scripts/setup_env.sh /opt/esp/idf/export.sh" >&2
    exit 1
fi

# Load ESP-IDF environment (exports PATH, IDF_PATH, etc.)
# shellcheck source=/dev/null
. "$IDF_EXPORT_SCRIPT"

idf.py -C "$REPO_ROOT" set-target esp32s3

cat <<'EOF'
Environment ready.
Next steps (run from repository root):
  idf.py build
  idf.py -p /dev/ttyUSB0 flash monitor  # replace with your serial port
EOF
