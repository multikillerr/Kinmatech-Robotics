#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
REQ_FILE="$ROOT_DIR/requirements.txt"
APP_ENTRY="$ROOT_DIR/ui/main.py"

if [[ ! -x "$PYTHON_BIN" ]]; then
  if command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="$(command -v python3)"
  else
    echo "Error: python3 not found." >&2
    exit 1
  fi
fi

if [[ ! -f "$REQ_FILE" ]]; then
  echo "Error: requirements file not found at $REQ_FILE" >&2
  exit 1
fi

if [[ ! -f "$APP_ENTRY" ]]; then
  echo "Error: app entry not found at $APP_ENTRY" >&2
  exit 1
fi

echo "Using Python: $PYTHON_BIN"
echo "Installing dependencies (user site, no virtual environment)..."
"$PYTHON_BIN" -m pip install --user -r "$REQ_FILE"

echo "Starting app..."
cd "$ROOT_DIR"
exec "$PYTHON_BIN" "$APP_ENTRY"
