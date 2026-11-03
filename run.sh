#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
APP_PYTHON="$SCRIPT_DIR/.venv/bin/python"
if [[ ! -x "$APP_PYTHON" ]]; then
  echo "项目虚拟环境不存在：$APP_PYTHON。请先按 INSTALL_CN.md 创建 .venv 并安装依赖。" >&2
  exit 1
fi
exec "$APP_PYTHON" ./app.py "$@"
