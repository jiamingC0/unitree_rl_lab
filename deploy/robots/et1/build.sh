#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<EOF
Usage:
  ./build.sh [x86|arm64] [--clean]

Examples:
  ./build.sh
  ./build.sh x86
  ./build.sh arm64
  ./build.sh arm64 --clean
EOF
}

target=""
clean=0

for arg in "$@"; do
  case "$arg" in
    x86|x86_64)
      target="x86-release"
      ;;
    arm64|aarch64|arm)
      target="arm64-release"
      ;;
    --clean|clean)
      clean=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ -z "$target" ]]; then
  machine="$(uname -m)"
  case "$machine" in
    x86_64|amd64)
      target="x86-release"
      ;;
    aarch64|arm64)
      target="arm64-release"
      ;;
    *)
      echo "Unsupported machine architecture: $machine" >&2
      echo "Please specify x86 or arm64 explicitly." >&2
      exit 1
      ;;
  esac
fi

build_dir="$SCRIPT_DIR/build/$target"

if [[ "$clean" -eq 1 ]]; then
  rm -rf "$build_dir"
fi

cmake --preset "$target" -S "$SCRIPT_DIR"
cmake --build --preset "$target" --parallel

echo "Built: $build_dir/et1_ctrl"
