#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<EOF
Usage:
  ./build.sh [x86|arm64] [--no-clean]

Examples:
  ./build.sh
  ./build.sh x86
  ./build.sh arm64
  ./build.sh arm64 --no-clean
EOF
}

target=""
clean=1

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
    --no-clean|no-clean)
      clean=0
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
      target="x86"
      ;;
    aarch64|arm64)
      target="arm64"
      ;;
    *)
      echo "Unsupported machine architecture: $machine" >&2
      echo "Please specify x86 or arm64 explicitly." >&2
      exit 1
      ;;
  esac
fi

case "$target" in
  x86-release|x86)
    target="x86-release"
    onnxruntime_root="$SCRIPT_DIR/../../thirdparty/onnxruntime-linux-x64-1.22.0"
    ;;
  arm64-release|arm64)
    target="arm64-release"
    onnxruntime_root="$SCRIPT_DIR/../../thirdparty/onnxruntime-linux-aarch64-1.26.0"
    ;;
  *)
    echo "Unsupported build target: $target" >&2
    exit 1
    ;;
esac

build_dir="$SCRIPT_DIR/build"

if [[ "$clean" -eq 1 ]]; then
  rm -rf "$build_dir"
fi

cmake \
  -S "$SCRIPT_DIR" \
  -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT="$onnxruntime_root"

cmake --build "$build_dir" --parallel

echo "Built: $build_dir/et1_ctrl"
