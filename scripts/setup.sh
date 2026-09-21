#!/usr/bin/env bash
# One command from a fresh clone to a working environment.
#
#   scripts/setup.sh            # dev preset: fast loop, no heavy engines
#   scripts/setup.sh sim        # the physics twin and the whole app
#   scripts/setup.sh --deps     # install system packages too (asks first)
#
# What it does NOT do: install anything without telling you, or fetch what CMake
# already fetches for itself (MuJoCo, Ruckig, spdlog, json, httplib, CLI11, MCAP
# are all pinned in the build).
set -euo pipefail
cd "$(dirname "$0")/.."

preset="dev"
want_deps=0
for arg in "$@"; do
  case "$arg" in
    --deps) want_deps=1 ;;
    dev|sim|full|asan) preset="$arg" ;;
    -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
    *) echo "unknown argument: $arg (expected dev|sim|full|asan or --deps)"; exit 2 ;;
  esac
done

say() { printf '\n\033[1m%s\033[0m\n' "$1"; }

# --- system packages ---------------------------------------------------------
# Only OpenCV and a toolchain. Everything else the build fetches and pins.
missing=()
command -v cmake >/dev/null || missing+=("cmake")
command -v git >/dev/null || missing+=("git")
if ! pkg-config --exists opencv4 2>/dev/null && [ ! -d /opt/homebrew/opt/opencv ] \
   && [ ! -d /usr/include/opencv4 ]; then
  missing+=("opencv")
fi

if [ ${#missing[@]} -gt 0 ]; then
  say "Missing: ${missing[*]}"
  if [ "$want_deps" -eq 1 ]; then
    if command -v brew >/dev/null; then
      brew install cmake opencv
    elif command -v apt-get >/dev/null; then
      sudo apt-get update
      sudo apt-get install -y --no-install-recommends build-essential cmake git libopencv-dev
    else
      echo "No brew or apt here — install ${missing[*]} yourself, then re-run."; exit 1
    fi
  else
    echo "Install them with 'scripts/setup.sh --deps', or continue without:"
    echo "  OpenCV missing only costs you the robonode.opencv detector; the"
    echo "  build disables it and everything else works."
  fi
fi

# --- configure + build -------------------------------------------------------
say "Configuring preset '$preset'"
cmake --preset "$preset"

say "Building (first run with the sim preset builds MuJoCo from source — minutes)"
cmake --build --preset "$preset" -j

# --- what you can do now -----------------------------------------------------
say "Ready."
case "$preset" in
  dev)
    cat <<'EOF'
  ctest --preset fast                     # the sub-two-second loop
  scripts/setup.sh sim                    # when you need the physics twin
EOF
    ;;
  *)
    cat <<'EOF'
  ./build/apps/cell_server/cell_server     # → http://localhost:8080
  ./build/apps/robonode_cli/robonode_cli --help
  ctest --preset sim                       # everything, physics included
  bash scripts/verify.sh                   # the Definition of Done
EOF
    ;;
esac
