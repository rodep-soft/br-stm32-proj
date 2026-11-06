#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -d "${SCRIPT_DIR}/abu-bno085_cmake" ]; then
    TARGET_DIR="${SCRIPT_DIR}/abu-bno085_cmake"
else
    TARGET_DIR="${SCRIPT_DIR}"
fi

echo "==> [Fix CMake] Patching CMake files in ${TARGET_DIR} for CMake 3.28..."

find "${TARGET_DIR}" -path "${TARGET_DIR}/build" -prune -o \
    \( -name "CMakeLists.txt" -o -name "*.cmake" \) -type f -print0 | \
    xargs -0 sed -i \
        -e 's/cmake_minimum_required(VERSION 3\.30)/cmake_minimum_required(VERSION 3.28)/g' \
        -e 's/\$<QUOTE>/\\"/g'

# Remove backup files created by STM32CubeMX2 when overwriting patched CMake files
find "${TARGET_DIR}" -path "${TARGET_DIR}/build" -prune -o \
    \( -name "CMakeLists.txt_bckp" -o -name "*.cmake_bckp" \) -type f -exec rm -f {} +

echo "==> [Fix CMake] Done."
