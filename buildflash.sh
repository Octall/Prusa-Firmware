#!/bin/bash
set -e
cd "$(dirname "$0")"

# Download/verify all build dependencies (avr-gcc 7.3.0, prusa3dboards, etc.)
# Idempotent — does nothing if deps are already present.
python3 utils/bootstrap.py

# Use the bootstrap-managed locked toolchain if present; fall back to system avr-gcc.
TOOLCHAIN_DIR=$(python3 utils/bootstrap.py --print-dependency-directory avr-gcc 2>/dev/null)
if [ -d "$TOOLCHAIN_DIR" ]; then
    TOOLCHAIN_FILE=cmake/AvrGcc.cmake
else
    TOOLCHAIN_FILE=cmake/AnyAvrGcc.cmake
fi

# Remove stale CMake configure cache so the toolchain file is always applied correctly.
# Object files in build/ are preserved, so only changed sources are recompiled.
rm -rf build/CMakeCache.txt build/CMakeFiles

cmake -B build -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE"
cmake --build build --target ToolIndexer_ENGLISH

avrdude -p m2560 -c stk500v2 -P /dev/ttyACM0 -b 115200 -D   -U flash:w:build/ToolIndexer_ENGLISH.hex:i

