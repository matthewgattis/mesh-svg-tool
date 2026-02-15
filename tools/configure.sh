#!/bin/sh

set -x

# If you want GCC:
export CC=gcc CXX=g++

# or, if you prefer Clang:
#export CC=clang CXX=clang++

cmake \
    -G "Unix Makefiles" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" \
    "$@"

