#!/bin/sh

# If you want GCC:
export CC=gcc CXX=g++

cmake --build "$@" -j
