#!/usr/bin/env bash
set -eu
CNOID_ROOT=/home/mahdi/nextage/choreonoid-1.7.0-main/choreonoid-1.7.0-main
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
g++ -std=c++14 -fPIC -shared \
  -I"$CNOID_ROOT" -I"$CNOID_ROOT/include" -I/usr/include/eigen3 \
  -L"$CNOID_ROOT/lib" -Wl,-rpath,"$CNOID_ROOT/lib" \
  -o "$SCRIPT_DIR/RealModelBallController.so" \
  "$SCRIPT_DIR/RealModelBallController.cpp" \
  -l:libCnoidBody.so.1.7 -l:libCnoidUtil.so.1.7
