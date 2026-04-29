#!/bin/bash

cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/AvrGcc.cmake
cmake --build build --target ToolIndexer_ENGLISH
