#!/usr/bin/env bash

if [ ! -d build ]; then
    sudo mkdir build
fi

cd build &&
cmake .. &&
make