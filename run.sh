#!/usr/bin/env bash

if [ ! -d build ]; then
    sudo mkdir build
fi

while getopts "s"r: opt; do
    case "$opt" in
        s)
            # Build and make the project
            echo "Building and making project" >&2
            cd build &&
            cmake .. &&
            make
        ;;
        r)
            # Build, make and run project experiment
            echo "Setting up to run $OPTARG experiment" >&2
            cd build &&
            cmake .. &&
            make &&
            echo "Running $OPTARG experiment" >&2 &&
            cd .. &&
            argos3 -c experiments/$OPTARG.argos
        ;;
    esac
done