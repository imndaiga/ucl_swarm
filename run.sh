#!/usr/bin/env bash

if [ ! -d build ]; then
    sudo mkdir build
fi

while getopts se:t: opt; do
    case "$opt" in
        s)
            # Build and make the project
            echo "Building and making project" >&2
            cd build &&
            cmake .. &&
            make
        ;;
        e)
            # Build, make and run project experiment
            echo "Setting up to run $OPTARG experiment" >&2
            cd build &&
            cmake .. &&
            make &&
            echo "Running $OPTARG experiment" >&2 &&
            cd .. &&
            argos3 -c experiments/$OPTARG.argos
        ;;
        t)
            #Run multiple trials with passed seed values on main experiment
            echo "Starting experiments with seeds: $OPTARG" >&2

            seeds=(${OPTARG})
            algos=("pso" "aco" "lawn")
            targetnum=50
            targetthresh=0.85

            snum=${#seeds[*]}
            anum=${#algos[*]}

            for (( a=0; a<anum; a++ ))
            do
                echo "Beginning ${algos[a]} trial number ${a}..." >&2
                for (( s=0; s<snum; s++))
                do
                    profile="profile_${algos[a]}_${s}.log"
                    xmlstarlet ed -L -u 'argos-configuration/framework/experiment/@random_seed' -v ${seeds[s]} experiments/main.argos &&
                    xmlstarlet ed -L -u 'argos-configuration/framework/profiling/@file' -v ${profile} experiments/main.argos &&
                    xmlstarlet ed -L -u 'argos-configuration/controllers/main_controller/params/experiment/@name' -v ${algos[a]} experiments/main.argos &&
                    xmlstarlet ed -L -u 'argos-configuration/controllers/main_controller/params/experiment/@target' -v ${targetthresh} experiments/main.argos &&
                    xmlstarlet ed -L -u 'argos-configuration/arena/distribute[1]/entity/@quantity' -v ${targetnum} experiments/main.argos &&
                    argos3 -c experiments/main.argos
                done
            done
    esac
done