#!/usr/bin/env bash

PROJDIR=${PWD}
EXPFILE="main"
ALGOS=()
EXPSEEDS=()
TARGETNUMS=()
TARGETTHRESHS=()
CAMPOS=('-4.59862,0,4.59862' '-1.22369,0,9.18125' '-0.0457076,-5.0651,5.79638' '-0.205796,-10.9602,1.62526')
CAMLOOK=('-3.80581,0,3.98914' '-1.1384,-0.000852933,8.1849' '-0.0353751,-4.56833,4.92856' '-0.195418,-9.99895,1.90085')
CAMFOCAL=('20' '20' '20' '20')
VIZ=0

function build_project {
    # Build and make the project
    echo "Building and making project..."

    if [ ! -d build ]; then
        echo "Creating build directory..."
        sudo mkdir build
    fi

    cd ${PROJDIR} &&
    cd build &&
    cmake .. &&
    make

    echo "Done!"
}

function run_experiment {
    # Build, make and run project experiment
    echo "Running ${EXPFILE} experiment..."
    cd ${PROJDIR} &&
    argos3 -l exp_info.log -e exp_err.log -c experiments/${EXPFILE}.argos
    echo "Done!"
}

while getopts a:be:n:s:t:V opt; do
    case "$opt" in
        a)
            ALGOS=(${OPTARG})
        ;;
        b)
            build_project
            exit 0
        ;;
        e)
            EXPFILE=${OPTARG}
        ;;
        n)
            TARGETNUMS=(${OPTARG})
        ;;
        s)
            EXPSEEDS=(${OPTARG})
        ;;
        t)
            TARGETTHRESHS=(${OPTARG})
        ;;
        V)
            VIZ=1
    esac
done

ssize=${#EXPSEEDS[*]}
asize=${#ALGOS[*]}
tsize=${#TARGETNUMS[*]}
threshsize=${#TARGETTHRESHS[*]}
camsize=${#CAMPOS[*]}

if [ ${asize} == 0 ]
then
    echo "No algorithms passed. Exiting!"
    exit 1
else
    for algo in ${ALGOS[*]}
    do
        if [ "${algo}" != "pso" -a "${algo}" != "aco" -a "${algo}" != "lawn" ]
        then
            echo "Invalid algorithm passed: only pso, aco or lawn. Exiting!"
            exit 2
        fi
    done
fi

if [ ${ssize} == 0 ]
then
    echo "No argos seeds passed. Exiting!"
    exit 3
fi

if [ ${tsize} == 0 ]
then
    echo "No target numbers passed. Exiting!"
    exit 4
fi

if [ ${threshsize} == 0 ]
then
    echo "No target thresholds passed. Exiting!"
    exit 6
else
    for thresh in ${TARGETTHRESHS[*]}
    do
        if (( $(echo "${thresh} > 1.0" | bc -l) == 1 ))
        then
            echo "Invalid threshold value passed. Exiting!"
            exit 5
        fi
    done
fi

build_project
cd ${PROJDIR}

for a in ${ALGOS[*]}
do
    echo -n "Beginning ${a} trial with "
    trial=0
    for s in ${EXPSEEDS[*]}
    do
        echo -n "( seed = ${s} )"
        trial=0
        for n in ${TARGETNUMS[*]}
        do
            echo -n "( targets = ${n} )"

            if [ ! -d "data/${n}" ]; then
                sudo mkdir data/${n}
                echo -n " ( created data target directory )"
            fi
            trial=0
            for t in ${TARGETTHRESHS[*]}
            do
                echo "( threshold = ${t} )"
                profile="profile_${a}_${trial}_${s}.log"

                xmlstarlet ed -L -u 'argos-configuration/framework/experiment/@random_seed' -v ${s} experiments/${EXPFILE}.argos &&
                xmlstarlet ed -L -u 'argos-configuration/framework/profiling/@file' -v data/${n}/${profile} experiments/${EXPFILE}.argos &&
                xmlstarlet ed -L -u 'argos-configuration/controllers/main_controller/params/experiment/@name' -v ${a} experiments/${EXPFILE}.argos &&
                xmlstarlet ed -L -u 'argos-configuration/controllers/main_controller/params/experiment/@target' -v ${t} experiments/${EXPFILE}.argos &&
                xmlstarlet ed -L -u 'argos-configuration/arena/distribute[1]/entity/@quantity' -v ${n} experiments/${EXPFILE}.argos

                xmlstarlet ed -L -d 'argos-configuration/visualization' experiments/${EXPFILE}.argos &&

                if [ ${VIZ} == 0 ]
                then
                    xmlstarlet ed -L -s 'argos-configuration' -t elem -n visualization -v '' experiments/${EXPFILE}.argos
                else
                    xmlstarlet ed -L -s 'argos-configuration' -t elem -n visualization -v '' experiments/${EXPFILE}.argos &&
                    xmlstarlet ed -L -s 'argos-configuration/visualization' -t elem -n qt-opengl -v '' experiments/${EXPFILE}.argos &&
                    xmlstarlet ed -L -s 'argos-configuration/visualization/qt-opengl' -t elem -n camera -v '' experiments/${EXPFILE}.argos &&

                    for (( c=0; c<camsize; c++ ))
                    do
                        xmlstarlet ed -L -s 'argos-configuration/visualization/qt-opengl/camera' -t elem -n placement -v '' experiments/${EXPFILE}.argos
                        xmlstarlet ed -L -i "//placement[${c}+1]" -t attr -n idx -v ${c} experiments/${EXPFILE}.argos &&
                        xmlstarlet ed -L -i "//placement[${c}+1]" -t attr -n position -v ${CAMPOS[${c}]} experiments/${EXPFILE}.argos &&
                        xmlstarlet ed -L -i "//placement[${c}+1]" -t attr -n look_at -v ${CAMLOOK[${c}]} experiments/${EXPFILE}.argos &&
                        xmlstarlet ed -L -i "//placement[${c}+1]" -t attr -n lens_focal_length -v ${CAMFOCAL[${c}]} experiments/${EXPFILE}.argos
                    done
                fi

                run_experiment
                ((trial++))
            done
        done
    done
done