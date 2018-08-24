#!/usr/bin/env bash

PROJDIR=${PWD}
EXPFILE="main"
ALGOS=()
TARGETNUMS=()
TARGETTHRESHS=()
CAMPOS=('-4.59862,0,4.59862' '-1.22369,0,9.18125' '-0.0457076,-5.0651,5.79638' '-0.205796,-10.9602,1.62526')
CAMLOOK=('-3.80581,0,3.98914' '-1.1384,-0.000852933,8.1849' '-0.0353751,-4.56833,4.92856' '-0.195418,-9.99895,1.90085')
CAMFOCAL=('20' '20' '20' '20')
VIZ=0
DRONENUM=4
SEEDCOUNT=0
CSVPREFIX="data"

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

    if [ $? != 0 ]
    then
        echo "Build/make process failed!"
        exit 1
    else
        echo "Done!"
    fi
}

function run_experiment {
    # Build, make and run project experiment
    echo "Running ${EXPFILE} experiment..."
    cd ${PROJDIR} &&
    argos3 -l exp_info.log -e exp_err.log -c experiments/tmp.argos &&
    rm experiments/tmp.argos &&
    echo "Done!"
}

function usage {
    echo "$0 usage:" && grep "[[:space:]].)\ #" $0 | sed 's/#//' | sed -r 's/([a-z])\)/-\1/'; exit 0;
}

while getopts a:bd:e:hIjn:N:s:t:v opt; do
    case "$opt" in
        a) # Select path planning algorithm/strategy (pso, aco or lawn).
            ALGOS=(${OPTARG})
        ;;
        b) # Build the main argos project. Use after editing source files.
            build_project
            exit 0
        ;;
        d) # Set the number of drones to place in simulation.
            DRONENUM=${OPTARG}
        ;;
        e) # Set experiment source file. Currently defaults to "main".
            EXPFILE=${OPTARG}
        ;;
        I) # Run installation of environment packages and dependancies.
            sudo apt install xmlstarlet &&
            sudo apt install python3-tk &&
            pip install --user virtualenv &&
            virtualenv env &&
            source env/bin/activate &&
            env/bin/pip3.5 install numpy scipy matplotlib ipython jupyter pandas sympy nose &&
            exit 0
        ;;
        j) # Run the jupyter environment.
            if [ -f env/bin/jupyter ]
            then
                sudo env/bin/jupyter notebook Statistics.ipynb
            else
                echo "Create experiment environment with the I) option."
                exit 1
            fi
        ;;
        n) # Set number of targets/plants to place in simulation.
            TARGETNUMS=(${OPTARG})
        ;;
        N) # Set value range of targets/plants to place in simulation.
            TARGETNUMS=$( seq ${OPTARG} )
        ;;
        s) # Set the number of independantly seeded trials to run.
            SEEDCOUNT=$OPTARG
        ;;
        t) # Set the target coverage/inspection percentage during trial.
            TARGETTHRESHS=(${OPTARG})
        ;;
        v) # Enable argos vizualization. Disabled by default for speed.
            VIZ=1
        ;;
        h | *) # Print this usage info.
            usage
        ;;
    esac
done

asize=${#ALGOS[*]}
tsize=${#TARGETNUMS[*]}
threshsize=${#TARGETTHRESHS[*]}
camsize=${#CAMPOS[*]}

if [ "${asize}" == 0 ]
then
    echo "No algorithms passed. Exiting!"
    exit 3
else
    for algo in ${ALGOS[*]}
    do
        if [ "${algo}" != "pso" -a "${algo}" != "aco" -a "${algo}" != "lawn" ]
        then
            echo "Invalid algorithm passed: only pso, aco or lawn. Exiting!"
            exit 4
        fi
    done
fi

if [ "${SEEDCOUNT}" == 0 ]
then
    echo "Seed counts not set. Exiting!"
    exit 5
fi

if [ "${tsize}" == 0 ]
then
    echo "No target numbers passed. Exiting!"
    exit 6
fi

if [ "${threshsize}" == 0 ]
then
    echo "No target thresholds passed. Exiting!"
    exit 7
else
    for thresh in ${TARGETTHRESHS[*]}
    do
        if (( $(echo "${thresh} > 1.0" | bc -l) == 1 ))
        then
            echo "Invalid threshold value passed. Exiting!"
            exit 8
        fi
    done
fi

build_project
cd ${PROJDIR}

trial=0

if [ ! -d "output" ]; then
    sudo mkdir output
    echo "Created output directory."
fi
data="output/${CSVPREFIX}_${trial}.csv"

while [ -f ${data} ]
do
    ((trial++))
    data="output/${CSVPREFIX}_${trial}.csv"
done

for a in ${ALGOS[*]}
do
    echo -n "Beginning ${a} trial with "

    for n in ${TARGETNUMS[*]}
    do
        echo -n "( targets = ${n} )"

        for t in ${TARGETTHRESHS[*]}
        do
            for s in $( seq 1 ${SEEDCOUNT} )
            do
                seed="$(openssl rand 4 | od -DAn)"
                seed="${seed#"${seed%%[![:space:]]*}"}"

                echo "( threshold = ${t} )( seed = ${seed} )"

                trial=0
                profile="output/profile_${trial}_${a}_${n}_${seed}.log"

                while [ -f "${profile}" ]
                do
                    ((trial++))
                    profile="output/profile_${trial}_${a}_${n}_${seed}.log"
                done

                cp experiments/${EXPFILE}.argos experiments/tmp.argos &&

                xmlstarlet ed -L -u 'argos-configuration/framework/experiment/@random_seed' -v "${seed}" experiments/tmp.argos &&
                xmlstarlet ed -L -u 'argos-configuration/framework/profiling/@file' -v "${profile}" experiments/tmp.argos &&
                xmlstarlet ed -L -u 'argos-configuration/controllers/main_controller/params/experiment/@name' -v "${a}" experiments/tmp.argos &&
                xmlstarlet ed -L -u 'argos-configuration/controllers/main_controller/params/experiment/@target' -v "${t}" experiments/tmp.argos &&
                xmlstarlet ed -L -u 'argos-configuration/controllers/main_controller/params/experiment/@csv' -v "${data}" experiments/tmp.argos &&
                xmlstarlet ed -L -u 'argos-configuration/arena/distribute[1]/entity/@quantity' -v "${n}" experiments/tmp.argos &&
                xmlstarlet ed -L -u 'argos-configuration/arena/distribute[2]/entity/@quantity' -v "${DRONENUM}" experiments/tmp.argos &&

                xmlstarlet ed -L -d 'argos-configuration/visualization' experiments/tmp.argos &&

                if [ "${VIZ}" == 0 ]
                then
                    xmlstarlet ed -L -s 'argos-configuration' -t elem -n visualization -v '' experiments/tmp.argos
                else
                    xmlstarlet ed -L -s 'argos-configuration' -t elem -n visualization -v '' experiments/tmp.argos &&
                    xmlstarlet ed -L -s 'argos-configuration/visualization' -t elem -n qt-opengl -v '' experiments/tmp.argos &&
                    xmlstarlet ed -L -s 'argos-configuration/visualization/qt-opengl' -t elem -n camera -v '' experiments/tmp.argos &&

                    for (( c=0; c<camsize; c++ ))
                    do
                        xmlstarlet ed -L -s 'argos-configuration/visualization/qt-opengl/camera' -t elem -n placement -v '' experiments/tmp.argos &&
                        xmlstarlet ed -L -i "//placement[${c}+1]" -t attr -n idx -v ${c} experiments/tmp.argos &&
                        xmlstarlet ed -L -i "//placement[${c}+1]" -t attr -n position -v ${CAMPOS[${c}]} experiments/tmp.argos &&
                        xmlstarlet ed -L -i "//placement[${c}+1]" -t attr -n look_at -v ${CAMLOOK[${c}]} experiments/tmp.argos &&
                        xmlstarlet ed -L -i "//placement[${c}+1]" -t attr -n lens_focal_length -v ${CAMFOCAL[${c}]} experiments/tmp.argos
                    done
                fi
                run_experiment
            done
        done
    done
done
