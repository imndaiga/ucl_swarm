/* Include the controller definition */
#include "main.h"
/* Include the pso swarm algorithm definitions */
#include <algorithms/pso/swarm.h>
/* Include the aco swarm algorithm definitions */
#include <algorithms/aco/swarm.h>
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* Function definitions for logging */
#include <argos3/core/utility/logging/argos_log.h>
/* Definition of the argos space */
#include <argos3/core/simulator/space/space.h>
/* Definition of the argos simulator */
#include <argos3/core/simulator/simulator.h>
/* Definition of the argos entities and props */
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <argos3/core/simulator/entity/positional_entity.h>
#include <argos3/plugins/robots/eye-bot/simulator/eyebot_entity.h>
#include <argos3/core/simulator/simulator.h>
/* Include necessary standard library definitions */
#include <string>
#include <random>

/****************************************/
/****************************************/

CEyeBotMain::CEyeBotMain() :
    m_pcPosAct(NULL),
    m_pcPosSens(NULL),
    m_pcProximity(NULL),
    m_pcCamera(NULL),
    m_pcSpace(NULL),
    m_pcRABA(NULL),
    m_pcRABS(NULL),
    m_pcLEDs(NULL){}

std::random_device rd;

/****************************************/
/****************************************/

void CEyeBotMain::Init(TConfigurationNode& t_node) {

    m_pcPosAct       = GetActuator <CCI_QuadRotorPositionActuator             >("quadrotor_position"             );
    m_pcRABA         = GetActuator <CCI_RangeAndBearingActuator               >("range_and_bearing"              );
    m_pcPosSens      = GetSensor   <CCI_PositioningSensor                     >("positioning"                    );
    m_pcProximity    = GetSensor   <CCI_EyeBotProximitySensor                 >("eyebot_proximity"               );
    m_pcCamera       = GetSensor   <CCI_ColoredBlobPerspectiveCameraSensor    >("colored_blob_perspective_camera");
    m_pcRABS         = GetSensor   <CCI_RangeAndBearingSensor                 >("range_and_bearing"              );
    m_pcLEDs         = GetActuator <CCI_LEDsActuator                          >("leds"                           );
    m_pcSpace        = &CSimulator::GetInstance().GetSpace();
    kf               = new KalmanFilter(m_sKalmanFilter.dt, m_sKalmanFilter.A, m_sKalmanFilter.C, m_sKalmanFilter.Q, m_sKalmanFilter.R, m_sKalmanFilter.P);
    m_cNearestTarget = new CLightEntity;
    /*
    * Parse the config file
    */
    try {
        /* Get swarm parameters */
        m_sSwarmParams.Init(GetNode(t_node, "swarm"));
        /* Get mapping parameters */
        m_sExperimentParams.Init(GetNode(t_node, "experiment"));
        /* Get quadcopter launch parameters */
        m_sStateData.Init(GetNode(t_node, "state"));
        /* Get waypoint parameters */
        m_sRandGen.Init(GetNode(t_node, "random"));
        /* Get waypoint parameters */
        m_sLawnParams.Init(GetNode(t_node, "lawn"));
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

    /*
    * Initialize kalman filter.
    */
    UpdatePosition(m_pcPosSens->GetReading().Position);

    HomePos = GetPosition();

    /* Enable camera filtering */
    m_pcCamera->Enable();
    Reset();
}

void CEyeBotMain::ControlStep() {
    StepUpdate();

    int SIM_LIMIT_STATUS = RecordTrial();
    if(SIM_LIMIT_STATUS == EXIT_FAILURE) {
        exit(EXIT_FAILURE);
    }

    switch(m_sStateData.State) {
        case SStateData::STATE_START:
            // Initialize tasks and global map.
            InitializeDrones();
            InitializeGlobalMap();
            Rest();
            break;
        case SStateData::STATE_MOVE:
            Move();
            break;
        case SStateData::STATE_EXECUTE_TASK:
            ExecuteTask();
            break;
        case SStateData::STATE_REST:
            Rest();
            break;
        case SStateData::STATE_LAND:
            Land();
            break;
        default:
            LOGERR << "[BUG] Unknown robot state: " << m_sStateData.State << std::endl;
    }

    /* Write debug information */
    // RLOG << "Current state: " << m_sStateData.State << std::endl;
    // RLOG << "Trial count: " << trialCounter << std::endl;
    // RLOG << "Target pos: " << m_cTargetPos << std::endl;
    // RLOG << "Current pos: " << m_pcPosSens->GetReading().Position << std::endl;
    // RLOG << "Filtered pos: " << GetPosition() << std::endl;
    // RLOG << "Local map size: " << LocalMap.size() << std::endl;
    // RLOG << "Local index: " << m_sStateData.LocalIndex << std::endl;
    // RLOG << "Global map size: " << GlobalMap.size() << std::endl;
    // RLOG << "Holding time: " << m_sStateData.HoldTime << std::endl;
    // RLOG << "Minimum holding time: " << m_sStateData.minimum_hold_time << std::endl;
    // RLOG << "Resting time: " << m_sStateData.RestTime << std::endl;
    // RLOG << "RestToMove: " << m_sStateData.RestToMoveProb << std::endl;
    // RLOG << "RestToLand: " << m_sStateData.RestToLandProb << std::endl;
}

void CEyeBotMain::Reset() {
    /* Reset robot state */
    m_sRandGen.Set((size_t)m_pTargetMap.size() - 1);
    m_sStateData.Reset();
    GlobalMap.clear();
    LocalMap.clear();
    m_pcRABA->ClearData();
    fileCreated = false;
    fileCounter=0;
}

/****************************************/
/****************************************/

void CEyeBotMain::Land() {
    if(m_sStateData.State != SStateData::STATE_LAND) {
        /* State initialization */
        m_sStateData.State = SStateData::STATE_LAND;
        m_cTargetPos = HomePos;
        m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
    } else {
        Rest();
    }
}

void CEyeBotMain::Move() {
    if(m_sStateData.State != SStateData::STATE_MOVE) {
        /* State initialization */
        m_sStateData.State = SStateData::STATE_MOVE;
    } else {
        if(m_sStateData.LocalIndex < LocalMap.size()) {
            std::vector<double> target_wp = GetWaypoint();
            m_cTargetPos = CVector3(target_wp[0], target_wp[1], target_wp[2]);
            m_pcPosAct->SetAbsolutePosition(m_cTargetPos);

            if(Distance(m_cTargetPos, GetPosition()) < m_sStateData.proximity_tolerance) {
                /* State transition */
                ExecuteTask();
            }
        } else {
            /* State transition */
            // LocalMap is empty or in error, go to rest state
            // RLOG << "No waypoints available. Resting now." << std::endl;
            Rest();
        }
    }
}

void CEyeBotMain::ExecuteTask() {

    if(m_sStateData.State != SStateData::STATE_EXECUTE_TASK) {
        /* State initialization */
        m_sStateData.State = SStateData::STATE_EXECUTE_TASK;
    } else {
        /* State logic */
        // RLOG << "Executing task." << std::endl;
        (this->*TaskFunction)();
        /* State transition */
        Move();
    }
}

void CEyeBotMain::Rest() {
    if(m_sStateData.State != SStateData::STATE_REST) {
        /* State initialize */
        m_sStateData.State = SStateData::STATE_REST;
        // Reset Waypoint index
        m_sStateData.LocalIndex = 0;
    } else {
        /* if robot has stayed at current position long enough,
        *  probabilistically change to move state.
        */
        double RestToMoveCheck = m_sRandGen.resttomove.get();
        double RestToLandCheck = m_sRandGen.resttoland.get();

        // RLOG << "Rest check: " << RestToMoveCheck << std::endl;
        // RLOG << "Land check: " << RestToLandCheck << std::endl;

        if(m_sStateData.RestTime > m_sStateData.minimum_rest_time && 
           RestToMoveCheck < m_sStateData.RestToMoveProb) {
            // Replanning unordered waypoints and add to local map.
            // RLOG << "Replanning now." << std::endl;
            UpdateLocalMap();

            /* State transition */
            Move();
            m_sStateData.RestTime = 0;
        } else if (m_sStateData.RestTime > m_sStateData.minimum_rest_time &&
                   RestToLandCheck < m_sStateData.RestToLandProb) {
            // Land once inspection is probabilistically complete.
            // RLOG << "Completed inspections. Landing now." << std::endl;
            /* State transition */
            Land();
            m_sStateData.RestTime = 0;
        } else {
            m_sStateData.RestTime++;
        }
    }
}

/****************************************/
/****************************************/

void CEyeBotMain::InitializeGlobalMap(bool verbose) {
    std::vector< std::vector<double> > raw_waypoints;
    std::vector< std::vector<double> > target_locations;
    std::vector<int> global_tour;
    long int global_tour_length;

    CSpace::TMapPerType& tLightMap = m_pcSpace->GetEntitiesByType("light");
    CSpace::TMapPerType boxes = m_pcSpace->GetEntitiesByType("box");
    CBoxEntity* Wall = any_cast<CBoxEntity*>(boxes["wall_north"]);
    CVector3 WallSize = Wall->GetSize();

    if(m_sExperimentParams.naive_mapping) {
        if(strcmp(m_sExperimentParams.name, "lawn")) {
            /* Retrieve and store the positions of each light in the arena */
            for(CSpace::TMapPerType::iterator it = tLightMap.begin(); it != tLightMap.end(); ++it) {
                // cast the entity to a light entity
                CLightEntity& cLightEnt = *any_cast<CLightEntity*>(it->second);
                std::vector<double> l_vec;

                l_vec.push_back(cLightEnt.GetPosition().GetX());
                l_vec.push_back(cLightEnt.GetPosition().GetY());
                l_vec.push_back(cLightEnt.GetPosition().GetZ());
                target_locations.push_back(l_vec);
            }
        } else if(!strcmp(m_sExperimentParams.name, "lawn")) {
            std::vector< std::vector<double> > unsorted;

            double z_i = 0.0;
            int flipCount = 1;

            while(z_i < WallSize.GetZ()) {
                double x_i = WallSize.GetX()/2.0;

                while(x_i > -WallSize.GetX()/2.0) {
                    std::vector<double> wp;
                    wp.push_back(x_i);
                    wp.push_back(WallSize.GetY());
                    wp.push_back(z_i);

                    unsorted.push_back(wp);
                    x_i -= m_sLawnParams.hstep;
                }

                if(flipCount % 2 == 0) {
                    // Ensure that alternate Lawn levels are flipped to generate
                    // a lawn path motion.
                    std::reverse(unsorted.begin(), unsorted.end());
                }

                for(size_t i = 0; i < unsorted.size(); i++ ) {
                    // Store Lawn level as sorted.
                    target_locations.push_back( unsorted[i] );
                }

                z_i += m_sLawnParams.vstep;
                flipCount++;
                unsorted.clear();
            }
        }
    } else {
        /* Implement target seeking here. Would require SFM abilities? */
    }

    for(size_t p_i = 0; p_i < target_locations.size(); p_i++) {
        std::vector<double> modified_loc;
        // Simulate gaussian sensor noise for each axis reading
        // and include reach and attitude variables.
        modified_loc.push_back(target_locations[p_i][0] + m_sRandGen.mapping.get());
        modified_loc.push_back(target_locations[p_i][1] - m_sStateData.Reach + m_sRandGen.mapping.get());
        modified_loc.push_back(target_locations[p_i][2] + m_sStateData.attitude + m_sRandGen.mapping.get());

        raw_waypoints.push_back(modified_loc);
    }

    if(raw_waypoints.size() > 0 && strcmp(m_sExperimentParams.name, "lawn")) {

        if(!strcmp(m_sExperimentParams.name, "pso")) {
            PsoSwarm swarm(m_sSwarmParams.particles, m_sSwarmParams.self_trust, m_sSwarmParams.past_trust, m_sSwarmParams.global_trust, raw_waypoints, "cm");
            swarm.optimize(global_tour, global_tour_length);
        } else if(!strcmp(m_sExperimentParams.name, "aco")) {
            AcoSwarm swarm(m_sSwarmParams.ants, raw_waypoints, m_sRandGen.aco_seed, "cm");
            swarm.optimize(global_tour, global_tour_length);
        }

        for(size_t n=0; n < global_tour.size(); n++) {
            // Store to waypoints map
            GlobalMap[global_tour[n]] = std::make_tuple(raw_waypoints[global_tour[n]], SStateData::TASK_EVALUATE, CColor::WHITE);
        }

        if(verbose) {
            RLOG << "Global Tour Distance: " << global_tour_length << std::endl;
            RLOG << "Shortest Global Path: ";
            for(size_t n=0; n < global_tour.size(); n++) {
                LOG << global_tour[n] << " - (";
                for(std::vector<double>::iterator twp_rd = raw_waypoints[global_tour[n]].begin(); twp_rd != raw_waypoints[global_tour[n]].end(); ++twp_rd) {
                    LOG << *twp_rd << " ";
                }
                LOG << ")" << std::endl;
            }
            RLOG << "Global Waypoint Map: " << std::endl;
            for(auto& wp : GlobalMap) {
                LOG << "Index: " << wp.first << " Map Location: (";
                for(auto& rd: std::get<0>(wp.second)) {
                    LOG << rd << " ";
                }
                LOG << ")" << std::endl;
            }
        }
    } else if (raw_waypoints.size() > 0 && !strcmp(m_sExperimentParams.name, "lawn")) {
        for(size_t i = 0; i < raw_waypoints.size(); i++) {
            // Store sorted waypoint into WaypointMap.
            GlobalMap[i] = std::make_tuple(raw_waypoints[i], SStateData::TASK_EVALUATE, CColor::WHITE);
        }

        if(verbose) {
            RLOG << "Wall size: " << WallSize << std::endl;
            RLOG << "GlobalMap size: " << GlobalMap.size() << std::endl;
            RLOG << "Global Waypoint Map: " << std::endl;
            for(auto& wp : GlobalMap) {
                LOG << "Index: " << wp.first << " Map Location: (";
                for(auto& rd: std::get<0>(wp.second)) {
                    LOG << rd << " ";
                }
                LOG << ")" << std::endl;
            }
        }
    }
}

void CEyeBotMain::UpdateLocalMap(bool verbose) {
    LocalMap.clear();
    std::vector<size_t> global_index;
    std::vector< std::vector<double> > unsorted_waypoints;

    if(strcmp(m_sExperimentParams.name, "lawn")) {
        tour.clear();
        tour_length = 0;

        for(auto& wp : GlobalMap) {
            if(std::get<1>(wp.second) == m_sStateData.TaskState) {
                unsorted_waypoints.push_back(std::get<0>(wp.second));
                global_index.push_back(wp.first);
            }
        }

        if(unsorted_waypoints.size() > 0) {
            if(!strcmp(m_sExperimentParams.name, "pso")) {
                PsoSwarm swarm(m_sSwarmParams.particles, m_sSwarmParams.self_trust, m_sSwarmParams.past_trust, m_sSwarmParams.global_trust, unsorted_waypoints, "cm");
                swarm.optimize(tour, tour_length);
            } else if(!strcmp(m_sExperimentParams.name, "aco")) {
                AcoSwarm swarm(m_sSwarmParams.ants, unsorted_waypoints, m_sRandGen.aco_seed, "cm");
                swarm.optimize(tour, tour_length);
            }

            for(size_t n=0; n < tour.size(); n++) {
                // Store in local waypoints map
                LocalMap[tour[n]] = std::make_tuple(unsorted_waypoints[tour[n]], m_sStateData.TaskState, std::get<2>(GlobalMap[global_index[n]]));
            }
        }

        if(verbose) {
            RLOG << "Local Tour Distance: " << tour_length << std::endl;
            RLOG << "Shortest Local Path: ";
            for(size_t n=0; n < tour.size(); n++) {
                LOG << tour[n] << " - (";
                for(std::vector<double>::iterator twp_rd = unsorted_waypoints[tour[n]].begin(); twp_rd != unsorted_waypoints[tour[n]].end(); ++twp_rd) {
                    LOG << *twp_rd << " ";
                }
                LOG << ")" << std::endl;
            }
        }
    } else if(!strcmp(m_sExperimentParams.name, "lawn")) {
        for(auto& wp : GlobalMap) {
            unsorted_waypoints.push_back(std::get<0>(wp.second));
            global_index.push_back(wp.first);
        }

        // Update local map with all unmarked targets
        for(size_t wp_id = 0; wp_id < unsorted_waypoints.size(); wp_id++) {
            LocalMap[wp_id] = std::make_tuple(unsorted_waypoints[wp_id], m_sStateData.TaskState, std::get<2>( GlobalMap[global_index[wp_id]] ));
        }
    }

    if(verbose) {
        RLOG << "Local Waypoint Map: " << std::endl;
        for(auto& wp : LocalMap) {
            LOG << "Index: " << wp.first << " Map Location: (";
            for(auto& rd: std::get<0>(wp.second)) {
                LOG << rd << " ";
            }
            LOG << ")" << std::endl;
        }
    }
}

void CEyeBotMain::UpdatePosition(CVector3 x0) {
    Eigen::VectorXd x_i_hat(m_sKalmanFilter.m);

    if(!kf->initialized) {
        Eigen::VectorXd x_0(m_sKalmanFilter.n);
        x_0 << x0.GetX(), x0.GetY(), x0.GetZ();
        kf->init(m_sKalmanFilter.dt, x_0);
    } else {
        Eigen::VectorXd x_i(m_sKalmanFilter.m);
        CVector3 xi = m_pcPosSens->GetReading().Position;
        x_i << xi.GetX(), xi.GetY(), xi.GetZ();
        kf->update(x_i);
    }

    x_i_hat = kf->state();
    // LOG << x_i_hat << std::endl;
    m_sKalmanFilter.state = CVector3(x_i_hat[0], x_i_hat[1], x_i_hat[2]);
}

void CEyeBotMain::UpdateNearestTarget() {
    CSpace::TMapPerType& tLightMap = m_pcSpace->GetEntitiesByType("light");
    CLightEntity* cLightEnt;
    /* Retrieve and evaluate the positions of each light in the arena */
    for(CSpace::TMapPerType::iterator it = tLightMap.begin(); it != tLightMap.end(); ++it) {
        // cast the entity to a light entity
        cLightEnt = any_cast<CLightEntity*>(it->second);
        CVector3 compensated_waypoint = m_cTargetPos + CVector3(0.0, m_sStateData.Reach, -m_sStateData.attitude);

        if(m_cNearestTarget) {
            if(Distance(cLightEnt->GetPosition(), compensated_waypoint) < Distance(m_cNearestTarget->GetPosition(), compensated_waypoint)) {
                m_cNearestTarget = cLightEnt;
            }
        } else {
            m_cNearestTarget = cLightEnt;
        }
    }
}

void CEyeBotMain::StepUpdate() {
    if(!strcmp(m_sExperimentParams.name, "lawn")) {
        if(m_sStateData.LaunchTime == m_sStateData.TimeToLaunch && !m_sStateData.HasLaunched) {
            m_sStateData.RestToLandProb = m_sStateData.InitialRestToLandProb;
            m_sStateData.RestToMoveProb = m_sStateData.InitialRestToMoveProb;
            m_sStateData.LaunchTime = 0;
            m_sStateData.HasLaunched = true;
        } else {
            m_sStateData.LaunchTime++;
        }
    }
    UpdatePosition();
    UpdateNearestTarget();
    ListenToNeighbours();
}

void CEyeBotMain::InitializeDrones() {
    CSpace::TMapPerType& tEyeBotMap = m_pcSpace->GetEntitiesByType("eye-bot");
    CEyeBotEntity* cEyeBotEnt;
    int task_id = 0;
    int node_count = 0;
    int launch_accum = 0;

    /* Retrieve and assign tasks to each eye-bot */
    for(CSpace::TMapPerType::iterator it = tEyeBotMap.begin(); it != tEyeBotMap.end(); ++it, task_id++, node_count++) {
        // Cast the entity to a eye-bot entity
        cEyeBotEnt = any_cast<CEyeBotEntity*>(it->second);

        // Ensure green state isn't included in map selection with -2.
        if(task_id > m_pTargetMap.size() - 2) {
            // Reset index if greater than the number of tasks available
            task_id = 0;
        }

        CEyeBotMain& cController = dynamic_cast<CEyeBotMain&>(cEyeBotEnt->GetControllableEntity().GetController());
        // Set controller state TaskState.
        cController.m_sStateData.TaskState = std::get<1>(m_pTargetMap[task_id]);
        cController.m_sStateData.TaskColor = std::get<2>(m_pTargetMap[task_id]);
        cController.m_pcLEDs->SetAllColors(cController.m_sStateData.TaskColor);
        // Set controller state Reach variable.
        cController.m_sStateData.Reach = cController.m_sStateData.global_reach + std::get<3>(m_pTargetMap[task_id]);

        // Vary resting and landing probabilities if lawn experiment.
        if(!strcmp(cController.m_sExperimentParams.name, "lawn")) {
            cController.m_sStateData.TimeToLaunch = launch_accum;
            launch_accum += m_sSwarmParams.launch_step;
            cController.m_sStateData.RestToLandProb = 1;
            cController.m_sStateData.RestToMoveProb = 0;
        }

        // Set controller TaskFunction.
        if(cController.m_sStateData.TaskState == SStateData::TASK_EVALUATE) {
            cController.TaskFunction = &CEyeBotMain::EvaluateFunction;
        } else if(cController.m_sStateData.TaskState == SStateData::TASK_WATER) {
            cController.TaskFunction = &CEyeBotMain::WaterFunction;
        } else if(cController.m_sStateData.TaskState == SStateData::TASK_NOURISH) {
            cController.TaskFunction = &CEyeBotMain::NourishFunction;
        } else if(cController.m_sStateData.TaskState == SStateData::TASK_TREATMENT) {
            cController.TaskFunction = &CEyeBotMain::TreatmentFunction;
        }

        // Initialize one leader evaluation drone.
        if(node_count < (m_pTargetMap.size() - 2) && cController.m_sStateData.TaskState == SStateData::TASK_EVALUATE) {
            cController.m_sStateData.IsLeader = true;
        } else {
            cController.m_sStateData.IsLeader = false;
        }

        m_mTaskedEyeBots[cEyeBotEnt->GetId()] = cController.m_sStateData.TaskState;
    }

    drones_initialized = true;
    m_sStateData.HasLaunched = false;

    // LOG << "Tasked eyebot map: " << std::endl;
    // for (std::map<std::string, SStateData::ETask>::const_iterator iter = m_mTaskedEyeBots.begin(); iter != m_mTaskedEyeBots.end(); iter++)
    // {
        // LOG << "Robot Id: " << iter->first << " " << "Task:" << iter->second << std::endl;
    // }
}

void CEyeBotMain::ListenToNeighbours() {
    /*
    * Social rule: listen to what targets have been found.
    */

    if(drones_initialized && strcmp(m_sExperimentParams.name, "lawn")) {
        // RLOG << "Message received: ";
        UInt8 task_id, wp_id;

        if(! m_pcRABS->GetReadings().empty()) {
            m_pEBMsg = &(m_pcRABS->GetReadings()[0]);
            task_id = m_pEBMsg->Data[0];
            wp_id = m_pEBMsg->Data[1];

            // Process received message.
            SStateData::ETask Task = (SStateData::ETask)task_id;
            size_t WP = (size_t)wp_id;
            // LOG << Task << " " << WP << ": updating map waypoint.";

            if (Task == m_sStateData.TaskState && std::get<1>(GlobalMap[WP]) != Task) {
                std::get<1>(GlobalMap[WP]) = Task;
                IncreaseMovingProb();
            }
        }
        else {
            m_pEBMsg = NULL;
            // LOG << "none";
        }
        LOG << std::endl;
        m_pcRABA->ClearData();
    }
}

int CEyeBotMain::RecordTrial() {
    if(m_sStateData.IsLeader) {
        int greenCounter = 0;
        int targetCounter = 0;
        bool dataAlreadySet = false;

        CSimulator* Simulator;
        Simulator = &CSimulator::GetInstance();
        CSpace::TMapPerType& tLightMap = m_pcSpace->GetEntitiesByType("light");
        CLightEntity* cLightEnt;

        /* Retrieve and count each green light */
        for(CSpace::TMapPerType::iterator it = tLightMap.begin(); it != tLightMap.end(); ++it) {
            cLightEnt = any_cast<CLightEntity*>(it->second);

            if(cLightEnt->GetColor() == CColor::GREEN) {
                greenCounter++;
            }
            targetCounter++;
        }

        double targetThreshold = std::floor(m_sExperimentParams.target * targetCounter);
        double minTargetThreshold = std::floor(0.5 * targetCounter);

        /*
        * Setup data csv file if it doesn't exist.
        */
        std::stringstream header, sim_data;

        header << "Type,TargetNum,TargetThresh,SimStep,Completed,MinimumHold,";
        header << "LaunchStep,InitialRtMProb,RtMDelta,InitialRtLProb,RtLDelta,";
        header << "MinimumRest,InitialMinimumHold,MaximumHold,";
        header << "GlobalReach,ProximityThresh,Attitude,SwarmParticles,SwarmSelfTrust,";
        header << "SwarmPastTrust,SwarmGlobalTrust,SwarmAnts,MappingMean,MappingStdDev,";
        header << "MappingSeed,RtMMin,RtMMax,RtMSeed,RtLMin,RtLMax,RtLSeed,";
        header << "ACOSeed,TaskCompletedMin,TaskCompletedMax,TaskCompletedSeed,";
        header << "TargetShuffleMin,TargetShuffleMax,TargetShuffleSeed,";
        header << "NaiveMapping,VStep,HStep,SimStepMax,SimTrialNum,ArgosSeed\n";

        sim_data << (std::string)m_sExperimentParams.name << "," << targetCounter << "," << targetThreshold;
        sim_data << "," << m_pcSpace->GetSimulationClock() << "," << greenCounter;
        sim_data << "," << m_sStateData.minimum_hold_time << "," << m_sSwarmParams.launch_step << ",";
        sim_data << m_sStateData.InitialRestToMoveProb << "," << m_sStateData.SocialRuleRestToMoveDeltaProb << ",";
        sim_data << m_sStateData.InitialRestToLandProb << "," << m_sStateData.SocialRuleRestToLandDeltaProb << ",";
        sim_data << m_sStateData.minimum_rest_time << "," << m_sStateData.initial_minimum_hold_time << "," << m_sStateData.maximum_hold_time << ",";
        sim_data << m_sStateData.global_reach << "," << m_sStateData.proximity_tolerance << ",";
        sim_data << m_sStateData.attitude << "," << m_sSwarmParams.particles << ",";
        sim_data << m_sSwarmParams.self_trust << "," << m_sSwarmParams.past_trust << ",";
        sim_data << m_sSwarmParams.global_trust << "," << m_sSwarmParams.ants << ",";
        sim_data << m_sRandGen.mapping_mean << "," << m_sRandGen.mapping_stddev << ",";
        sim_data << m_sRandGen.mapping_seed << "," << m_sRandGen.rtm_min << ",";
        sim_data << m_sRandGen.rtm_max << "," << m_sRandGen.rtm_seed << ",";
        sim_data << m_sRandGen.rtl_min << "," << m_sRandGen.rtl_max << ",";
        sim_data << m_sRandGen.rtl_seed << "," << m_sRandGen.aco_seed << ",";
        sim_data << m_sRandGen.task_completed_min << "," << m_sRandGen.task_completed_max << ",";
        sim_data << m_sRandGen.task_completed_seed << "," << m_sRandGen.target_shuffle_min << ",";
        sim_data << m_sRandGen.target_shuffle_max << "," << m_sRandGen.target_shuffle_seed << ",";
        sim_data << m_sExperimentParams.naive_mapping << "," << m_sLawnParams.vstep << ",";
        sim_data << m_sLawnParams.hstep << "," << m_sExperimentParams.sim_step_max << "," << m_sExperimentParams.trial_num << ","<< Simulator->GetRandomSeed();

        if (m_sExperimentParams.csv == "") {
            while(!fileCreated) {
                m_sExperimentParams.csv = "data_" + (std::string)m_sExperimentParams.name + "_" + std::to_string(fileCounter) + ".csv";

                std::ifstream checkfile(m_sExperimentParams.csv);

                if(!checkfile) {
                    checkfile.close();
                    std::ofstream outfile;
                    outfile.open(m_sExperimentParams.csv, std::ios::app);
                    outfile << header.str();
                    fileCreated = true;
                    outfile.close();
                }

                fileCounter++;
            }
        } else if (m_sExperimentParams.csv != "") {
            std::ifstream checkfile(m_sExperimentParams.csv);
            int c = checkfile.peek();

            if (c == EOF) {
                std::ofstream outfile;
                outfile.open(m_sExperimentParams.csv, std::ios::app);
                outfile << header.str();
                outfile.close();
            }
        }

        // Record only when new trial data is generated, a simulator step hard limit is reached.
        if(m_pcSpace->GetSimulationClock() >= m_sExperimentParams.sim_step_max) {
            RLOG << "Failed Trial " << trialCounter;

            return EXIT_FAILURE;
        }

        if(greenCounter >= targetThreshold) {
            std::ifstream datafile(m_sExperimentParams.csv);
            string sim_data_last = getLastLine(datafile);

            std::istringstream last(sim_data_last);
            std::string token;

            while(std::getline(last, token, ',')) {
                dataAlreadySet = sim_data.str().find(token) != std::string::npos;
            }

            if( !dataAlreadySet ) {
                std::ofstream outfile;
                outfile.open(m_sExperimentParams.csv, std::ios::app);

                outfile << sim_data.str() << std::endl;
                outfile.close();
            }

            if( trialCounter < m_sExperimentParams.trials ) {
                RLOG << "Trial " << trialCounter << " Completed!";

                trialCounter++;
                Simulator->Terminate();
                Simulator->Reset();
            } else if( trialCounter == m_sExperimentParams.trials ) {
                RLOG << "All " << trialCounter << " Trials Completed!";
                Simulator->Terminate();
            }
        }
    }
    return EXIT_SUCCESS;
}

void CEyeBotMain::UpdateWaypoint() {
    if(m_sStateData.HoldTime > m_sStateData.minimum_hold_time) {
        m_sStateData.LocalIndex++;
        m_sStateData.HoldTime = 0;
        m_sStateData.minimum_hold_time = m_sStateData.initial_minimum_hold_time;
    } else if(m_sStateData.HoldTime > m_sStateData.maximum_hold_time) {
        m_sStateData.LocalIndex++;
        m_sStateData.HoldTime = 0;
        m_sStateData.minimum_hold_time = m_sStateData.initial_minimum_hold_time;
    } else {
        m_sStateData.HoldTime++;
    }
}

void CEyeBotMain::ActOnTarget(std::string action) {
    SStateData::ETask target_task = SStateData::TASK_INVALID;
    CColor target_color = CColor::BLACK;

    if(m_sRandGen.taskcompleted.get()) {
        // LOG << "completing " << action << " task!";
        for(size_t t_id = 0; t_id < m_pTargetMap.size() ; t_id++) {
            if(action == std::get<0>(m_pTargetMap[t_id])) {
                target_color = std::get<2>(m_pTargetMap[t_id]);
                target_task = std::get<1>(m_pTargetMap[t_id]);

                if(m_cNearestTarget->GetColor() == target_color) {
                    // LOG << "found " << target_color << " plant at " << "(" << m_cNearestTarget->GetPosition() << ")";
                    m_cNearestTarget->SetColor(CColor::GREEN);
                    std::get<1>(GlobalMap[GetGlobalIndex()]) = target_task;
                    std::get<2>(GlobalMap[GetGlobalIndex()]) = CColor::GREEN;
                    IncreaseMovingProb();
                } else if(m_cNearestTarget->GetColor() == CColor::GREEN) {
                    // LOG << "found " << CColor::GREEN << " plant at " << "(" << m_cNearestTarget->GetPosition() << ")";
                    std::get<1>(GlobalMap[GetGlobalIndex()]) = target_task;
                    std::get<2>(GlobalMap[GetGlobalIndex()]) = CColor::GREEN;
                    IncreaseLandingProb();
                }
            }
        }
        LOG  << std::endl;
        // RLOG << "Sending task: ";
        SendTask(target_task);
        UpdateWaypoint();
    }  else {
        // LOG << action << " task interrupted/not completed!";
        m_sStateData.minimum_hold_time += 1;
    }
}

void CEyeBotMain::IncreaseLandingProb() {
    // Increase probability that robot will go into land state.
    m_sStateData.RestToLandProb += m_sStateData.SocialRuleRestToLandDeltaProb;
    // Truncate RestToLand probability value.
    m_sStateData.RestToLandProb = fmax(fmin(m_sStateData.RestToLandProb, m_sRandGen.resttoland.max()), m_sRandGen.resttoland.min());
    // Decrease probability that robot will go into move state.
    m_sStateData.RestToMoveProb -= m_sStateData.SocialRuleRestToMoveDeltaProb;
    // Truncate RestToMove probability value.
    m_sStateData.RestToMoveProb = fmax(fmin(m_sStateData.RestToMoveProb, m_sRandGen.resttomove.max()), m_sRandGen.resttomove.min());
}

void CEyeBotMain::IncreaseMovingProb() {
    // Increase probability that robot will go into move state.
    m_sStateData.RestToMoveProb += m_sStateData.SocialRuleRestToMoveDeltaProb;
    // Truncate RestToMove probability value.
    m_sStateData.RestToMoveProb = fmax(fmin(m_sStateData.RestToMoveProb, m_sRandGen.resttomove.max()), m_sRandGen.resttomove.min());
    // Decrease probability that robot will go into land state.
    m_sStateData.RestToLandProb -= m_sStateData.SocialRuleRestToLandDeltaProb;
    // Truncate RestToLand probability value.
    m_sStateData.RestToLandProb = fmax(fmin(m_sStateData.RestToLandProb, m_sRandGen.resttoland.max()), m_sRandGen.resttoland.min());
}

void CEyeBotMain::SendTask(SStateData::ETask task) {

    if(m_sStateData.HoldTime == 1 && task != SStateData::TASK_INVALID && strcmp(m_sExperimentParams.name, "lawn")) {
        CByteArray cBuf(10);
        cBuf[0] = (UInt8)task                 & 0xff;
        cBuf[1] = (UInt8)GetGlobalIndex()     & 0xff;

        m_pcRABA->SetData(cBuf);
        // LOG << "sent";
    } else {
        // LOG << "cancelled";
    }
    LOG << std::endl;
}

size_t CEyeBotMain::GetGlobalIndex() {
    size_t g_id;
    // Search for index to global map waypoint
    for(auto& gwp : GlobalMap) {
        if(std::get<0>(gwp.second) == std::get<0>(LocalMap[m_sStateData.LocalIndex])) {
            g_id = gwp.first;
        }
    }
    return (g_id);
}

std::vector<double> CEyeBotMain::GetWaypoint() {
    return (std::get<0>(LocalMap[m_sStateData.LocalIndex]));
}

CVector3 CEyeBotMain::GetPosition() {
    return (m_sKalmanFilter.state);
}

/****************************************/
/****************************************/

void CEyeBotMain::EvaluateFunction() {
    SStateData::ETask TargetTask = SStateData::TASK_INVALID;
    CColor TargetColor = CColor::BLACK;

    // RLOG << "Processing...";
    // Probabilistically action and assign target state.
    if(m_sRandGen.taskcompleted.get()) {
        if(m_cNearestTarget->GetColor() == CColor::WHITE) {
            // LOG << "found untagged (white/grey) plant at " << "(" << m_cNearestTarget->GetPosition() << ")" << std::endl;

            TargetColor = std::get<2>(m_pTargetMap[m_sRandGen.targetshuffle.get()]);
            m_cNearestTarget->SetColor(TargetColor);
        }

        if(m_cNearestTarget->GetColor() == CColor::WHITE) {
            // LOG << "found retagged (white/grey) plant at " << "(" << m_cNearestTarget->GetPosition() << ")";
            TargetTask = SStateData::TASK_EVALUATE;
            TargetColor = CColor::WHITE;
        } else if(m_cNearestTarget->GetColor() == CColor::GREEN) {
            // LOG << "found healthy (green) plant at " << "(" << m_cNearestTarget->GetPosition() << ")";
            TargetTask = SStateData::TASK_NULL;
            TargetColor = CColor::GREEN;
        } else if(m_cNearestTarget->GetColor() == CColor::MAGENTA) {
            // LOG << "found dry (magenta) plant at " << "(" << m_cNearestTarget->GetPosition() << ")";
            TargetTask = SStateData::TASK_WATER;
            TargetColor = CColor::MAGENTA;
        } else if(m_cNearestTarget->GetColor() == CColor::YELLOW) {
            // LOG << "found malnourished (yellow) plant at " << "(" << m_cNearestTarget->GetPosition() << ")";
            TargetTask = SStateData::TASK_NOURISH;
            TargetColor = CColor::YELLOW;
        } else if(m_cNearestTarget->GetColor() == CColor::RED) {
            // LOG << "found sick (red) plant at " << "(" << m_cNearestTarget->GetPosition() << ")";
            TargetTask = SStateData::TASK_TREATMENT;
            TargetColor = CColor::RED;
        }
        LOG << std::endl;

        if(TargetTask != SStateData::TASK_INVALID && TargetColor != CColor::BLACK) {
            std::get<1>(GlobalMap[GetGlobalIndex()]) = SStateData::TASK_EVALUATE;
            std::get<2>(GlobalMap[GetGlobalIndex()]) = TargetColor;

            if(TargetTask == SStateData::TASK_NULL) {
                IncreaseLandingProb();
            } else {
                IncreaseMovingProb();
            }

            // RLOG  << std::endl << "Sending task: ";
            SendTask(TargetTask);
            UpdateWaypoint();
        }
    }  else {
        // LOG << "evaluation task interrupted/not completed!" << std::endl;
        m_sStateData.minimum_hold_time += 1;
    }
}

void CEyeBotMain::WaterFunction() {
    // RLOG << "Processing...";
    ActOnTarget("water");
}

void CEyeBotMain::NourishFunction() {
    // RLOG << "Processing...";
    ActOnTarget("nourish");
}

void CEyeBotMain::TreatmentFunction() {
    // RLOG << "Processing...";
    ActOnTarget("treatment");
}

/****************************************/
/****************************************/

void CEyeBotMain::SSwarmParams::Init(TConfigurationNode& t_node) {
    try {
        GetNodeAttribute(t_node, "ants", ants);
        GetNodeAttribute(t_node, "particles", particles);
        GetNodeAttribute(t_node, "self_trust", self_trust);
        GetNodeAttribute(t_node, "past_trust", past_trust);
        GetNodeAttribute(t_node, "global_trust", global_trust);
        GetNodeAttribute(t_node, "launch_step", launch_step);
    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing swarm parameters.", ex);
    }
}

void CEyeBotMain::SExperimentParams::Init(TConfigurationNode& t_node) {
    try {
        GetNodeAttribute(t_node, "trials", trials);
        GetNodeAttribute(t_node, "target", target);
        GetNodeAttribute(t_node, "name", name);
        GetNodeAttribute(t_node, "naive_mapping", naive_mapping);
        GetNodeAttribute(t_node, "csv", csv);
        GetNodeAttribute(t_node, "maximum_sim_step", sim_step_max);
        GetNodeAttribute(t_node, "trial_num", trial_num);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing experiment parameters.", ex);
    }
}

void CEyeBotMain::SRandomGen::Init(TConfigurationNode& t_node) {
    try {
        GetNodeAttribute(t_node, "mapping_seed", mapping_seed);
        GetNodeAttribute(t_node, "mapping_mean", mapping_mean);
        GetNodeAttribute(t_node, "mapping_stddev", mapping_stddev);
        GetNodeAttribute(t_node, "moving_seed", rtm_seed);
        GetNodeAttribute(t_node, "landing_seed", rtl_seed);
        GetNodeAttribute(t_node, "aco_seed", aco_seed);
        GetNodeAttribute(t_node, "target_shuffle_seed", target_shuffle_seed);
        GetNodeAttribute(t_node, "task_completed_seed", task_completed_seed);
    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing random parameters.", ex);
    }
}

void CEyeBotMain::SRandomGen::Set(size_t target_states_size) {
    rtl_min = 0.0;
    rtl_max = 1.0;
    rtm_min = 0.0;
    rtm_max = 1.0;
    task_completed_min = 0;
    task_completed_max = 1;
    target_shuffle_min = 0;
    target_shuffle_max = target_states_size;

    mapping.Init(mapping_mean, mapping_stddev, mapping_seed);
    resttomove.Init(rtm_min, rtm_max, rtm_seed);
    resttoland.Init(rtl_min, rtl_max, rtl_seed);
    taskcompleted.Init(task_completed_min, task_completed_max, task_completed_seed);
    targetshuffle.Init(target_shuffle_min, target_shuffle_max, target_shuffle_seed);
}

void CEyeBotMain::SLawnParams::Init(TConfigurationNode& t_node) {
    try {
        GetNodeAttribute(t_node, "hstep", hstep);
        GetNodeAttribute(t_node, "vstep", vstep);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing waypoint parameters.", ex);
    }
}

void CEyeBotMain::SStateData::Init(TConfigurationNode& t_node) {
    try {
        GetNodeAttribute(t_node, "initial_rest_to_move_prob", InitialRestToMoveProb);
        GetNodeAttribute(t_node, "social_rule_rest_to_move_delta_prob", SocialRuleRestToMoveDeltaProb);
        GetNodeAttribute(t_node, "initial_rest_to_land_prob", InitialRestToLandProb);
        GetNodeAttribute(t_node, "social_rule_rest_to_land_delta_prob", SocialRuleRestToLandDeltaProb);
        GetNodeAttribute(t_node, "global_reach", global_reach);
        GetNodeAttribute(t_node, "proximity_tolerance", proximity_tolerance);
        GetNodeAttribute(t_node, "attitude", attitude);
        GetNodeAttribute(t_node, "initial_minimum_hold_time", initial_minimum_hold_time);
        GetNodeAttribute(t_node, "maximum_hold_time", maximum_hold_time);
        GetNodeAttribute(t_node, "minimum_rest_time", minimum_rest_time);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing state parameters.", ex);
    }
}

void CEyeBotMain::SStateData::Reset() {
    State = SStateData::STATE_START;
    LocalIndex = 0;
    HoldTime = 0;
    RestTime = 0;
    RestToMoveProb = InitialRestToMoveProb;
    RestToLandProb = InitialRestToLandProb;
    minimum_hold_time = initial_minimum_hold_time;
}

/****************************************/
/****************************************/

void CEyeBotMain::SGaussDist::Init(double& mean, double& stddev, int& gen_seed) {
    gen = new std::default_random_engine(gen_seed);
    nd = new std::normal_distribution<double>(mean, stddev);
}

void CEyeBotMain::SUniformIntDist::Init(int min, int max, int& gen_seed) {
    gen = new std::default_random_engine(gen_seed);
    uid = new std::uniform_int_distribution<int>(min, max);
}

void CEyeBotMain::SUniformRealDist::Init(double min, double max, int& gen_seed) {
    gen = new std::default_random_engine(gen_seed);
    udd = new std::uniform_real_distribution<double>(min, max);
}

CEyeBotMain::SKF::SKF() {
    // Discrete LTI projectile motion, measuring position only
    A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
    C << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    // Initialize reasonable covariance matrices
    Q << .05, .0, .0, .0, .05, .0, .0, .0, .05;
    R << 5, 0, 0, 0, 5, 0, 0, 0, 5;
    P << 1000., 0., 0., 0., 1000., 0., 0., 0., 1000.;
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CEyeBotMain, "main_controller")