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
    m_pcRABS(NULL){}

/****************************************/
/****************************************/

void CEyeBotMain::Init(TConfigurationNode& t_node) {

    m_pcPosAct       = GetActuator <CCI_QuadRotorPositionActuator             >("quadrotor_position"             );
    m_pcRABA         = GetActuator <CCI_RangeAndBearingActuator               >("range_and_bearing"              );
    m_pcPosSens      = GetSensor   <CCI_PositioningSensor                     >("positioning"                    );
    m_pcProximity    = GetSensor   <CCI_EyeBotProximitySensor                 >("eyebot_proximity"               );
    m_pcCamera       = GetSensor   <CCI_ColoredBlobPerspectiveCameraSensor    >("colored_blob_perspective_camera");
    m_pcRABS         = GetSensor   <CCI_RangeAndBearingSensor                 >("range_and_bearing"              );
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
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

    /*
    * Initialize kalman filter.
    */
    UpdatePosition(m_pcPosSens->GetReading().Position);

    HomePos = m_sKalmanFilter.state;

    /* Enable camera filtering */
    m_pcCamera->Enable();
    Reset();
}

void CEyeBotMain::ControlStep() {
    UpdatePosition();
    UpdateNearestTarget();
    ListenToNeighbours();
    Record();

    switch(m_sStateData.State) {
        case SStateData::STATE_START:
            // Initialize tasks and global map.
            InitializeSwarm();
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
    RLOG << "Current state: " << m_sStateData.State << std::endl;
    RLOG << "Target pos: " << m_cTargetPos << std::endl;
    RLOG << "Current pos: " << m_pcPosSens->GetReading().Position << std::endl;
    RLOG << "Filtered pos: " << m_sKalmanFilter.state << std::endl;
    RLOG << "Waypoint index: " << m_sStateData.WaypointIndex << std::endl;
    RLOG << "Incoming waypoint size: " << m_sStateData.UnorderedWaypoints.size() << std::endl;
    RLOG << "Local map size: " << m_sStateData.WaypointMap.size() << std::endl;
    RLOG << "Global map size: " << m_pGlobalMap.size() << std::endl;
    RLOG << "Holding time: " << m_sStateData.HoldTime << std::endl;
    RLOG << "Resting time: " << m_sStateData.RestTime << std::endl;
    RLOG << "RestToMove: " << m_sStateData.RestToMoveProb << std::endl;
    RLOG << "RestToLand: " << m_sStateData.RestToLandProb << std::endl;
}

void CEyeBotMain::Reset() {
    /* Reset robot state */
    m_sStateData.Reset();
    m_pGlobalMap.clear();
    m_pcRABA->ClearData();
    fileCreated = false;
    fileCounter++;
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
        if(m_sStateData.WaypointIndex < m_sStateData.WaypointMap.size()) {
            std::vector<double> target_wp = GetWaypoint();
            m_cTargetPos = CVector3(target_wp[0], target_wp[1], target_wp[2]);
            m_pcPosAct->SetAbsolutePosition(m_cTargetPos);

            if(Distance(m_cTargetPos, m_sKalmanFilter.state) < m_sStateData.proximity_tolerance) {
                /* State transition */
                ExecuteTask();
            }
        } else {
            /* State transition */
            // WaypointMap is empty or in error, go to rest state
            RLOG << "No waypoints available. Resting now." << std::endl;
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
        RLOG << "Executing task." << std::endl;
        (this->*TaskFunction)();
        /* State transition */
        Move();
    }
}

void CEyeBotMain::Rest() {
    if(m_sStateData.State != SStateData::STATE_REST) {
        /* State initialize */
        m_sStateData.State = SStateData::STATE_REST;
    } else {
        /* if robot has stayed at current position long enough,
        *  probabilistically change to move state.
        */
        double RestToMoveCheck = m_sRandGen.resttomove.get();
        double RestToLandCheck = m_sRandGen.resttoland.get();

        RLOG << "Rest check: " << RestToMoveCheck << std::endl;
        RLOG << "Land check: " << RestToLandCheck << std::endl;

        if(m_sStateData.RestTime > m_sStateData.minimum_rest_time && 
           RestToMoveCheck < m_sStateData.RestToMoveProb) {
            // Replanning unordered waypoints and add to local map.
            RLOG << "Replanning now." << std::endl;
            GenerateMap(m_sStateData.WaypointMap, m_sStateData.UnorderedWaypoints);

            /* State transition */
            Move();
            m_sStateData.RestTime = 0;
        } else if (RestToMoveCheck > m_sStateData.RestToMoveProb &&
                   RestToLandCheck < m_sStateData.RestToLandProb) {
            // Land once inspection is probabilistically complete.
            RLOG << "Completed inspections. Landing now." << std::endl;
            m_sStateData.WaypointMap.clear();

            /* State transition */
            Land();
        } else {
            m_sStateData.RestTime++;
        }
    }
}

/****************************************/
/****************************************/

void CEyeBotMain::InitializeWaypoints(std::vector< std::vector<double> >& waypoints) {
    std::vector<std::vector<double>> TargetLocations;

    if(m_sExperimentParams.naive_mapping) {
        CSpace::TMapPerType& tLightMap = m_pcSpace->GetEntitiesByType("light");

        /* Retrieve and store the positions of each light in the arena */
        for(CSpace::TMapPerType::iterator it = tLightMap.begin(); it != tLightMap.end(); ++it) {
            // cast the entity to a light entity
            CLightEntity& cLightEnt = *any_cast<CLightEntity*>(it->second);
            std::vector<double> l_vec;

            l_vec.push_back(cLightEnt.GetPosition().GetX());
            l_vec.push_back(cLightEnt.GetPosition().GetY());
            l_vec.push_back(cLightEnt.GetPosition().GetZ());
            TargetLocations.push_back(l_vec);
        }
    } else {
        /* Implement target seeking here. Would require SFM abilities? */
    }

    for(size_t p_i = 0; p_i < TargetLocations.size(); p_i++) {
        std::vector<double> modified_loc;
        // Simulate gaussian sensor noise for each axis reading
        // and include reach and attitude variables.
        modified_loc.push_back(TargetLocations[p_i][0] + m_sRandGen.mapping.get());
        modified_loc.push_back(TargetLocations[p_i][1] - m_sStateData.Reach + m_sRandGen.mapping.get());
        modified_loc.push_back(TargetLocations[p_i][2] + m_sStateData.attitude + m_sRandGen.mapping.get());

        waypoints.push_back(modified_loc);
    }
}

void CEyeBotMain::GenerateMap(std::map<size_t, std::pair< std::vector<double>, CColor >>& map, std::vector< std::vector<double> >& unsorted_waypoints, bool verbose) {
    tour.clear();
    tour_length = 0;

    if(unsorted_waypoints.size() > 0) {

        if(!strcmp(m_sExperimentParams.name, "pso")) {
            PsoSwarm swarm(m_sSwarmParams.particles, m_sSwarmParams.self_trust, m_sSwarmParams.past_trust, m_sSwarmParams.global_trust, m_sStateData.UnorderedWaypoints, "cm");
            swarm.optimize(tour, tour_length);
        } else if(!strcmp(m_sExperimentParams.name, "aco")) {
            AcoSwarm swarm(m_sSwarmParams.ants, m_sStateData.UnorderedWaypoints, m_sRandGen.aco_seed, "cm");
            swarm.optimize(tour, tour_length);
        }

        for(size_t n=0; n < tour.size(); n++) {
            // Store to waypoints map
            map[tour[n]] = std::make_pair(unsorted_waypoints[tour[n]], CColor::WHITE);
        }

        if(verbose) {
            RLOG << "Tour Distance: " << tour_length << std::endl;
            RLOG << "Shortest Path: ";
            for(size_t n=0; n < tour.size(); n++) {
                LOG << tour[n] << " - (";
                for(std::vector<double>::iterator twp_rd = unsorted_waypoints[tour[n]].begin(); twp_rd != unsorted_waypoints[tour[n]].end(); ++twp_rd) {
                    LOG << *twp_rd << " ";
                }
                LOG << ")" << std::endl;
            }

            RLOG << "Waypoint Map: " << std::endl;
            for(auto& wp : map) {
                LOG << "Index: " << wp.first << " Map Location: (";
                for(auto& rd: wp.second.first) {
                    LOG << rd << " ";
                }
                LOG << ")" << std::endl;
            }
        }
    }
    // Clear unordered waypoints temporary container.
    unsorted_waypoints.clear();
    m_sStateData.WaypointIndex = 0;
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

void CEyeBotMain::InitializeSwarm() {
    CSpace::TMapPerType& tEyeBotMap = m_pcSpace->GetEntitiesByType("eye-bot");
    CEyeBotEntity* cEyeBotEnt;
    int task_id = 0;
    int node_count = 0;

    /* Retrieve and assign tasks to each eye-bot */
    for(CSpace::TMapPerType::iterator it = tEyeBotMap.begin(); it != tEyeBotMap.end(); ++it, task_id++) {
        // Cast the entity to a eye-bot entity
        cEyeBotEnt = any_cast<CEyeBotEntity*>(it->second);

        if(task_id > m_pTaskStates.size() - 1) {
            // Reset index if greater than the number of tasks available
            task_id = 0;
        }

        CEyeBotMain& cController = dynamic_cast<CEyeBotMain&>(cEyeBotEnt->GetControllableEntity().GetController());
        // Set controller state TaskState.
        cController.m_sStateData.TaskState = m_pTaskStates[task_id];
        // Set controller state Reach variable.
        cController.m_sStateData.Reach = cController.m_sStateData.global_reach + cController.m_sStateData.ReachModifiers[m_pTaskStates[task_id]];

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

        m_mTaskedEyeBots[cEyeBotEnt->GetId()] = cController.m_sStateData.TaskState;
        node_count++;
    }

    InitializeWaypoints(m_sStateData.UnorderedWaypoints);
    GenerateMap(m_pGlobalMap, m_sStateData.UnorderedWaypoints);

    // Initialize one leader evaluation drone with the global map to traverse through.
    if(m_sStateData.TaskState == SStateData::TASK_EVALUATE && m_pGlobalMap.size() > 0) {
        for(size_t i=0; i < m_pGlobalMap.size(); i++) {
            m_sStateData.UnorderedWaypoints.push_back(m_pGlobalMap[i].first);
        }
        m_sStateData.IsLeader = true;
    } else {
        m_sStateData.IsLeader = false;
    }
    swarm_initialized = true;

    LOG << "Tasked eyebot map: " << std::endl;
    for (std::map<std::string, SStateData::ETask>::const_iterator iter = m_mTaskedEyeBots.begin(); iter != m_mTaskedEyeBots.end(); iter++)
    {
        LOG << "Robot Id: " << iter->first << " " << "Task:" << iter->second << std::endl;
    }
}

void CEyeBotMain::ListenToNeighbours() {
    /*
    * Social rule: listen to what targets have been found.
    */

    if(swarm_initialized) {
        RLOG << "Message received: ";
        UInt8 task_id, wp_id, agent_id;

        if(! m_pcRABS->GetReadings().empty()) {
            m_pEBMsg = &(m_pcRABS->GetReadings()[0]);
            task_id = m_pEBMsg->Data[0];
            wp_id = m_pEBMsg->Data[1];
            agent_id = m_pEBMsg->Data[2];

            ProcessWaypoint(task_id, wp_id, agent_id);
        }
        else {
            m_pEBMsg = NULL;
            LOG << "none";
        }
        LOG << std::endl;
        m_pcRABA->ClearData();
    }
}

void CEyeBotMain::ProcessWaypoint(UInt8& task_id, UInt8& wp_id, UInt8& agent_id) {

    if(task_id == SStateData::TASK_NULL) {
        IncreaseLandingProb();
    } else if(task_id == m_sStateData.TaskState) {
        LOG << task_id << " " << wp_id << ": ";
        bool target_exists = false;

        for(size_t wp = 0; wp < m_sStateData.UnorderedWaypoints.size(); wp++) {
            if(m_pGlobalMap[wp_id].first == m_sStateData.UnorderedWaypoints[wp]) {
                target_exists = true;
            }
        }

        if(!target_exists && !m_sStateData.IsLeader) {
            // Append new waypoints if not the leader drone.
            m_sStateData.UnorderedWaypoints.push_back(m_pGlobalMap[wp_id].first);
            IncreaseMovingProb();
            LOG << "appended.";
        } else {
            LOG << "discarded.";
        }
    } else {
        LOG << "forwarding: ";
        UInt8 EmptyID = -1;
        SendTask(task_id, EmptyID);
    }
}

/****************************************/
/****************************************/

void CEyeBotMain::EvaluateFunction() {
    UInt8 TargetTask = -1;

    if(m_cNearestTarget->GetColor() == CColor::WHITE) {
        RLOG << "Found untagged (white/grey) plant at " << "(" << m_cNearestTarget->GetPosition() << ")" << std::endl;
        // Probabilistically assign target state.
        CColor TargetColor = m_pTargetStates[m_sRandGen.targetshuffle.get()];
        m_cNearestTarget->SetColor(TargetColor);
    }

    RLOG << "Processing...";
    if(m_cNearestTarget->GetColor() == CColor::WHITE) {
        LOG << "found retagged (white/grey) plant at " << "(" << m_cNearestTarget->GetPosition() << ")";
        TargetTask = SStateData::TASK_EVALUATE;
    } else if(m_cNearestTarget->GetColor() == CColor::GREEN) {
        LOG << "found healthy (green) plant at " << "(" << m_cNearestTarget->GetPosition() << ")";
        TargetTask = SStateData::TASK_NULL;
        m_pGlobalMap[m_sStateData.WaypointIndex].second = CColor::GREEN;
    } else if(m_cNearestTarget->GetColor() == CColor::BROWN) {
        LOG << "found dry (brown) plant at " << "(" << m_cNearestTarget->GetPosition() << ")";
        TargetTask = SStateData::TASK_WATER;
    } else if(m_cNearestTarget->GetColor() == CColor::YELLOW) {
        LOG << "found malnourished (yellow) plant at " << "(" << m_cNearestTarget->GetPosition() << ")";
        TargetTask = SStateData::TASK_NOURISH;
    } else if(m_cNearestTarget->GetColor() == CColor::RED) {
        LOG << "found sick (red) plant at " << "(" << m_cNearestTarget->GetPosition() << ")";
        TargetTask = SStateData::TASK_TREATMENT;
    }

    if(m_sStateData.IsLeader) {
        if(TargetTask != SStateData::TASK_NULL) {
            IncreaseMovingProb();
        } else {
            IncreaseLandingProb();
        }
    }

    LOG  << std::endl << "Sending task: ";
    UInt8 EmptyID = -1;
    SendTask(TargetTask, EmptyID);
    UpdateWaypoint();
}

void CEyeBotMain::WaterFunction() {
    if(m_cNearestTarget->GetColor() == CColor::BROWN && m_sStateData.TaskState == SStateData::TASK_WATER) {
        m_cNearestTarget->SetColor(CColor::GREEN);
        // if(m_sRandGen.taskcompleted.get()) {
        //     m_cNearestTarget->SetColor(CColor::GREEN);
        //     UpdateWaypoint();
        // } else {
        //     LOG << "Watering task not completed!" << std::endl;
        // }
    }
    UpdateWaypoint();
}

void CEyeBotMain::NourishFunction() {
    if(m_cNearestTarget->GetColor() == CColor::YELLOW && m_sStateData.TaskState == SStateData::TASK_NOURISH) {
        m_cNearestTarget->SetColor(CColor::GREEN);
        // if(m_sRandGen.taskcompleted.get()) {
        //     m_cNearestTarget->SetColor(CColor::GREEN);
        //     UpdateWaypoint();
        // } else {
        //     LOG << "Nourishing task not completed!" << std::endl;
        // }
    }
    UpdateWaypoint();
}

void CEyeBotMain::TreatmentFunction() {
    if(m_cNearestTarget->GetColor() == CColor::RED && m_sStateData.TaskState == SStateData::TASK_TREATMENT) {
        m_cNearestTarget->SetColor(CColor::GREEN);
        // if(m_sRandGen.taskcompleted.get()) {
        //     m_cNearestTarget->SetColor(CColor::GREEN);
        //     UpdateWaypoint();
        // } else {
        //     LOG << "Treatment task not completed!" << std::endl;
        // }
    }
    UpdateWaypoint();
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
    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing swarm parameters.", ex);
    }
}

void CEyeBotMain::SExperimentParams::Init(TConfigurationNode& t_node) {
    try {
        GetNodeAttribute(t_node, "name", name);
        GetNodeAttribute(t_node, "naive_mapping", naive_mapping);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing experiment parameters.", ex);
    }
}

void CEyeBotMain::SRandomGen::Init(TConfigurationNode& t_node) {
    try {
        double mapping_mean, mapping_stddev;
        int mapping_seed, moving_seed, landing_seed, target_shuffle_seed, task_completed_seed;

        GetNodeAttribute(t_node, "mapping_seed", mapping_seed);
        GetNodeAttribute(t_node, "moving_seed", moving_seed);
        GetNodeAttribute(t_node, "landing_seed", landing_seed);
        GetNodeAttribute(t_node, "aco_seed", aco_seed);
        GetNodeAttribute(t_node, "target_shuffle_seed", target_shuffle_seed);
        GetNodeAttribute(t_node, "task_completed_seed", task_completed_seed);

        mapping.Init(mapping_mean, mapping_stddev, mapping_seed);
        resttomove.Init(0.0, 1.0, moving_seed);
        resttoland.Init(0.0, 1.0, landing_seed);
        taskcompleted.Init(0, 1, task_completed_seed);
        // The following integers should be monitored and updated
        // from m_pcTargetStates.
        targetshuffle.Init(0, 4, target_shuffle_seed);
    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing random parameters.", ex);
    }
}

void CEyeBotMain::SStateData::Init(TConfigurationNode& t_node) {
    try {
        GetNodeAttribute(t_node, "initial_rest_to_move_prob", InitialRestToMoveProb);
        GetNodeAttribute(t_node, "social_rule_rest_to_move_delta_prob", SocialRuleRestToMoveDeltaProb);
        GetNodeAttribute(t_node, "initial_move_to_land_prob", InitialRestToLandProb);
        GetNodeAttribute(t_node, "social_rule_move_to_land_delta_prob", SocialRuleRestToLandDeltaProb);
        GetNodeAttribute(t_node, "global_reach", global_reach);
        GetNodeAttribute(t_node, "proximity_tolerance", proximity_tolerance);
        GetNodeAttribute(t_node, "attitude", attitude);
        GetNodeAttribute(t_node, "minimum_hold_time", minimum_hold_time);
        GetNodeAttribute(t_node, "minimum_rest_time", minimum_rest_time);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing state parameters.", ex);
    }
}

void CEyeBotMain::SStateData::Reset() {
    State = SStateData::STATE_START;
    WaypointIndex = 0;
    HoldTime = 0;
    RestTime = 0;
    WaypointMap.clear();
    UnorderedWaypoints.clear();
    RestToMoveProb = InitialRestToMoveProb;
    RestToLandProb = InitialRestToLandProb;
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