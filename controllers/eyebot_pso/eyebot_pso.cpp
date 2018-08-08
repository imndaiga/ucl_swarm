/* Include the controller definition */
#include "eyebot_pso.h"
/* Include the pso swarm algorithm definitions */
#include <algorithms/pso/swarm.h>
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

CEyeBotPso::CEyeBotPso() :
    m_pcPosAct(NULL),
    m_pcPosSens(NULL),
    m_pcProximity(NULL),
    m_pcCamera(NULL),
    m_pcSpace(NULL),
    m_pcRABA(NULL),
    m_pcRABS(NULL){}

/****************************************/
/****************************************/

void CEyeBotPso::Init(TConfigurationNode& t_node) {

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
        /* Get quadcopter launch parameters */
        m_sStateData.Init(GetNode(t_node, "state"));
        /* Get waypoint parameters */
        m_sWaypointParams.Init(GetNode(t_node, "waypoints"));
        /* Get the generator seed parameters */
        m_sSeedParams.Init(GetNode(t_node, "seeds"));
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

    /*
    * Initialize filters and noise models
    */
    UpdatePosition(m_pcPosSens->GetReading().Position);
    m_sMappingNoiseGen.Init(m_sWaypointParams.ns_mean, m_sWaypointParams.ns_stddev, m_sSeedParams.mapping);
    m_sTargetStateShuffleGen.Init(0, m_pTargetStates.size() - 1, m_sSeedParams.shuffle);
    m_sTaskCompletedGen.Init(0, 1, m_sSeedParams.success);
    m_sRestToMoveGen.Init(0.0, 1.0, m_sSeedParams.move);
    m_sRestToLandGen.Init(0.0, 1.0, m_sSeedParams.land);

    HomePos = m_sKalmanFilter.state;

    /* Enable camera filtering */
    m_pcCamera->Enable();
    Reset();
}

void CEyeBotPso::ControlStep() {
    UpdatePosition();
    UpdateNearestTarget();
    ListenToNeighbours();

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

void CEyeBotPso::Reset() {
    /* Reset robot state */
    m_sStateData.Reset();
    m_pGlobalMap.clear();
    m_pcRABA->ClearData();
}

/****************************************/
/****************************************/

void CEyeBotPso::Land() {
    if(m_sStateData.State != SStateData::STATE_LAND) {
        /* State initialization */
        m_sStateData.State = SStateData::STATE_LAND;
        m_cTargetPos = HomePos;
        m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
    } else {
        Rest();
    }
}

void CEyeBotPso::Move() {
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

void CEyeBotPso::ExecuteTask() {

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

void CEyeBotPso::Rest() {
    if(m_sStateData.State != SStateData::STATE_REST) {
        /* State initialize */
        m_sStateData.State = SStateData::STATE_REST;
    } else {
        /* if robot has stayed at current position long enough,
        *  probabilistically change to move state.
        */
        double RestToMoveCheck = m_sRestToMoveGen.Rand();
        double RestToLandCheck = m_sRestToLandGen.Rand();

        RLOG << "Rest check: " << RestToMoveCheck << std::endl;
        RLOG << "Land check: " << RestToLandCheck << std::endl;

        if(m_sStateData.RestTime > m_sStateData.minimum_rest_time && 
           RestToMoveCheck < m_sStateData.RestToMoveProb) {
            // Replanning unordered waypoints and add to local map.
            RLOG << "Replanning now." << std::endl;
            GenerateMap(m_sStateData.WaypointMap, m_sStateData.UnorderedWaypoints);
            m_sStateData.WaypointIndex = 0;

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

void CEyeBotPso::InitializeWaypoints(std::vector< std::vector<double> >& waypoints, bool naive) {
    std::vector<std::vector<double>> TargetLocations;

    if(naive) {
        CSpace::TMapPerType& tLightMap = m_pcSpace->GetEntitiesByType("light");

        /* Retrieve and store the positions of each light in the arena */
        for(CSpace::TMapPerType::iterator it = tLightMap.begin(); it != tLightMap.end(); ++it) {
            // cast the entity to a light entity
            CLightEntity& cLightEnt = *any_cast<CLightEntity*>(it->second);
            std::vector<double> l_vec;

            l_vec.push_back(cLightEnt.GetPosition().GetX());
            l_vec.push_back(cLightEnt.GetPosition().GetY() - m_sStateData.Reach);
            l_vec.push_back(cLightEnt.GetPosition().GetZ() + m_sStateData.attitude);
            TargetLocations.push_back(l_vec);
        }
    } else {
        /* Implement target seeking here. Would require SFM abilities? */
    }

    std::vector<double> noisyLoc;
    for(size_t p_ind = 0; p_ind < TargetLocations.size(); p_ind++) {
        noisyLoc.clear();
        for(std::vector<double>::iterator rd = TargetLocations[p_ind].begin(); rd != TargetLocations[p_ind].end(); rd++) {
            // Simulate gaussian sensor noise for each axis reading
            noisyLoc.push_back(*rd + m_sMappingNoiseGen.Rand());
        }
        waypoints.push_back(noisyLoc);
    }
}

void CEyeBotPso::GenerateMap(std::map<size_t, std::vector<double>>& map, std::vector< std::vector<double> >& unsorted_waypoints, bool verbose) {
    Swarm swarm(m_sSwarmParams.particles, m_sSwarmParams.self_trust, m_sSwarmParams.past_trust, m_sSwarmParams.global_trust, m_sStateData.UnorderedWaypoints, "cm");

    swarm_sol = swarm.optimize();

    for(size_t n=0; n < swarm_sol.tour.size(); n++) {
        // Store to waypoints map
        map[swarm_sol.tour[n]] = unsorted_waypoints[swarm_sol.tour[n]];
    }

    if(verbose) {
        RLOG << "PSO Tour Distance: " << swarm_sol.tour_length << std::endl;
        RLOG << "Shortest Path: ";
        for(size_t n=0; n < swarm_sol.tour.size(); n++) {
            LOG << swarm_sol.tour[n] << " - (";
            for(std::vector<double>::iterator twp_rd = unsorted_waypoints[swarm_sol.tour[n]].begin(); twp_rd != unsorted_waypoints[swarm_sol.tour[n]].end(); ++twp_rd) {
                LOG << *twp_rd << " ";
            }
            LOG << ")" << std::endl;
        }

        RLOG << "Waypoint Map: " << std::endl;
        for(std::map<size_t, std::vector<double>>::iterator map_wp = map.begin(); map_wp != map.end(); ++map_wp) {
            LOG << "Index: " << map_wp->first << std::endl << "Map Location: ( ";
            for(std::vector<double>::iterator mwp_rd = map_wp->second.begin(); mwp_rd != map_wp->second.end(); ++mwp_rd) {
                LOG << *mwp_rd << " ";
            }
            LOG << ")" << std::endl;
        }
    }

    // Clear unordered waypoints temporary container.
    unsorted_waypoints.clear();
}

void CEyeBotPso::UpdatePosition(CVector3 x0) {
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

void CEyeBotPso::UpdateNearestTarget() {
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

void CEyeBotPso::InitializeSwarm() {
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

        CEyeBotPso& cController = dynamic_cast<CEyeBotPso&>(cEyeBotEnt->GetControllableEntity().GetController());
        // Set controller state TaskState.
        cController.m_sStateData.TaskState = m_pTaskStates[task_id];
        // Set controller state Reach variable.
        cController.m_sStateData.Reach = cController.m_sStateData.global_reach + cController.m_sStateData.ReachModifiers[m_pTaskStates[task_id]];

        // Set controller TaskFunction.
        if(cController.m_sStateData.TaskState == SStateData::TASK_EVALUATE) {
            cController.TaskFunction = &CEyeBotPso::EvaluateFunction;
        } else if(cController.m_sStateData.TaskState == SStateData::TASK_WATER) {
            cController.TaskFunction = &CEyeBotPso::WaterFunction;
        } else if(cController.m_sStateData.TaskState == SStateData::TASK_NOURISH) {
            cController.TaskFunction = &CEyeBotPso::NourishFunction;
        } else if(cController.m_sStateData.TaskState == SStateData::TASK_TREATMENT) {
            cController.TaskFunction = &CEyeBotPso::TreatmentFunction;
        }

        InitializeWaypoints(cController.m_sStateData.UnorderedWaypoints);
        GenerateMap(cController.m_pGlobalMap, cController.m_sStateData.UnorderedWaypoints);
        cController.m_sStateData.WaypointIndex = 0;

        // Initialize one leader evaluation drone with the global map to traverse through.
        if(cController.m_sStateData.TaskState == SStateData::TASK_EVALUATE && node_count < m_pTaskStates.size() && cController.m_pGlobalMap.size() > 0) {
            for(size_t i=0; i < cController.m_pGlobalMap.size(); i++) {
                cController.m_sStateData.UnorderedWaypoints.push_back(cController.m_pGlobalMap[i]);
            }
        }

        m_mTaskedEyeBots[cEyeBotEnt->GetId()] = cController.m_sStateData.TaskState;
        node_count++;
    }

    LOG << "Tasked eyebot map: " << std::endl;
    for (std::map<std::string, SStateData::ETask>::const_iterator iter = m_mTaskedEyeBots.begin(); iter != m_mTaskedEyeBots.end(); iter++)
    {
        LOG << "Robot Id: " << iter->first << " " << "Task:" << iter->second << std::endl;
    }
}

void CEyeBotPso::ListenToNeighbours() {
    /*
    * Social rule: listen to what targets have been found.
    */
    RLOG << "Message received: ";
    UInt8 task_id, wp_id;

    if(! m_pcRABS->GetReadings().empty()) {
        m_pEBMsg = &(m_pcRABS->GetReadings()[0]);
        task_id = m_pEBMsg->Data[0];
        wp_id = m_pEBMsg->Data[1];

        ProcessWaypoint(task_id, wp_id);
    }
    else {
        m_pEBMsg = NULL;
        LOG << "none";
    }
    LOG << std::endl;
    m_pcRABA->ClearData();
}

void CEyeBotPso::ProcessWaypoint(UInt8& task_id, UInt8& wp_id) {

    if(task_id == SStateData::TASK_NULL) {
        // Increase probability that robot will go into land state.
        m_sStateData.RestToLandProb += m_sStateData.SocialRuleRestToLandDeltaProb;
        // Truncate RestToLand probability value.
        m_sStateData.RestToLandProb = fmax(fmin(m_sStateData.RestToLandProb, m_sRestToLandGen.max()), m_sRestToLandGen.min());
        // Decrease probability that robot will go into move state.
        m_sStateData.RestToMoveProb -= m_sStateData.SocialRuleRestToMoveDeltaProb;
        // Truncate RestToMove probability value.
        m_sStateData.RestToMoveProb = fmax(fmin(m_sStateData.RestToMoveProb, m_sRestToMoveGen.max()), m_sRestToMoveGen.min());
    } else if(task_id == m_sStateData.TaskState) {
        LOG << task_id << " " << wp_id << ": ";
        bool target_exists = false;

        for(size_t wp = 0; wp < m_sStateData.UnorderedWaypoints.size(); wp++) {
            if(m_pGlobalMap[wp_id] == m_sStateData.UnorderedWaypoints[wp]) {
                target_exists = true;
            }
        }

        if(!target_exists && m_sStateData.TaskState != SStateData::TASK_EVALUATE) {
            // Append new waypoints if not an evaluation drone.
            m_sStateData.UnorderedWaypoints.push_back(m_pGlobalMap[wp_id]);
            // Increase probability that robot will go into move state.
            m_sStateData.RestToMoveProb += m_sStateData.SocialRuleRestToMoveDeltaProb;
            // Truncate RestToMove probability value.
            m_sStateData.RestToMoveProb = fmax(fmin(m_sStateData.RestToMoveProb, m_sRestToMoveGen.max()), m_sRestToMoveGen.min());
            // Decrease probability that robot will go into land state.
            m_sStateData.RestToLandProb -= m_sStateData.SocialRuleRestToLandDeltaProb;
            // Truncate RestToLand probability value.
            m_sStateData.RestToLandProb = fmax(fmin(m_sStateData.RestToLandProb, m_sRestToLandGen.max()), m_sRestToLandGen.min());
            LOG << "appended.";
        } else {
            LOG << "discarded.";
        }
    } else {
        LOG << "forwarding: " << task_id << ", " << wp_id << ".";
        CByteArray cBuf(10);
        cBuf[0] = task_id       & 0xff;
        cBuf[1] = wp_id         & 0xff;
        m_pcRABA->SetData(cBuf);
    }
}

/****************************************/
/****************************************/

void CEyeBotPso::EvaluateFunction() {
    int IdentifiedTask = -1;

    if(m_cNearestTarget->GetColor() == CColor::WHITE) {
        RLOG << "Found untagged (white/grey) plant at " << "(" << m_cNearestTarget->GetPosition() << ")" << std::endl;
        // Probabilistically assign target state.
        CColor TargetColor = m_pTargetStates[m_sTargetStateShuffleGen.Rand()];
        m_cNearestTarget->SetColor(TargetColor);
    }

    RLOG << "Processing...";
    if(m_cNearestTarget->GetColor() == CColor::WHITE) {
        LOG << "found retagged (white/grey) plant at " << "(" << m_cNearestTarget->GetPosition();
        IdentifiedTask = SStateData::TASK_EVALUATE;
    } else if(m_cNearestTarget->GetColor() == CColor::GREEN) {
        LOG << "found healthy (green) plant at " << "(" << m_cNearestTarget->GetPosition();
        IdentifiedTask = SStateData::TASK_NULL;
    } else if(m_cNearestTarget->GetColor() == CColor::BROWN) {
        LOG << "found dry (brown) plant at " << "(" << m_cNearestTarget->GetPosition();
        IdentifiedTask = SStateData::TASK_WATER;
    } else if(m_cNearestTarget->GetColor() == CColor::YELLOW) {
        LOG << "found malnourished (yellow) plant at " << "(" << m_cNearestTarget->GetPosition();
        IdentifiedTask = SStateData::TASK_NOURISH;
    } else if(m_cNearestTarget->GetColor() == CColor::RED) {
        LOG << "found sick (red) plant at " << "(" << m_cNearestTarget->GetPosition();
        IdentifiedTask = SStateData::TASK_TREATMENT;
    }

    if(IdentifiedTask != SStateData::TASK_NULL) {
        // Increase probability that robot will go into move state.
        m_sStateData.RestToMoveProb += m_sStateData.SocialRuleRestToMoveDeltaProb;
        // Truncate RestToMove probability value.
        m_sStateData.RestToMoveProb = fmax(fmin(m_sStateData.RestToMoveProb, m_sRestToMoveGen.max()), m_sRestToMoveGen.min());
        // Decrease probability that robot will go into land state.
        m_sStateData.RestToLandProb -= m_sStateData.SocialRuleRestToLandDeltaProb;
        // Truncate RestToLand probability value.
        m_sStateData.RestToLandProb = fmax(fmin(m_sStateData.RestToLandProb, m_sRestToLandGen.max()), m_sRestToLandGen.min());
    } else {
        // Decrease probability that robot will go into move state.
        m_sStateData.RestToMoveProb -= m_sStateData.SocialRuleRestToMoveDeltaProb;
        // Truncate RestToMove probability value.
        m_sStateData.RestToMoveProb = fmax(fmin(m_sStateData.RestToMoveProb, m_sRestToMoveGen.max()), m_sRestToMoveGen.min());
        // Increase probability that robot will go into land state.
        m_sStateData.RestToLandProb += m_sStateData.SocialRuleRestToLandDeltaProb;
        // Truncate RestToLand probability value.
        m_sStateData.RestToLandProb = fmax(fmin(m_sStateData.RestToLandProb, m_sRestToLandGen.max()), m_sRestToLandGen.min());
    }

    LOG  << std::endl << ") Sending task: ";
    if(m_sStateData.HoldTime == 1 && IdentifiedTask != -1) {
        // Signal task to neighbouring eyebots once and continue to next waypoint
        LOG << IdentifiedTask << ", " << (UInt8)m_sStateData.WaypointIndex ;
        CByteArray cBuf(10);
        cBuf[0] = IdentifiedTask                            & 0xff;
        cBuf[1] = (UInt8)m_sStateData.WaypointIndex         & 0xff;

        m_pcRABA->SetData(cBuf);
    } else {
        LOG << "cancelled";
    }
    LOG << std::endl;

    UpdateWaypoint();
}

void CEyeBotPso::WaterFunction() {
    if(m_cNearestTarget->GetColor() == CColor::BROWN && m_sStateData.TaskState == SStateData::TASK_WATER) {
        m_cNearestTarget->SetColor(CColor::GREEN);
        // if(m_sTaskCompletedGen.Rand()) {
        //     m_cNearestTarget->SetColor(CColor::GREEN);
        //     UpdateWaypoint();
        // } else {
        //     LOG << "Watering task not completed!" << std::endl;
        // }
    }
    UpdateWaypoint();
}

void CEyeBotPso::NourishFunction() {
    if(m_cNearestTarget->GetColor() == CColor::YELLOW && m_sStateData.TaskState == SStateData::TASK_NOURISH) {
        m_cNearestTarget->SetColor(CColor::GREEN);
        // if(m_sTaskCompletedGen.Rand()) {
        //     m_cNearestTarget->SetColor(CColor::GREEN);
        //     UpdateWaypoint();
        // } else {
        //     LOG << "Nourishing task not completed!" << std::endl;
        // }
    }
    UpdateWaypoint();
}

void CEyeBotPso::TreatmentFunction() {
    if(m_cNearestTarget->GetColor() == CColor::RED && m_sStateData.TaskState == SStateData::TASK_TREATMENT) {
        m_cNearestTarget->SetColor(CColor::GREEN);
        // if(m_sTaskCompletedGen.Rand()) {
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

void CEyeBotPso::SSwarmParams::Init(TConfigurationNode& t_node) {
    try {
        GetNodeAttribute(t_node, "particles", particles);
        GetNodeAttribute(t_node, "self_trust", self_trust);
        GetNodeAttribute(t_node, "past_trust", past_trust);
        GetNodeAttribute(t_node, "global_trust", global_trust);
    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing swarm parameters.", ex);
    }
}

void CEyeBotPso::SWaypointParams::Init(TConfigurationNode& t_node) {
    try {
        GetNodeAttribute(t_node, "ns_mean", ns_mean);
        GetNodeAttribute(t_node, "ns_stddev", ns_stddev);
        GetNodeAttribute(t_node, "naive_mapping", naive_mapping);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing waypoint parameters.", ex);
    }
}

void CEyeBotPso::SSeedParams::Init(TConfigurationNode& t_node) {
    try {
        GetNodeAttribute(t_node, "mapping", mapping);
        GetNodeAttribute(t_node, "shuffle", shuffle);
        GetNodeAttribute(t_node, "success", success);
        GetNodeAttribute(t_node, "move", move);
        GetNodeAttribute(t_node, "land", land);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing seed parameters.", ex);
    }
}

void CEyeBotPso::SStateData::Init(TConfigurationNode& t_node) {
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

void CEyeBotPso::SStateData::Reset() {
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

void CEyeBotPso::SGaussDist::Init(double& mean, double& stddev, int& gen_seed) {
    gen = new std::default_random_engine(gen_seed);
    nd = new std::normal_distribution<double>(mean, stddev);
}

void CEyeBotPso::SUniformIntDist::Init(int min, int max, int& gen_seed) {
    gen = new std::default_random_engine(gen_seed);
    uid = new std::uniform_int_distribution<int>(min, max);
}

void CEyeBotPso::SUniformRealDist::Init(double min, double max, int& gen_seed) {
    gen = new std::default_random_engine(gen_seed);
    udd = new std::uniform_real_distribution<double>(min, max);
}

CEyeBotPso::SKF::SKF() {
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

REGISTER_CONTROLLER(CEyeBotPso, "eyebot_pso_controller")