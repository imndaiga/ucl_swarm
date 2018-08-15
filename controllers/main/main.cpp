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
    UpdatePosition();
    UpdateNearestTarget();
    ListenToNeighbours();
    Record();

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
    RLOG << "Current state: " << m_sStateData.State << std::endl;
    RLOG << "Target pos: " << m_cTargetPos << std::endl;
    RLOG << "Current pos: " << m_pcPosSens->GetReading().Position << std::endl;
    RLOG << "Filtered pos: " << GetPosition() << std::endl;
    RLOG << "Local map size: " << LocalMap.size() << std::endl;
    RLOG << "Local index: " << m_sStateData.LocalIndex << std::endl;
    RLOG << "Global map size: " << GlobalMap.size() << std::endl;
    RLOG << "Holding time: " << m_sStateData.HoldTime << std::endl;
    RLOG << "Resting time: " << m_sStateData.RestTime << std::endl;
    RLOG << "RestToMove: " << m_sStateData.RestToMoveProb << std::endl;
    RLOG << "RestToLand: " << m_sStateData.RestToLandProb << std::endl;
}

void CEyeBotMain::Reset() {
    /* Reset robot state */
    m_sRandGen.Set((size_t)m_pTargetStates.size()-1);
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
        // Encourage motion in land state when running lawn experiment.
        if(!strcmp(m_sExperimentParams.name, "lawn")) {
            IncreaseMovingProb();
        }
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
            UpdateLocalMap();

            /* State transition */
            Move();
            m_sStateData.RestTime = 0;
        } else if (m_sStateData.RestTime > m_sStateData.minimum_rest_time &&
                   RestToLandCheck < m_sStateData.RestToLandProb) {
            // Land once inspection is probabilistically complete.
            RLOG << "Completed inspections. Landing now." << std::endl;

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
            GlobalMap[global_tour[n]] = std::make_pair(raw_waypoints[global_tour[n]], SStateData::TASK_EVALUATE);
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
                for(auto& rd: wp.second.first) {
                    LOG << rd << " ";
                }
                LOG << ")" << std::endl;
            }
        }
    } else if (raw_waypoints.size() > 0 && !strcmp(m_sExperimentParams.name, "lawn")) {
        for(size_t i = 0; i < raw_waypoints.size(); i++) {
            // Store sorted waypoint into WaypointMap.
            GlobalMap[i] = std::make_pair(raw_waypoints[i], SStateData::TASK_EVALUATE);
        }

        if(verbose) {
            RLOG << "Wall size: " << WallSize << std::endl;
            RLOG << "GlobalMap size: " << GlobalMap.size() << std::endl;
            RLOG << "Global Waypoint Map: " << std::endl;
            for(auto& wp : GlobalMap) {
                LOG << "Index: " << wp.first << " Map Location: (";
                for(auto& rd: wp.second.first) {
                    LOG << rd << " ";
                }
                LOG << ")" << std::endl;
            }
        }
    }
}

void CEyeBotMain::UpdateLocalMap(bool verbose) {
    LocalMap.clear();

    if(strcmp(m_sExperimentParams.name, "lawn")) {
        std::vector< std::vector<double> > unsorted_waypoints;
        tour.clear();
        tour_length = 0;

        for(auto& wp : GlobalMap) {
            if(wp.second.second == m_sStateData.TaskState) {
                unsorted_waypoints.push_back(wp.second.first);
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
                LocalMap[tour[n]] = std::make_pair(unsorted_waypoints[tour[n]], m_sStateData.TaskState);
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

            RLOG << "Local Waypoint Map: " << std::endl;
            for(auto& wp : LocalMap) {
                LOG << "Index: " << wp.first << " Map Location: (";
                for(auto& rd: wp.second.first) {
                    LOG << rd << " ";
                }
                LOG << ")" << std::endl;
            }
        }
    } else if(!strcmp(m_sExperimentParams.name, "lawn")) {
        size_t local_index = 0;

        // Only update local map with unmarked targets.
        for(auto& wp : GlobalMap) {
            if(wp.second.second != SStateData::TASK_NULL) {
                LocalMap[local_index] = std::make_pair(wp.second.first, SStateData::TASK_EVALUATE);
                local_index++;
            }
        }

        if(verbose) {
            RLOG << "Local Waypoint Map: " << std::endl;
            for(auto& wp : LocalMap) {
                LOG << "Index: " << wp.first << " Map Location: (";
                for(auto& rd: wp.second.first) {
                    LOG << rd << " ";
                }
                LOG << ")" << std::endl;
            }
        }
    }
    // Reset Waypoint index
    m_sStateData.LocalIndex = 0;
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

void CEyeBotMain::InitializeDrones() {
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

        // Vary resting and landing probabilities if lawn experiment.
        if(!strcmp(cController.m_sExperimentParams.name, "lawn")) {
            // Increase probability that robot will go into land state.
            cController.m_sStateData.RestToLandProb += cController.m_sStateData.SocialRuleRestToLandDeltaProb * node_count;
            // Truncate RestToLand probability value.
            cController.m_sStateData.RestToLandProb = fmax(fmin(cController.m_sStateData.RestToLandProb, cController.m_sRandGen.resttoland.max()), cController.m_sRandGen.resttoland.min());
            // Decrease probability that robot will go into move state.
            cController.m_sStateData.RestToMoveProb -= cController.m_sStateData.SocialRuleRestToMoveDeltaProb * node_count;
            // Truncate RestToMove probability value.
            cController.m_sStateData.RestToMoveProb = fmax(fmin(cController.m_sStateData.RestToMoveProb, cController.m_sRandGen.resttomove.max()), cController.m_sRandGen.resttomove.min());
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
        if(node_count < m_pTaskStates.size() && cController.m_sStateData.TaskState == SStateData::TASK_EVALUATE) {
            cController.m_sStateData.IsLeader = true;
        } else {
            cController.m_sStateData.IsLeader = false;
        }

        m_mTaskedEyeBots[cEyeBotEnt->GetId()] = cController.m_sStateData.TaskState;
        node_count++;
    }

    drones_initialized = true;

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

    if(drones_initialized && strcmp(m_sExperimentParams.name, "lawn")) {
        RLOG << "Message received: ";
        UInt8 task_id, wp_id;

        if(! m_pcRABS->GetReadings().empty()) {
            m_pEBMsg = &(m_pcRABS->GetReadings()[0]);
            task_id = m_pEBMsg->Data[0];
            wp_id = m_pEBMsg->Data[1];

            // Process received message.
            SStateData::ETask Task = (SStateData::ETask)task_id;
            size_t WP = (size_t)wp_id;

            if(Task == SStateData::TASK_NULL) {
                GlobalMap[WP].second = SStateData::TASK_NULL;
                IncreaseLandingProb();
            } else if(Task == m_sStateData.TaskState && GlobalMap[WP].second != Task) {
                LOG << Task << " " << WP << ": updating map waypoint.";
                GlobalMap[WP].second = Task;
                IncreaseMovingProb();
            } else {
                LOG << "forwarding: ";
                SendTask(Task);
            }
        }
        else {
            m_pEBMsg = NULL;
            LOG << "none";
        }
        LOG << std::endl;
        m_pcRABA->ClearData();
    }
}

/****************************************/
/****************************************/

void CEyeBotMain::EvaluateFunction() {
    SStateData::ETask TargetTask = SStateData::TASK_INVALID;

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
        IncreaseLandingProb();
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

    GlobalMap[GetGlobalIndex()].second = TargetTask;

    LOG  << std::endl << "Sending task: ";
    SendTask(TargetTask);
    UpdateWaypoint();
}

void CEyeBotMain::WaterFunction() {
    RLOG << "Processing...";
    if(m_cNearestTarget->GetColor() == CColor::BROWN && m_sStateData.TaskState == SStateData::TASK_WATER) {
        m_cNearestTarget->SetColor(CColor::GREEN);
        GlobalMap[m_sStateData.LocalIndex].second = SStateData::TASK_NULL;
        SendTask(SStateData::TASK_NULL);
    } else if(m_cNearestTarget->GetColor() == CColor::GREEN) {
        LOG << "found healthy (green) plant at " << "(" << m_cNearestTarget->GetPosition() << ")";
        GlobalMap[GetGlobalIndex()].second = SStateData::TASK_NULL;
        SendTask(SStateData::TASK_NULL);
    }
    UpdateWaypoint();
}

void CEyeBotMain::NourishFunction() {
    RLOG << "Processing...";
    if(m_cNearestTarget->GetColor() == CColor::YELLOW && m_sStateData.TaskState == SStateData::TASK_NOURISH) {
        m_cNearestTarget->SetColor(CColor::GREEN);
        GlobalMap[m_sStateData.LocalIndex].second = SStateData::TASK_NULL;
        SendTask(SStateData::TASK_NULL);
    } else if(m_cNearestTarget->GetColor() == CColor::GREEN) {
        LOG << "found healthy (green) plant at " << "(" << m_cNearestTarget->GetPosition() << ")";
        GlobalMap[GetGlobalIndex()].second = SStateData::TASK_NULL;
        SendTask(SStateData::TASK_NULL);

    }
    UpdateWaypoint();
}

void CEyeBotMain::TreatmentFunction() {
    RLOG << "Processing...";
    if(m_cNearestTarget->GetColor() == CColor::RED && m_sStateData.TaskState == SStateData::TASK_TREATMENT) {
        m_cNearestTarget->SetColor(CColor::GREEN);
        GlobalMap[m_sStateData.LocalIndex].second = SStateData::TASK_NULL;
        SendTask(SStateData::TASK_NULL);
    } else if(m_cNearestTarget->GetColor() == CColor::GREEN) {
        LOG << "found healthy (green) plant at " << "(" << m_cNearestTarget->GetPosition() << ")";
        GlobalMap[GetGlobalIndex()].second = SStateData::TASK_NULL;
        SendTask(SStateData::TASK_NULL);

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
        GetNodeAttribute(t_node, "minimum_hold_time", minimum_hold_time);
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