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

    m_pcPosAct    = GetActuator <CCI_QuadRotorPositionActuator             >("quadrotor_position");
    m_pcPosSens   = GetSensor   <CCI_PositioningSensor                     >("positioning"       );
    m_pcProximity = GetSensor   <CCI_EyeBotProximitySensor                 >("eyebot_proximity"  );
    m_pcCamera    = GetSensor   <CCI_ColoredBlobPerspectiveCameraSensor    >("colored_blob_perspective_camera");
    m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
    m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
    m_pcSpace     = &CSimulator::GetInstance().GetSpace();
    kf = new KalmanFilter(m_sKalmanFilter.dt, m_sKalmanFilter.A, m_sKalmanFilter.C, m_sKalmanFilter.Q, m_sKalmanFilter.R, m_sKalmanFilter.P);

    /*
    * Parse the config file
    */
    try {
        /* Get swarm parameters */
        m_sSwarmParams.Init(GetNode(t_node, "swarm"));
        /* Get quadcopter launch parameters */
        m_sDroneParams.Init(GetNode(t_node, "drone"));
        /* Get waypoint parameters */
        m_sWaypointParams.Init(GetNode(t_node, "waypoints"));
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

    /*
    * Initialize filters and noise models
    */
    int mp_seed = 123;
    int tss_seed = 123;
    int tc_seed = 123;
    UpdatePosition(m_pcPosSens->GetReading().Position);
    m_sMappingNoise.Init(m_sWaypointParams.ns_mean, m_sWaypointParams.ns_stddev, mp_seed);
    m_sTargetStateShuffle.Init(0, m_pTargetStates.size() - 1, tss_seed);
    m_sTaskCompleted.Init(0, 1, tc_seed);

    HomePos = m_sKalmanFilter.state;

    /* Enable camera filtering */
    m_pcCamera->Enable();
    Reset();
}

void CEyeBotPso::ControlStep() {
    UpdatePosition();
    UpdateNearestLight();

    switch(m_sStateData.State) {
        case SStateData::STATE_START:
            /*
            * Distribute tasks between available eye-bots
            * in the arena. The allocator must be run before
            * we can generate waypoints.
            */
            AllocateTasks();
            /*
            * Map all targets in the arena: this can be done naively
            * with the passed argos parameters or with the help
            * of the camera sensor.
            */
            GenerateWaypoints(m_sWaypointParams.naive_mapping);
            if(m_sStateData.TaskState == SStateData::TASK_EVALUATE) {
                TaskFunction = &CEyeBotPso::EvaluateFunction;
            } else if(m_sStateData.TaskState == SStateData::TASK_WATER) {
                TaskFunction = &CEyeBotPso::WaterFunction;
            } else if(m_sStateData.TaskState == SStateData::TASK_NOURISH) {
                TaskFunction = &CEyeBotPso::NourishFunction;
            } else if(m_sStateData.TaskState == SStateData::TASK_TREATMENT) {
                TaskFunction = &CEyeBotPso::TreatmentFunction;
            }
            TakeOff();
            break;
        case SStateData::STATE_TAKE_OFF:
            TakeOff();
            break;
        case SStateData::STATE_ADVANCE:
            WaypointAdvance();
            break;
        case SStateData::STATE_EXECUTE_TASK:
            ExecuteTask();
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
    RLOG << "Waypoint: " << m_sStateData.Waypoint + 1 << std::endl;
}

void CEyeBotPso::Reset() {
    /* Start the behavior */

    /* Reset robot state */
    m_sStateData.Reset();
    m_pcRABA->ClearData();
}

/****************************************/
/****************************************/

void CEyeBotPso::TakeOff() {
    if(m_sStateData.State != SStateData::STATE_TAKE_OFF) {
        /* State initialization */
        m_sStateData.State = SStateData::STATE_TAKE_OFF;
        m_cTargetPos = HomePos + CVector3(0.0, m_sStateData.Reach, m_sDroneParams.launch_altitude);
        m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
    } else {
        if(Distance(m_cTargetPos, m_sKalmanFilter.state) < m_sDroneParams.proximity_tolerance) {
            /* State transition */
            WaypointAdvance();
        }
    }
}

void CEyeBotPso::Land() {
    if(m_sStateData.State != SStateData::STATE_LAND) {
        /* State initialization */
        m_sStateData.State = SStateData::STATE_LAND;
        m_cTargetPos = m_sKalmanFilter.state;
        m_cTargetPos.SetZ(0.0f);
        m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
    }
}

void CEyeBotPso::WaypointAdvance() {
    if(m_sStateData.State != SStateData::STATE_ADVANCE) {
        /* State initialization */
        m_sStateData.State = SStateData::STATE_ADVANCE;
    } else {
        if(m_sStateData.WaypointMap.size() > 0 && m_sStateData.Waypoint < m_sStateData.WaypointMap.size()) {
            std::vector<double> target_wp = std::get<0>(m_sStateData.WaypointMap[m_sStateData.Waypoint]);
            m_cTargetPos = CVector3(target_wp[0], target_wp[1], target_wp[2]);
            m_pcPosAct->SetAbsolutePosition(m_cTargetPos);

            if(Distance(m_cTargetPos, m_sKalmanFilter.state) < m_sDroneParams.proximity_tolerance) {
                /* State transition */
                ExecuteTask();
            }
        } else if (m_sStateData.Waypoint == m_sStateData.WaypointMap.size() && m_sStateData.WaypointMap.size() > 0) {
            /* State transition */
            RLOG << "Traversed all waypoints." << std::endl;
            m_cTargetPos = HomePos;
            m_pcPosAct->SetAbsolutePosition(m_cTargetPos);

            if(Distance(m_cTargetPos, m_sKalmanFilter.state) < m_sDroneParams.proximity_tolerance) {
                Land();
            }
        } else if(m_sStateData.Waypoint > m_sStateData.WaypointMap.size()) {
            LOG << "[ERROR] Waypoint outside of range." << std::endl;
            RLOG << "[ERROR] Waypoint outside of range." << std::endl;
            Land();
        } else if( m_sStateData.WaypointMap.size() == 0) {
            RLOG << "No waypoints have been generated." << std::endl;
            Land();
        }
    }
}

void CEyeBotPso::GenerateWaypoints(bool& naive) {

    if(naive) {
        CSpace::TMapPerType& tLightMap = m_pcSpace->GetEntitiesByType("light");

        /* Retrieve and store the positions of each light in the arena */
        for(CSpace::TMapPerType::iterator it = tLightMap.begin(); it != tLightMap.end(); ++it) {
            // cast the entity to a light entity
            CLightEntity& cLightEnt = *any_cast<CLightEntity*>(it->second);
            std::vector<double> l_vec;

            l_vec.push_back(cLightEnt.GetPosition().GetX());
            l_vec.push_back(cLightEnt.GetPosition().GetY() - m_sStateData.Reach);
            l_vec.push_back(cLightEnt.GetPosition().GetZ() + m_sDroneParams.attitude);
            m_cPlantLocList.push_back(l_vec);
        }
    } else {
        /* Implement target seeking here. Would require SFM abilities? */
    }

    std::vector<double> noisyLoc;
    std::vector<std::vector<double>> tmpWaypoints;
    for(size_t p_ind = 0; p_ind < m_cPlantLocList.size(); p_ind++) {
        noisyLoc.clear();
        for(std::vector<double>::iterator rd = m_cPlantLocList[p_ind].begin(); rd != m_cPlantLocList[p_ind].end(); rd++) {
            // Simulate gaussian sensor noise for each axis reading
            noisyLoc.push_back(*rd + m_sMappingNoise.Rand());
        }
        tmpWaypoints.push_back(noisyLoc);
    }

    Swarm swarm(m_sSwarmParams.particles, m_sSwarmParams.self_trust, m_sSwarmParams.past_trust, m_sSwarmParams.global_trust, tmpWaypoints, "cm");

    swarm_sol = swarm.optimize();

    LOG << "PSO Tour Distance: " << swarm_sol.tour_length << std::endl;
    LOG << "Shortest Path: ";
    for(size_t n=0; n < swarm_sol.tour.size(); n++) {
        LOG << swarm_sol.tour[n] << " - (";
        for(std::vector<double>::iterator twp_rd = tmpWaypoints[swarm_sol.tour[n]].begin(); twp_rd != tmpWaypoints[swarm_sol.tour[n]].end(); ++twp_rd) {
            LOG << *twp_rd << " ";
        }
        LOG << ")" << std::endl;
        // Store to waypoints map
        m_sStateData.WaypointMap[swarm_sol.tour[n]] = std::make_pair(tmpWaypoints[swarm_sol.tour[n]], SStateData::TASK_NULL);
    }

    LOG << "Waypoint Map: " << std::endl;
    for(std::map<size_t, std::pair<std::vector<double>, SStateData::ETask>>::iterator map_wp = m_sStateData.WaypointMap.begin(); map_wp != m_sStateData.WaypointMap.end(); ++map_wp) {
        LOG << "Index: " << map_wp->first << std::endl << "Map Location: ( ";
        for(std::vector<double>::iterator mwp_rd = std::get<0>(map_wp->second).begin(); mwp_rd != std::get<0>(map_wp->second).end(); ++mwp_rd) {
            LOG << *mwp_rd << " ";
        }
        LOG << ")" << std::endl << "Tag: " << std::get<1>(map_wp->second) << std::endl;
    }
}

void CEyeBotPso::MapWall(bool& naive) {
    CVector3 wallPos;

    if(naive) {
        /* Retrieve the wall object in the arena*/
        CSpace::TMapPerType& tBoxMap = m_pcSpace->GetEntitiesByType("box");
        CBoxEntity* cBoxEnt = any_cast<CBoxEntity*>(tBoxMap["wall_north"]);
    } else {
        /* Implement target seeking here. Would require SFM abilities? */
    }

    LOG << "Simulator-extracted wall props: " << std::endl;
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

void CEyeBotPso::UpdateNearestLight() {
    CSpace::TMapPerType& tLightMap = m_pcSpace->GetEntitiesByType("light");
    CLightEntity* cLightEnt;
    /* Retrieve and evaluate the positions of each light in the arena */
    for(CSpace::TMapPerType::iterator it = tLightMap.begin(); it != tLightMap.end(); ++it) {
        // cast the entity to a light entity
        cLightEnt = any_cast<CLightEntity*>(it->second);
        CVector3 compensated_waypoint = m_cTargetPos + CVector3(0.0, m_sStateData.Reach, -m_sDroneParams.attitude);

        if(m_cTargetLight) {
            if(Distance(cLightEnt->GetPosition(), compensated_waypoint) < Distance(m_cTargetLight->GetPosition(), compensated_waypoint)) {
                m_cTargetLight = cLightEnt;
            }
        } else {
            m_cTargetLight = cLightEnt;
        }
    }
}

void CEyeBotPso::ExecuteTask() {

    if(m_sStateData.State != SStateData::STATE_EXECUTE_TASK) {
        /* State initialization */
        m_sStateData.State = SStateData::STATE_EXECUTE_TASK;
    } else {
        /* State logic */
        (this->*TaskFunction)();
        /* State transition */
        WaypointAdvance();
    }
}

void CEyeBotPso::AllocateTasks() {
    CSpace::TMapPerType& tEyeBotMap = m_pcSpace->GetEntitiesByType("eye-bot");
    CEyeBotEntity* cEyeBotEnt;
    int task_id = 0;

    /* Retrieve and assign tasks to each eye-bot */
    for(CSpace::TMapPerType::iterator it = tEyeBotMap.begin(); it != tEyeBotMap.end(); ++it, task_id++) {
        // Cast the entity to a eye-bot entity
        cEyeBotEnt = any_cast<CEyeBotEntity*>(it->second);

        if(task_id > m_pTaskStates.size() - 1) {
            // Reset index if greater than the number of tasks available
            task_id = 0;
        }
        // Set controller task variable.
        CEyeBotPso& cController = dynamic_cast<CEyeBotPso&>(cEyeBotEnt->GetControllableEntity().GetController());
        cController.m_sStateData.TaskState = m_pTaskStates[task_id];
        m_mTaskedEyeBots[cEyeBotEnt->GetId()] = m_pTaskStates[task_id];
    }
    // Initialize the eyebots task allocator
    m_sStateData.Init(m_sDroneParams.global_reach);

    LOG << "Tasked eyebot map: " << std::endl;
    for (std::map<std::string, SStateData::ETask>::const_iterator iter = m_mTaskedEyeBots.begin(); iter != m_mTaskedEyeBots.end(); iter++)
    {
        LOG << "Robot Id: " << iter->first << " " << "Task:" << iter->second << std::endl;
    }
}

/*
* Based on the assigned tag perform varied tasks.
* White - reassign tag to plant
* Green - leave plant alone
* Yellow - apply medication
* Red - water the plant
*/

void CEyeBotPso::EvaluateFunction() {
    if(m_cTargetLight->GetColor() == CColor::WHITE || m_cTargetLight->GetColor() == CColor::GRAY50) {
        LOG << "Found untagged (white/grey) plant at " << "(" << m_cTargetLight->GetPosition() << ")" << std::endl;
        // Probabilistically assign target state.
        CColor TargetColor = m_pTargetStates[m_sTargetStateShuffle.Rand()];
        m_cTargetLight->SetColor(TargetColor);
        m_sStateData.Waypoint++;
    } else if(m_cTargetLight->GetColor() == CColor::GREEN) {
        LOG << "Found healthy (green) plant at " << "(" << m_cTargetLight->GetPosition() << ")" << std::endl;
        m_sStateData.Waypoint++;
    } else if(m_cTargetLight->GetColor() == CColor::BROWN) {
        LOG << "Found dry (brown) plant at " << "(" << m_cTargetLight->GetPosition() << ")" << std::endl;
        // Wait until water drone responds with a confirmation packet.
        // m_sStateData.Waypoint++;
    } else if(m_cTargetLight->GetColor() == CColor::YELLOW) {
        LOG << "Found malnourished (yellow) plant at " << "(" << m_cTargetLight->GetPosition() << ")" << std::endl;
        // m_pcRABA->SetData(0, m_sStateData.WaypointMap[m_sStateData.Waypoint]);
        // Wait until nourish drone responds with a confirmation packet.
        // m_sStateData.Waypoint++;
    } else if(m_cTargetLight->GetColor() == CColor::RED) {
        LOG << "Found sick (red) plant at " << "(" << m_cTargetLight->GetPosition() << ")" << std::endl;
        // m_pcRABA->SetData(0, m_sStateData.WaypointMap[m_sStateData.Waypoint]);
        // Wait until treatment drone responds with a confirmation packet.
        // m_sStateData.Waypoint++;
    }
}

void CEyeBotPso::WaterFunction() {
    if(m_cTargetLight->GetColor() == CColor::BROWN && m_sStateData.TaskState == SStateData::TASK_WATER) {
        if(m_sTaskCompleted.Rand()) {
            // m_pcRABA->SetData(0, m_sStateData.WaypointMap[m_sStateData.Waypoint]);
            m_sStateData.Waypoint++;
        } else {
            LOG << "Watering task not completed!" << std::endl;
        }
    }
}

void CEyeBotPso::NourishFunction() {
    if(m_cTargetLight->GetColor() == CColor::YELLOW && m_sStateData.TaskState == SStateData::TASK_NOURISH) {
        if(m_sTaskCompleted.Rand()) {
            // m_pcRABA->SetData(0, m_sStateData.WaypointMap[m_sStateData.Waypoint]);
            m_sStateData.Waypoint++;
        } else {
            LOG << "Nourishing task not completed!" << std::endl;
        }
    }
}

void CEyeBotPso::TreatmentFunction() {
    if(m_cTargetLight->GetColor() == CColor::RED && m_sStateData.TaskState == SStateData::TASK_TREATMENT) {
        if(m_sTaskCompleted.Rand()) {
            // m_pcRABA->SetData(0, m_sStateData.WaypointMap[m_sStateData.Waypoint]);
            m_sStateData.Waypoint++;
        } else {
            LOG << "Treatment task not completed!" << std::endl;
        }
    }
}

/****************************************/
/****************************************/

void CEyeBotPso::SSwarmParams::Init(TConfigurationNode& t_node) {
    try {
        int p_size;
        double trust_val;

        GetNodeAttribute(t_node, "particles", p_size);
        particles = p_size;
        GetNodeAttribute(t_node, "self_trust", trust_val);
        self_trust = trust_val;
        GetNodeAttribute(t_node, "past_trust", trust_val);
        past_trust = trust_val;
        GetNodeAttribute(t_node, "global_trust", trust_val);
        global_trust = trust_val;
    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing swarm parameters.", ex);
    }
}

void CEyeBotPso::SDroneParams::Init(TConfigurationNode& t_node) {
    try {
        Real p_val;

        GetNodeAttribute(t_node, "launch_altitude", p_val);
        launch_altitude = p_val;
        GetNodeAttribute(t_node, "global_reach", p_val);
        global_reach = p_val;
        GetNodeAttribute(t_node, "proximity_tolerance", p_val);
        proximity_tolerance = p_val;
        GetNodeAttribute(t_node, "attitude", p_val);
        attitude = p_val;
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing quadcopter launch parameters.", ex);
    }
}

void CEyeBotPso::SWaypointParams::Init(TConfigurationNode& t_node) {
    try {
        double param_val;
        bool param_bool;

        GetNodeAttribute(t_node, "ns_mean", param_val);
        ns_mean = param_val;
        GetNodeAttribute(t_node, "ns_stddev", param_val);
        ns_stddev = param_val;
        GetNodeAttribute(t_node, "naive_mapping", param_bool);
        naive_mapping = param_bool;
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing waypoint parameters.", ex);
    }
}

void CEyeBotPso::SGaussDist::Init(double& mean, double& stddev, int& gen_seed) {
    gen = new std::default_random_engine(gen_seed);
    nd = new std::normal_distribution<double>(mean, stddev);
}

void CEyeBotPso::SUniformIntDist::Init(int min, int max, int& gen_seed) {
    gen = new std::default_random_engine(gen_seed);
    uid = new std::uniform_int_distribution<int>(min, max);
}

void CEyeBotPso::SStateData::Init(double& global_reach) {
    switch(TaskState) {
        case SStateData::TASK_EVALUATE:
            Reach = global_reach * 1.5;
            break;
        case SStateData::TASK_WATER:
            Reach = global_reach * 1.1;
            break;
        case SStateData::TASK_NOURISH:
            Reach = global_reach * 0.7;
            break;
        case SStateData::TASK_TREATMENT:
            Reach = global_reach * 0.3;
            break;
        default:
            break;
    }
}

void CEyeBotPso::SStateData::Reset() {
    State = SStateData::STATE_START;
    Waypoint = 0;
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