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
    m_pcSpace(NULL){}

/****************************************/
/****************************************/

void CEyeBotPso::Init(TConfigurationNode& t_node) {

    m_pcPosAct    = GetActuator <CCI_QuadRotorPositionActuator             >("quadrotor_position");
    m_pcPosSens   = GetSensor   <CCI_PositioningSensor                     >("positioning"       );
    m_pcProximity = GetSensor   <CCI_EyeBotProximitySensor                 >("eyebot_proximity"  );
    m_pcCamera    = GetSensor   <CCI_ColoredBlobPerspectiveCameraSensor    >("colored_blob_perspective_camera");
    m_pcSpace     = &CSimulator::GetInstance().GetSpace();
    kf = new KalmanFilter(m_sKalmanFilter.dt, m_sKalmanFilter.A, m_sKalmanFilter.C, m_sKalmanFilter.Q, m_sKalmanFilter.R, m_sKalmanFilter.P);

    /*
    * Parse the config file
    */
    try {
        /* Get swarm parameters */
        m_sSwarmParams.Init(GetNode(t_node, "swarm"));
        /* Get quadcopter launch parameters */
        m_sQuadLaunchParams.Init(GetNode(t_node, "launch"));
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

    switch(m_eState) {
        case STATE_START:
            /*
            * Map targets in the arena: this can be done naively
            * with the passed argos parameters or with the help
            * of the camera sensor.
            */
            GenerateWaypoints(m_sWaypointParams.naive_mapping, m_sWaypointParams.add_origin);
            /*
            * Distribute tasks between available eye-bots
            * in the arena.
            */
            AllocateTasks();
            TakeOff();
            break;
        case STATE_TAKE_OFF:
            TakeOff();
            break;
        case STATE_ADVANCE:
            WaypointAdvance();
            break;
        case STATE_EXECUTE_TASK:
            ExecuteTask();
            break;
        case STATE_LAND:
            Land();
            break;
        default:
            LOGERR << "[BUG] Unknown robot state: " << m_eState << std::endl;
    }

    /* Write debug information */
    RLOG << "Current state: " << m_eState << std::endl;
    RLOG << "Target pos: " << m_cTargetPos << std::endl;
    RLOG << "Current pos: " << m_pcPosSens->GetReading().Position << std::endl;
    RLOG << "Filtered pos: " << m_sKalmanFilter.state << std::endl;
    RLOG << "Waypoint: " << m_unWaypoint + 1 << std::endl;
}

void CEyeBotPso::Reset() {
    /* Start the behavior */
    m_eState = STATE_START;
    m_unWaypoint = 0;
}

/****************************************/
/****************************************/

void CEyeBotPso::TakeOff() {
    if(m_eState != STATE_TAKE_OFF) {
        /* State initialization */
        m_eState = STATE_TAKE_OFF;
        m_cTargetPos = HomePos + CVector3(0.0, m_sAllocations.reach, m_sQuadLaunchParams.altitude);
        m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
    } else {
        if(Distance(m_cTargetPos, m_sKalmanFilter.state) < m_sQuadLaunchParams.proximity_tolerance) {
            /* State transition */
            WaypointAdvance();
        }
    }
}

void CEyeBotPso::Land() {
    if(m_eState != STATE_LAND) {
        /* State initialization */
        m_eState = STATE_LAND;
        m_cTargetPos = m_sKalmanFilter.state;
        m_cTargetPos.SetZ(0.0f);
        m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
    }
}

void CEyeBotPso::WaypointAdvance() {
    if(m_eState != STATE_ADVANCE) {
        /* State initialization */
        m_eState = STATE_ADVANCE;
    } else {
        if(swarm_sol.tour.size() > 0 && m_unWaypoint < WaypointPositions.size()) {
            wp_loc target_wp = WaypointPositions[swarm_sol.tour[m_unWaypoint]];
            m_cTargetPos = CVector3(target_wp[0], target_wp[1], target_wp[2]);
            m_pcPosAct->SetAbsolutePosition(m_cTargetPos);

            if(Distance(m_cTargetPos, m_sKalmanFilter.state) < m_sQuadLaunchParams.proximity_tolerance) {
                /* State transition */
                ExecuteTask();
            }
        } else if (m_unWaypoint == WaypointPositions.size()) {
            /* State transition */
            m_cTargetPos = HomePos;
            m_pcPosAct->SetAbsolutePosition(m_cTargetPos);

            if(Distance(m_cTargetPos, m_sKalmanFilter.state) < m_sQuadLaunchParams.proximity_tolerance) {
                Land();
            }
        } else if(m_unWaypoint == WaypointPositions.size()) {
            LOG << "[ERROR] Waypoint outside of range." << std::endl;
            Land();
        } else if(swarm_sol.tour.size() == 0) {
            LOG << "No waypoints have been swarm generated." << std::endl;
            Land();
        }
    }
}

void CEyeBotPso::GenerateWaypoints(bool& naive, bool& add_origin) {
    /*
    * We can only generate waypoints after
    * the task allocator has been
    * initialized.
    */
    m_sAllocations.Init(m_sQuadLaunchParams.reach);
    if(naive) {
        CSpace::TMapPerType& tLightMap = m_pcSpace->GetEntitiesByType("light");

        /* Retrieve and store the positions of each light in the arena */
        for(CSpace::TMapPerType::iterator it = tLightMap.begin(); it != tLightMap.end(); ++it) {
            // cast the entity to a light entity
            CLightEntity& cLightEnt = *any_cast<CLightEntity*>(it->second);
            std::vector<double> l_vec;

            l_vec.push_back(cLightEnt.GetPosition().GetX());
            l_vec.push_back(cLightEnt.GetPosition().GetY() - m_sAllocations.reach);
            l_vec.push_back(cLightEnt.GetPosition().GetZ() + m_sWaypointParams.z_assess);
            m_cPlantLocList.push_back(l_vec);
        }
    } else {
        /* Implement target seeking here. Would require SFM abilities? */
    }

    if(add_origin) {
        wp_loc originPos;

        originPos.push_back(m_sKalmanFilter.state.GetX());
        originPos.push_back(m_cPlantLocList[0][1]);
        originPos.push_back(m_sKalmanFilter.state.GetZ());

        WaypointPositions.push_back(originPos);
    }

    WaypointPositions.insert(WaypointPositions.end(), m_cPlantLocList.begin(), m_cPlantLocList.end());

    // /* Simulate gaussian sensor noise for each axis reading */
    for(size_t wp = 0; wp < WaypointPositions.size(); wp++) {
        WaypointPositions[wp][0] = WaypointPositions[wp][0] + m_sMappingNoise.Rand();
        WaypointPositions[wp][1] = WaypointPositions[wp][1] + m_sMappingNoise.Rand();
        WaypointPositions[wp][2] = WaypointPositions[wp][2] + m_sMappingNoise.Rand();
    }

    LOG << "Waypoint locations: " << std::endl;
    for(size_t t=0; t < WaypointPositions.size(); t++) {
        LOG << WaypointPositions[t][0] << ", " << WaypointPositions[t][1] << ", " << WaypointPositions[t][2] << std::endl;
    }

    Swarm swarm(m_sSwarmParams.particles, m_sSwarmParams.self_trust, m_sSwarmParams.past_trust, m_sSwarmParams.global_trust, WaypointPositions, "cm");
    swarm_sol = swarm.optimize();

    LOG << "PSO Tour Distance: " << swarm_sol.tour_length << std::endl;
    LOG << "Shortest Path: ";
    for(size_t n=0; n < swarm_sol.tour.size(); n++) {
        LOG << swarm_sol.tour[n] << " ";
    }
    LOG << std::endl;
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
        CVector3 compensated_waypoint = m_cTargetPos + CVector3(0.0, m_sAllocations.reach, -m_sWaypointParams.z_assess);

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
    /*
    * Based on the assigned tag perform varied tasks.
    * White - reassign tag to plant
    * Green - leave plant alone
    * Yellow - apply medication
    * Red - water the plant
    */

    if(m_eState != STATE_EXECUTE_TASK) {
        /* State initialization */
        m_eState = STATE_EXECUTE_TASK;
    } else {
        /* State logic */
        if((m_cTargetLight->GetColor() == CColor::WHITE || m_cTargetLight->GetColor() == CColor::GRAY50) && m_sAllocations.task == EVALUATE_TASK) {
            LOG << "Found untagged (white/grey) plant at " << "(" << m_cTargetLight->GetPosition() << ")" << std::endl;
            // Probabilistically assign target state.
            if(m_unWaypoint < WaypointPositions.size()) {
                m_cTargetLight->SetColor(m_pTargetStates[m_sTargetStateShuffle.Rand()]);
            }
            m_unWaypoint++;
        } else if(m_cTargetLight->GetColor() == CColor::GREEN) {
            LOG << "Found healthy (green) plant at " << "(" << m_cTargetLight->GetPosition() << ")" << std::endl;
            // Nothing to do here.
            m_unWaypoint++;
        } else if(m_cTargetLight->GetColor() == CColor::BROWN && m_sAllocations.task == WATER_TASK) {
            LOG << "Found dry (brown) plant at " << "(" << m_cTargetLight->GetPosition() << ")" << std::endl;
            if(m_sTaskCompleted.Rand()) {
                m_unWaypoint++;
            } else {
                LOG << "Watering task not completed!" << std::endl;
            }
        } else if(m_cTargetLight->GetColor() == CColor::YELLOW && m_sAllocations.task == NOURISH_TASK) {
            LOG << "Found sick (yellow) plant at " << "(" << m_cTargetLight->GetPosition() << ")" << std::endl;
            if(m_sTaskCompleted.Rand()) {
                m_unWaypoint++;
            } else {
                LOG << "Nourishing task not completed!" << std::endl;
            }
        } else if(m_cTargetLight->GetColor() == CColor::RED && m_sAllocations.task == TREATMENT_TASK) {
            LOG << "Found dry (red) plant at " << "(" << m_cTargetLight->GetPosition() << ")" << std::endl;
            if(m_sTaskCompleted.Rand()) {
                m_unWaypoint++;
            } else {
                LOG << "Treatment task not completed!" << std::endl;
            }
        }
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

        if(task_id > m_pTasks.size() - 1) {
            // Reset index if greater than the number of tasks available
            task_id = 0;
        }
        // Set controller task variable.
        CEyeBotPso& cController = dynamic_cast<CEyeBotPso&>(cEyeBotEnt->GetControllableEntity().GetController());
        cController.m_sAllocations.task = m_pTasks[task_id];
        m_mTaskedEyeBots[cEyeBotEnt->GetId()] = m_pTasks[task_id];
    }

    LOG << "Tasked eyebot map: " << std::endl;
    for (std::map<std::string, ETask>::const_iterator iter = m_mTaskedEyeBots.begin(); iter != m_mTaskedEyeBots.end(); iter++)
    {
        LOG << "Robot Id: " << iter->first << " " << "Task:" << iter->second << std::endl;
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

void CEyeBotPso::SQuadLaunchParams::Init(TConfigurationNode& t_node) {
    try {
        Real p_val;

        GetNodeAttribute(t_node, "altitude", p_val);
        altitude = p_val;
        GetNodeAttribute(t_node, "reach", p_val);
        reach = p_val;
        GetNodeAttribute(t_node, "proximity_tolerance", p_val);
        proximity_tolerance = p_val;
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing quadcopter launch parameters.", ex);
    }
}

void CEyeBotPso::SWaypointParams::Init(TConfigurationNode& t_node) {
    try {
        double param_val;
        bool param_bool;

        GetNodeAttribute(t_node, "z_assess", param_val);
        z_assess = param_val;
        GetNodeAttribute(t_node, "ns_mean", param_val);
        ns_mean = param_val;
        GetNodeAttribute(t_node, "ns_stddev", param_val);
        ns_stddev = param_val;
        GetNodeAttribute(t_node, "naive_mapping", param_bool);
        naive_mapping = param_bool;
        GetNodeAttribute(t_node, "add_origin", param_bool);
        add_origin = param_bool;
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

void CEyeBotPso::SEyeBotTask::Init(double& global_reach) {
    switch(task) {
        case EVALUATE_TASK:
            reach = global_reach * 1.5;
            break;
        case WATER_TASK:
            reach = global_reach * 1.1;
            break;
        case NOURISH_TASK:
            reach = global_reach * 0.7;
            break;
        case TREATMENT_TASK:
            reach = global_reach * 0.3;
            break;
        default:
            break;
    }
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