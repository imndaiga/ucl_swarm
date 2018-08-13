/* Include the controller definition */
#include "eyebot_lawn.h"
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
/* Include necessary standard library definitions */
#include <random>

/****************************************/
/****************************************/

CEyeBotLawn::CEyeBotLawn() :
    m_pcPosAct(NULL),
    m_pcPosSens(NULL),
    m_pcProximity(NULL),
    m_pcSpace(NULL) {}

/****************************************/
/****************************************/

void CEyeBotLawn::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "quadrotor_position") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><eyebot_lawn><actuators> and
    * <controllers><eyebot_lawn><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
    m_pcPosAct    = GetActuator <CCI_QuadRotorPositionActuator>("quadrotor_position");
    m_pcPosSens   = GetSensor   <CCI_PositioningSensor        >("positioning"       );
    m_pcProximity = GetSensor <CCI_EyeBotProximitySensor      >("eyebot_proximity"  );
    m_pcSpace     = &CSimulator::GetInstance().GetSpace();

    m_cNearestTarget = new CLightEntity;
    kf               = new KalmanFilter(m_sKalmanFilter.dt, m_sKalmanFilter.A, m_sKalmanFilter.C, m_sKalmanFilter.Q, m_sKalmanFilter.R, m_sKalmanFilter.P);

    /*
    * Parse the config file
    */
    try {
        /* Get quadcopter launch parameters */
        m_sStateData.Init(GetNode(t_node, "state"));
        /* Get waypoint parameters */
        m_sWaypointParams.Init(GetNode(t_node, "waypoints"));
        /* Get random parameters */
        m_sRandGen.Init(GetNode(t_node, "random"));
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

    /*
    * Initialize the kalman filter.
    */
    UpdatePosition(m_pcPosSens->GetReading().Position);

    HomePos = GetPosition();

    /*
    * Initialize the state variables of the behavior
    */
    Reset();
}

/****************************************/
/****************************************/

void CEyeBotLawn::ControlStep() {
    UpdatePosition();
    UpdateNearestTarget();

    /* Execute state logic */
    switch(m_sStateData.State) {
        case SStateData::STATE_START:
            TakeOff();
            break;
        case SStateData::STATE_TAKE_OFF:
            TakeOff();
            break;
        case SStateData::STATE_MOVE:
            Move();
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
}

void CEyeBotLawn::Reset() {
    /* Start the behavior */
    m_sStateData.Reset();
    /* No message received */
    m_psFBMsg = NULL;
    MapWall();
}

void CEyeBotLawn::TakeOff() {
    if(m_sStateData.State != SStateData::STATE_TAKE_OFF) {
        /* State initialization */
        m_sStateData.State = SStateData::STATE_TAKE_OFF;
        m_cTargetPos = GetPosition() + CVector3(0.0f, m_sStateData.Reach, m_sStateData.launch_altitude);
        m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
    } else {
        if(Distance(m_cTargetPos, GetPosition()) < m_sStateData.proximity_tolerance) {
            /* State transition */
            Move();
        }
    }
}

void CEyeBotLawn::Land() {
    if(m_sStateData.State != SStateData::STATE_LAND) {
        /* State initialization */
        m_sStateData.State = SStateData::STATE_LAND;
        m_cTargetPos = GetPosition();
        m_cTargetPos.SetZ(0.0f);
        m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
    }
}

/****************************************/
/****************************************/

void CEyeBotLawn::Move() {
    if(m_sStateData.State != SStateData::STATE_MOVE) {
        /* State initialization */
        m_sStateData.State = SStateData::STATE_MOVE;
    } else {
        if(m_sStateData.WaypointIndex < m_sStateData.WaypointMap.size()) {
            std::vector<double> target_wp = GetWaypoint();
            m_cTargetPos = CVector3(target_wp[0], target_wp[1], target_wp[2]);
            m_pcPosAct->SetAbsolutePosition(m_cTargetPos);

            if(Distance(m_cTargetPos, GetPosition()) < m_sStateData.proximity_tolerance) {
                /* State transition */
                ExecuteTask();
            }
        } else {
            /* State transition */
            // Waypoints is empty or in error, go to land state.
            RLOG << "No waypoints available. Resting now." << std::endl;
            Land();
        }
    }
}

void CEyeBotLawn::ExecuteTask() {

    if(m_sStateData.State != SStateData::STATE_EXECUTE_TASK) {
        /* State initialization */
        m_sStateData.State = SStateData::STATE_EXECUTE_TASK;
    } else {
        /* State logic */
        RLOG << "Executing task." << std::endl;

        // Initialize evaluation at current target.
        EvaluateFunction();

        /* Execute task logic at current target */
        switch(m_sStateData.TargetTask) {
            case SStateData::TASK_EVALUATE :
                EvaluateFunction();
                break;
            case SStateData::TASK_WATER :
                WaterFunction();
                break;
            case SStateData::TASK_NOURISH :
                NourishFunction();
                break;
            case SStateData::TASK_TREATMENT :
                TreatmentFunction();
                break;
            case SStateData::TASK_NULL :
                UpdateWaypoint();
                break;
            default:
                LOGERR << "[BUG] Invalid task state: " << m_sStateData.TargetTask << std::endl;
        }

        // State transition
        Move();
    }
}

void CEyeBotLawn::MapWall() {
    CSpace::TMapPerType boxes = m_pcSpace->GetEntitiesByType("box");
    CBoxEntity* Wall = any_cast<CBoxEntity*>(boxes["wall_north"]);
    CVector3 WallSize = Wall->GetSize();
    std::vector< std::vector<double> > Unsorted, Sorted;

    double z_i = 0.0;
    int flipCount = 1;

    while(z_i < WallSize.GetZ()) {
        double x_i = WallSize.GetX()/2.0;

        while(x_i > -WallSize.GetX()/2.0) {
            std::vector<double> wp;
            wp.push_back(x_i + m_sRandGen.mapping.get());
            wp.push_back(0.0f + m_sRandGen.mapping.get());
            wp.push_back(z_i + m_sRandGen.mapping.get());

            Unsorted.push_back(wp);
            x_i -= m_sWaypointParams.hstep;
        }

        if(flipCount % 2 == 0) {
            // Ensure that alternate waypoint levels are flipped to generate
            // a lawn path motion.
            std::reverse(Unsorted.begin(), Unsorted.end());
        }

        for(size_t i = 0; i < Unsorted.size(); i++ ) {
            // Store waypoint level as sorted.
            Sorted.push_back( Unsorted[i] );
        }

        z_i += m_sWaypointParams.vstep;
        flipCount++;
        Unsorted.clear();
    }

    for(size_t i = 0; i < Sorted.size(); i++) {
        // Store sorted waypoint into WaypointMap.
        m_sStateData.WaypointMap[i] = Sorted[i];
    }

    RLOG << "Wall size: " << WallSize << std::endl;
    RLOG << "WaypointMap size: " << m_sStateData.WaypointMap.size() << std::endl;
    RLOG << "Waypoints: " << std::endl;
    for(size_t wp = 0; wp < m_sStateData.WaypointMap.size(); wp++) {
        LOG << "( ";
        for (size_t wp_reading = 0; wp_reading < m_sStateData.WaypointMap[wp].size(); wp_reading++) {
            LOG << m_sStateData.WaypointMap[wp][wp_reading] << " ";
        }
        LOG << ")" << std::endl;
    }
}

void CEyeBotLawn::UpdatePosition(CVector3 x0) {
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

void CEyeBotLawn::UpdateNearestTarget() {
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

/****************************************/
/****************************************/

void CEyeBotLawn::EvaluateFunction() {

    if(m_cNearestTarget->GetColor() == CColor::WHITE) {
        RLOG << "Found untagged (white/grey) plant at " << "(" << m_cNearestTarget->GetPosition() << ")" << std::endl;
        // Probabilistically assign target state.
        CColor TargetColor = m_pTargetStates[m_sRandGen.targetshuffle.get()];
        LOG << m_sRandGen.targetshuffle.get() << std::endl;
        m_cNearestTarget->SetColor(TargetColor);
    }

    RLOG << "Processing...";
    if(m_cNearestTarget->GetColor() == CColor::WHITE) {
        LOG << "found retagged (white/grey) plant at " << "(" << m_cNearestTarget->GetPosition() << ")";
        m_sStateData.TargetTask = SStateData::TASK_EVALUATE;
        UpdateWaypoint();
    } else if(m_cNearestTarget->GetColor() == CColor::GREEN) {
        LOG << "found healthy (green) plant at " << "(" << m_cNearestTarget->GetPosition() << ")";
        m_sStateData.TargetTask = SStateData::TASK_NULL;
    } else if(m_cNearestTarget->GetColor() == CColor::BROWN) {
        LOG << "found dry (brown) plant at " << "(" << m_cNearestTarget->GetPosition() << ")";
        m_sStateData.TargetTask = SStateData::TASK_WATER;
    } else if(m_cNearestTarget->GetColor() == CColor::YELLOW) {
        LOG << "found malnourished (yellow) plant at " << "(" << m_cNearestTarget->GetPosition() << ")";
        m_sStateData.TargetTask = SStateData::TASK_NOURISH;
    } else if(m_cNearestTarget->GetColor() == CColor::RED) {
        LOG << "found sick (red) plant at " << "(" << m_cNearestTarget->GetPosition() << ")";
        m_sStateData.TargetTask = SStateData::TASK_TREATMENT;
    }
}

void CEyeBotLawn::WaterFunction() {
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

void CEyeBotLawn::NourishFunction() {
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

void CEyeBotLawn::TreatmentFunction() {
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

void CEyeBotLawn::SWaypointParams::Init(TConfigurationNode& t_node) {
    try {
        GetNodeAttribute(t_node, "hstep", hstep);
        GetNodeAttribute(t_node, "vstep", vstep);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing waypoint parameters.", ex);
    }
}

void CEyeBotLawn::SRandomGen::Init(TConfigurationNode& t_node) {
    try {
        double mapping_mean, mapping_stddev;
        int mapping_seed, target_shuffle_seed;

        GetNodeAttribute(t_node, "mapping_mean", mapping_mean);
        GetNodeAttribute(t_node, "mapping_stddev", mapping_stddev);
        GetNodeAttribute(t_node, "mapping_seed", mapping_seed);
        GetNodeAttribute(t_node, "target_shuffle_seed", target_shuffle_seed);

        mapping.Init(mapping_mean, mapping_stddev, mapping_seed);
        // The following integers should be monitored and updated
        // from m_pcTargetStates.
        targetshuffle.Init(0, 4, target_shuffle_seed);
    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing random parameters.", ex);
    }
}

void CEyeBotLawn::SStateData::Init(TConfigurationNode& t_node) {
    try {
        GetNodeAttribute(t_node, "global_reach", Reach);
        GetNodeAttribute(t_node, "launch_altitude", launch_altitude);
        GetNodeAttribute(t_node, "proximity_tolerance", proximity_tolerance);
        GetNodeAttribute(t_node, "attitude", attitude);
        GetNodeAttribute(t_node, "minimum_hold_time", minimum_hold_time);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing state parameters.", ex);
    }
}

void CEyeBotLawn::SStateData::Reset() {
    State = SStateData::STATE_START;
    WaypointIndex = 0;
    HoldTime = 0;
    WaypointMap.clear();
}

CEyeBotLawn::SKF::SKF() {
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

void CEyeBotLawn::SGaussDist::Init(double& mean, double& stddev, int& gen_seed) {
    gen = new std::default_random_engine(gen_seed);
    nd = new std::normal_distribution<double>(mean, stddev);
}

void CEyeBotLawn::SUniformIntDist::Init(int min, int max, int& gen_seed) {
    gen = new std::default_random_engine(gen_seed);
    uid = new std::uniform_int_distribution<int>(min, max);
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CEyeBotLawn, "eyebot_lawn_controller")