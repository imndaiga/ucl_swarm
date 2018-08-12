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

/****************************************/
/****************************************/

/* Altitude to Lawn to move along the lawn */
static const Real ALTITUDE = 0.1f;

/* Distance to wall to move along the lawn at */
static const Real REACH = 3.0f;

/* Distance to move horizontally along the lawn length at */
static const Real HORIZONTAL_STEP = 0.05f;

/* Distance to move vertically along the lawn length at */
static const Real VERTICAL_STEP = 0.05f;

/* Tolerance for the distance to a target point to decide to do something else */
static const Real PROXIMITY_TOLERANCE = 0.00001f;

/* How many points the robot traverses to move horizontally along the lawn */
static const UInt32 LAWN_VERTICAL_WAYPOINTS = 5;

/* Direction in which the robot traverses to move horizontally along the lawn */
Real LAWN_DIRECTION = 1.0f;

/****************************************/
/****************************************/

CEyeBotLawn::CEyeBotLawn() :
    m_pcPosAct(NULL),
    m_pcPosSens(NULL),
    m_pcProximity(NULL),
    m_pcRABSens(NULL),
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
    m_pcRABSens   = GetSensor   <CCI_RangeAndBearingSensor    >("range_and_bearing" );
    m_pcProximity = GetSensor <CCI_EyeBotProximitySensor    >("eyebot_proximity"  );
    m_pcSpace     = &CSimulator::GetInstance().GetSpace();

    /*
    * Parse the config file
    */
    try {
        /* Get quadcopter launch parameters */
        m_sStateData.Init(GetNode(t_node, "state"));
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

   /*
    * Initialize the state variables of the behavior
    */
    Reset();
}

/****************************************/
/****************************************/

void CEyeBotLawn::ControlStep() {
   /* Get RAB message, if any */
    RLOG << "Message received: ";
    if(! m_pcRABSens->GetReadings().empty()) {
        m_psFBMsg = &(m_pcRABSens->GetReadings()[0]);
        LOG << *reinterpret_cast<const UInt64*>(m_psFBMsg->Data.ToCArray());
    }
    else {
        m_psFBMsg = NULL;
        LOG << "none";
    }
    LOG << std::endl;
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
        m_cTargetPos = m_pcPosSens->GetReading().Position + CVector3(0.0f, REACH, ALTITUDE);
        m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
    } else {
        if(Distance(m_cTargetPos, m_pcPosSens->GetReading().Position) < PROXIMITY_TOLERANCE) {
            /* State transition */
            Move();
        }
    }
}

void CEyeBotLawn::Land() {
    if(m_sStateData.State != SStateData::STATE_LAND) {
        /* State initialization */
        m_sStateData.State = SStateData::STATE_LAND;
        m_cTargetPos = m_pcPosSens->GetReading().Position;
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

            if(Distance(m_cTargetPos, m_pcPosSens->GetReading().Position) < PROXIMITY_TOLERANCE) {
                /* State transition */
                UpdateWaypoint();
            }
        } else {
            /* State transition */
            // Waypoints is empty or in error, go to land state.
            RLOG << "No waypoints available. Resting now." << std::endl;
            Land();
        }
    }
}

void CEyeBotLawn::MapWall() {
    CSpace::TMapPerType boxes = m_pcSpace->GetEntitiesByType("box");
    CBoxEntity* Wall = any_cast<CBoxEntity*>(boxes["wall_north"]);
    CVector3 WallSize = Wall->GetSize();
    std::vector< std::vector<double> > Unsorted, Sorted;

    // Variables to control the unit steps in both x and z directions.
    double x_increment = 1.0;
    double z_increment = 1.0;
    double z_i = 0.0;
    // Ensure that alternate waypoint levels are flipped to generate
    // a lawn path motion.
    bool flipFlag = false;

    while(z_i < WallSize.GetZ()) {
        double x_i = WallSize.GetX()/2.0;

        while(x_i > -WallSize.GetX()/2.0) {
            std::vector<double> wp;
            wp.push_back(x_i);
            wp.push_back(0.0);
            wp.push_back(z_i);

            Unsorted.push_back(wp);
            x_i -= x_increment;
        }

        if(flipFlag) {
            // Reverse new waypoint level.
            std::reverse(Unsorted.begin(), Unsorted.end());
            flipFlag = false;
        }

        for(size_t i = 0; i < Unsorted.size(); i++ ) {
            // Store waypoint level as sorted.
            Sorted.push_back( Unsorted[i] );
        }

        z_i += z_increment;
        flipFlag = true;
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

/****************************************/
/****************************************/

void CEyeBotLawn::SStateData::Init(TConfigurationNode& t_node) {
    try {
        GetNodeAttribute(t_node, "global_reach", Reach);
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