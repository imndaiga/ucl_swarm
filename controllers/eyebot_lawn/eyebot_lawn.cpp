/* Include the controller definition */
#include "eyebot_lawn.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* Function definitions for logging */
#include <argos3/core/utility/logging/argos_log.h>

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
static const Real PROXIMITY_TOLERANCE = 0.01f;

/* How many points the robot traverses to move horizontally along the lawn */
static const UInt32 LAWN_HORIZONTAL_WAYPOINTS = 50;

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
    m_pcRABSens(NULL) {}

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
    m_pcPosAct  = GetActuator <CCI_QuadRotorPositionActuator>("quadrotor_position");
    m_pcPosSens = GetSensor   <CCI_PositioningSensor        >("positioning"       );
    m_pcRABSens = GetSensor   <CCI_RangeAndBearingSensor    >("range_and_bearing" );
    m_pcProximity = GetSensor <CCI_EyeBotProximitySensor    >("eyebot_proximity"  );
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
    switch(m_eState) {
        case STATE_START:
            TakeOff();
            break;
        case STATE_TAKE_OFF:
            TakeOff();
            break;
        case STATE_MOVE_HORIZONTALLY:
            MoveHorizontally();
            break;
        case STATE_MOVE_VERTICALLY:
            MoveVertically();
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
}

/****************************************/
/****************************************/

void CEyeBotLawn::Reset() {
    /* Start the behavior */
    m_eState = STATE_START;
    /* No message received */
    m_psFBMsg = NULL;
}

/****************************************/
/****************************************/

void CEyeBotLawn::TakeOff() {
    if(m_eState != STATE_TAKE_OFF) {
        /* State initialization */
        m_eState = STATE_TAKE_OFF;
        m_cTargetPos = m_pcPosSens->GetReading().Position + CVector3(0.0f, REACH, ALTITUDE);
        m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
    } else {
        if(Distance(m_cTargetPos, m_pcPosSens->GetReading().Position) < PROXIMITY_TOLERANCE) {
            /* State transition */
            MoveHorizontally();
        }
    }
}

/****************************************/
/****************************************/

void CEyeBotLawn::Land() {
    if(m_eState != STATE_LAND) {
        /* State initialization */
        m_eState = STATE_LAND;
        m_cTargetPos = m_pcPosSens->GetReading().Position;
        m_cTargetPos.SetZ(0.0f);
        m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
    }
}

/****************************************/
/****************************************/

void CEyeBotLawn::MoveHorizontally() {
    if(m_eState != STATE_MOVE_HORIZONTALLY) {
        /* State initialization */
        m_eState = STATE_MOVE_HORIZONTALLY;
        LAWN_DIRECTION = LAWN_DIRECTION * -1.0;
    } else {
        if(m_unWaypoint >= LAWN_HORIZONTAL_WAYPOINTS) {
            /* State transition */
            m_unWaypoint = 0;
            MoveVertically();
        } else {
            /* State logic */
            m_cTargetPos = m_pcPosSens->GetReading().Position + CVector3((HORIZONTAL_STEP)*LAWN_DIRECTION, 0.0f, 0.0f);
            m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
            ++m_unWaypoint;
        }
    }
}

/****************************************/
/****************************************/

void CEyeBotLawn::MoveVertically() {
    if(m_eState != STATE_MOVE_VERTICALLY) {
        /* State initialization */
        m_eState = STATE_MOVE_VERTICALLY;
    } else {
        if(m_unWaypoint >= LAWN_VERTICAL_WAYPOINTS) {
            /* State transition */
            m_unWaypoint = 0;
            MoveHorizontally();
        } else {
            /* State logic */
            m_cTargetPos = m_pcPosSens->GetReading().Position + CVector3(0.0f, 0.0f, VERTICAL_STEP);
            m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
            ++m_unWaypoint;
        }
    }
    
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