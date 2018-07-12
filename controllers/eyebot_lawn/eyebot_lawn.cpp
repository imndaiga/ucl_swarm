/* Include the controller definition */
#include "eyebot_lawn.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* Function definitions for logging */
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

/* Altitude to Lawn to move along the Lawn */
static const Real ALTITUDE = 3.0f;

/****************************************/
/****************************************/

CEyeBotLawn::CEyeBotLawn() :
   m_pcPosAct(NULL),
   m_pcPosSens(NULL),
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
   m_pcPosAct  = GetActuator<CCI_QuadRotorPositionActuator>("quadrotor_position");
   m_pcPosSens = GetSensor  <CCI_PositioningSensor        >("positioning"       );
   m_pcRABSens = GetSensor  <CCI_RangeAndBearingSensor    >("range_and_bearing" );
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
   TakeOff();
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
    /* State initialization */
    m_eState = STATE_TAKE_OFF;
    m_cTargetPos = m_pcPosSens->GetReading().Position + CVector3(0.0f, 0.0f, ALTITUDE);
    m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
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