/* Include the controller definition */
#include "eyebot_pso.h"
/* Include the pso swarm algorithm definition */
#include <pso/swarm.h>
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* Function definitions for logging */
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

/* Altitude to Pso to move along the Pso */
static const Real ALTITUDE = 0.1f;

/* Distance to wall to move along the Pso at */
static const Real REACH = 3.0f;

/****************************************/
/****************************************/

CEyeBotPso::CEyeBotPso() :
    m_pcPosAct(NULL),
    m_pcPosSens(NULL),
    m_pcProximity(NULL),
    m_pcCamera(NULL),
    m_pcTags(NULL),
    m_pcRABSens(NULL) {}

/****************************************/
/****************************************/

void CEyeBotPso::Init(TConfigurationNode& t_node) {

    m_pcPosAct    = GetActuator <CCI_QuadRotorPositionActuator        >("quadrotor_position");
    m_pcPosSens   = GetSensor   <CCI_PositioningSensor                >("positioning"       );
    m_pcRABSens   = GetSensor   <CCI_RangeAndBearingSensor            >("range_and_bearing" );
    m_pcProximity = GetSensor   <CCI_EyeBotProximitySensor            >("eyebot_proximity"  );
    m_pcCamera    = GetSensor   <CCI_CameraSensor                     >("cameras"           );
    // m_pcTags      = GetId       <CCI_CameraSensorTagDetectorAlgorithm >("tag_detector"      );

    int particle_count = 20;
    double self_trust = 0.2;
    double past_trust = 0.1;
    double global_trust = 0.7;
    float test_distance_target = 86.63;
    double distance;

    Swarm eyebotPsoSwarm(particle_count, self_trust, past_trust, global_trust);
    eyebotPsoSwarm.load_test();
    distance = eyebotPsoSwarm.solve();

    LOG << "PSO Distance: " << distance << " Target Distance: " << test_distance_target << std::endl;
    LOG << "Shortest Path: " << eyebotPsoSwarm.best_position.to_string();

    Reset();
}

/****************************************/
/****************************************/

void CEyeBotPso::ControlStep() {
    // LOG << "Tag Reading: " << m_pcTags->GetReadings().Payload << std::endl;
}

/****************************************/
/****************************************/

void CEyeBotPso::Reset() {
    /* Start the behavior */
    m_eState = STATE_START;
    /* No message received */
    m_psFBMsg = NULL;
}

/****************************************/
/****************************************/

void CEyeBotPso::TakeOff() {
    if(m_eState != STATE_TAKE_OFF) {
        /* State initialization */
        m_eState = STATE_TAKE_OFF;
    } else {
        /* State transition */
    }
}

/****************************************/
/****************************************/

void CEyeBotPso::Land() {
    if(m_eState != STATE_LAND) {
        /* State initialization */
        m_eState = STATE_LAND;
        m_cTargetPos = m_pcPosSens->GetReading().Position;
        m_cTargetPos.SetZ(0.0f);
        m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
    }
}

REGISTER_CONTROLLER(CEyeBotPso, "eyebot_pso_controller")