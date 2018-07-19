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
static const Real ALTITUDE = 3.0f;

/* Distance to wall to move along the Pso at */
static const Real REACH = 3.0f;

/****************************************/
/****************************************/

CEyeBotPso::CEyeBotPso() :
    m_pcPosAct(NULL),
    m_pcPosSens(NULL),
    m_pcProximity(NULL),
    m_pcCamera(NULL),
    m_pcRABSens(NULL) {}

/****************************************/
/****************************************/

void CEyeBotPso::Init(TConfigurationNode& t_node) {

    m_pcPosAct    = GetActuator <CCI_QuadRotorPositionActuator             >("quadrotor_position");
    m_pcPosSens   = GetSensor   <CCI_PositioningSensor                     >("positioning"       );
    m_pcRABSens   = GetSensor   <CCI_RangeAndBearingSensor                 >("range_and_bearing" );
    m_pcProximity = GetSensor   <CCI_EyeBotProximitySensor                 >("eyebot_proximity"  );
    m_pcCamera    = GetSensor   <CCI_ColoredBlobPerspectiveCameraSensor    >("colored_blob_perspective_camera");

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
    LOG << "Shortest Path: " << eyebotPsoSwarm.best_position.to_string() << std::endl;

    /* Enable camera filtering */
    m_pcCamera->Enable();
    Reset();
}

/****************************************/
/****************************************/

void CEyeBotPso::ControlStep() {
    /* Get the camera readings */
    const CCI_ColoredBlobPerspectiveCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();
    /* Go through the camera readings to calculate plant direction vectors */

    if(! sReadings.BlobList.empty()) {
        CVector2 cAccum;
        size_t unBlobsSeen = 0;
        for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {
            /*
            * The camera perceives the light as a green blob
            * So, consider only red blobs
            */
           if(sReadings.BlobList[i]->Color == CColor::GREEN) {
                /*
                * Take the blob distance and angle
                * With the distance, calculate the global position of each plant
                */
               LOG << "Found plant at (" << sReadings.BlobList[i]->X << "," << sReadings.BlobList[i]->Y << ")";
               ++unBlobsSeen;
           }
        }
        LOG << std::endl;
    }
    switch(m_eState) {
        case STATE_START:
            TakeOff();
            break;
        case STATE_TAKE_OFF:
            TakeOff();
            break;
        case STATE_LAND:
            Land();
            break;
        default:
            LOGERR << "[BUG] Unknown robot state: " << m_eState << std::endl;
    }
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
        m_cTargetPos = m_pcPosSens->GetReading().Position + CVector3(0.0f, REACH, ALTITUDE);
        m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
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