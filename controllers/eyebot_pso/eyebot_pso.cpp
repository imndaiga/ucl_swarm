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

/* Tolerance for the distance to a target point to decide to do something else */
static const Real PROXIMITY_TOLERANCE = 0.00001f;

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

void CEyeBotPso::ControlStep() {;

    switch(m_eState) {
        case STATE_START:
            TakeOff();
            break;
        case STATE_TAKE_OFF:
            TakeOff();
            break;
        case STATE_LOCALIZE_TARGETS:
            LocalizeTargets();
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
        if(Distance(m_cTargetPos, m_pcPosSens->GetReading().Position) < PROXIMITY_TOLERANCE) {
            /* State transition */
            LocalizeTargets();
        }
    }
}

/****************************************/
/****************************************/

void CEyeBotPso::LocalizeTargets() {
    if(m_eState != STATE_LOCALIZE_TARGETS) {
        /* State initialization */
        m_eState = STATE_LOCALIZE_TARGETS;

        /* Go through the camera readings to calculate plant direction vectors */
        enumTargetReadings(CColor::GREEN);
        LOG << "Number of "<< CColor::GREEN <<" targets found: " << m_psPCMsg.size() << std::endl;

        if(m_psPCMsg.size() > 0) {
            LOG << "Approaching first target at " << m_psPCMsg[0] << std::endl;
            m_cTargetPos = CVector3(0.0, 0.9, 1.0);
            m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
        }
    } else {
        if(Distance(m_cTargetPos, m_pcPosSens->GetReading().Position) < PROXIMITY_TOLERANCE) {
            /* State transition */
        }
    }
}

void CEyeBotPso::enumTargetReadings(CColor BlobColor) {
    /* Get the camera readings */
    CCI_ColoredBlobPerspectiveCameraSensor::SReadings tmpReading = m_pcCamera->GetReadings();

    if(! tmpReading.BlobList.empty()) {

        for(size_t i = 0; i < tmpReading.BlobList.size(); ++i) {
            /*
            * The camera perceives the light as a green blob
            * So, consider only green blobs representing plants
            * identified using cv techniques.
            */
            if(tmpReading.BlobList[i]->Color == BlobColor) {
                /*
                * Take the blob x,y distance
                * With the distance, calculate the global position of each plant
                */
                if(i >= m_psPCMsg.size()) {
                    bool precapTarget = false;
                    for(size_t b=0; b < m_psPCMsg.size(); b++) {
                        /* Ensure that the reading is not a duplicate */
                        if((tmpReading.BlobList[i]->X == m_psPCMsg[b].GetX()) && (tmpReading.BlobList[i]->Y == m_psPCMsg[b].GetY())) {
                            precapTarget = true;
                        }
                    }
                    if(!precapTarget) {
                        /* Calculate the global coordinates of each newly registered target
                        * Arena in .argos config is in meters, distances returned by sensors are in centimeters.
                        */
                        LOG << "target: (" << tmpReading.BlobList[i]->X/100. << "," << tmpReading.BlobList[i]->Y/100. << ") quad: (" << m_pcPosSens->GetReading().Position.GetX() << "," << m_pcPosSens->GetReading().Position.GetY() << ")" << std::endl;
                        m_psPCMsg.push_back(CVector2(tmpReading.BlobList[i]->X/100. - m_pcPosSens->GetReading().Position.GetX(), tmpReading.BlobList[i]->Y/100. - m_pcPosSens->GetReading().Position.GetY()));
                    }
                }
            }
        }
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