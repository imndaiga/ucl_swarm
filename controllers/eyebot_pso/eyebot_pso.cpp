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

void CEyeBotPso::SPlantTargetsParams::Init(TConfigurationNode& t_node) {
    try {
        CVector3 pLocation;
        GetNodeAttribute(t_node, "center", pLocation);
        Center = pLocation;
        GetNodeAttribute(t_node, "distances", pLocation);
        Distances = pLocation;
        GetNodeAttribute(t_node, "layout", pLocation);
        Layout = pLocation;
        GetNodeAttribute(t_node, "quantity", Quantity);
    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing plant target parameters.", ex);
    }
}

/****************************************/
/****************************************/

/* Altitude to Pso to move along the Pso */
static const Real ALTITUDE = 3.0f;

/* Distance to wall to move along the Pso at */
static const Real REACH = 3.0f;

/* Tolerance for the distance to a target point to decide to do something else */
static const Real PROXIMITY_TOLERANCE = 0.01f;

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

    /*
    * Parse the config file
    */
    try {
        /* Plant parameters */
        m_sPlantTargetParams.Init(GetNode(t_node, "plant_targets"));
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

    Swarm eyebotPsoSwarm(particle_count, self_trust, past_trust, global_trust);
    eyebotPsoSwarm.load_test();
    distance = eyebotPsoSwarm.solve();

    LOG << "PSO Distance: " << distance << " Target Distance: " << test_distance_target << std::endl;
    LOG << "Shortest Path: " << eyebotPsoSwarm.best_position.to_string() << std::endl;
    LOG << "Plant target params: " << std::endl;
    LOG << "{ Center : " << m_sPlantTargetParams.Center << " }" << std::endl;
    LOG << "{ Distances : " << m_sPlantTargetParams.Distances << " }" << std::endl;
    LOG << "{ Layout : " << m_sPlantTargetParams.Layout << " }" << std::endl;
    LOG << "{ Quantity : " << m_sPlantTargetParams.Quantity << " }" << std::endl;

    computeLocalisation();
    LOG << "Target locations computed as: " << std::endl;
    for(size_t t=0; t < m_sPlantTargetParams.Quantity; t++) {
        LOG << m_cPlantLocList[t] << std::endl;
    }
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

/****************************************/
/****************************************/

void CEyeBotPso::computeLocalisation() {

    double width = ( m_sPlantTargetParams.Layout.GetX() * m_sPlantTargetParams.Distances.GetX() ) - 0.5;
    double height = ( m_sPlantTargetParams.Layout.GetZ() * m_sPlantTargetParams.Distances.GetZ() ) - 0.5;
    CVector3 currLoc = CVector3(m_sPlantTargetParams.Center.GetX() - width/2.0, m_sPlantTargetParams.Center.GetY(), m_sPlantTargetParams.Center.GetZ() - height/2.0);

    for(size_t t=0; t < m_sPlantTargetParams.Quantity; t++) {
        m_cPlantLocList.push_back(currLoc);

        if( t == m_sPlantTargetParams.Layout.GetX() - 1 ) {
            currLoc += CVector3(0.0, 0.0, m_sPlantTargetParams.Distances.GetZ());
        } else if( t < m_sPlantTargetParams.Layout.GetX() - 1) {
            currLoc += CVector3(m_sPlantTargetParams.Distances.GetX(), 0.0, 0.0);
        } else if(t > m_sPlantTargetParams.Layout.GetX() - 1) {
            currLoc -= CVector3(m_sPlantTargetParams.Distances.GetX(), 0.0, 0.0);
        }
    }
}

REGISTER_CONTROLLER(CEyeBotPso, "eyebot_pso_controller")