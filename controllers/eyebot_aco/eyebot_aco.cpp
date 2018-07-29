/* Include the controller definition */
#include "eyebot_aco.h"
/* Include the aco swarm algorithm definition */
#include <algorithms/aco/swarm.h>
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* Function definitions for logging */
#include <argos3/core/utility/logging/argos_log.h>
/* Definition of the argos space */
#include <argos3/core/simulator/space/space.h>
/* Definition of the argos simulator */
#include <argos3/core/simulator/simulator.h>
/* Definition of the argos entities */
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <string>

/****************************************/
/****************************************/

int n_ants = 10;
long int seed = 123;
tsp_sol swarm_sol;

/****************************************/
/****************************************/

CEyeBotAco::CEyeBotAco() :
    m_pcPosAct(NULL),
    m_pcPosSens(NULL),
    m_pcProximity(NULL),
    m_pcCamera(NULL),
    m_pcSpace(NULL) {}

/****************************************/
/****************************************/

void CEyeBotAco::SPlantTargetsParams::Init(TConfigurationNode& t_node) {
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

/* Altitude to move along the Aco path */
static const Real ALTITUDE = 0.1f;

/* Distance to wall to move along the wall at */
static const Real REACH = 3.0f;

/* Tolerance for the distance to a target point to decide to do something else */
static const Real PROXIMITY_TOLERANCE = 0.01f;

/****************************************/
/****************************************/

void CEyeBotAco::Init(TConfigurationNode& t_node) {

    m_pcPosAct    = GetActuator <CCI_QuadRotorPositionActuator             >("quadrotor_position");
    m_pcPosSens   = GetSensor   <CCI_PositioningSensor                     >("positioning"       );
    m_pcProximity = GetSensor   <CCI_EyeBotProximitySensor                 >("eyebot_proximity"  );
    m_pcCamera    = GetSensor   <CCI_ColoredBlobPerspectiveCameraSensor    >("colored_blob_perspective_camera");
    m_pcSpace     = &CSimulator::GetInstance().GetSpace();

    /*
    * Parse the config file
    */
    try {
        /* Get plant parameters */
        m_sPlantTargetParams.Init(GetNode(t_node, "plant_targets"));
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

    /* Map targets in the arena: this can be done naively
    * with the passed argos parameters or with the help
    * of the camera sensor.
    */
    MapArena(true);
    /* Enable camera filtering */
    m_pcCamera->Enable();
    Reset();
}

/****************************************/
/****************************************/

void CEyeBotAco::ControlStep() {
    switch(m_eState) {
        case STATE_START:
            TakeOff();
            break;
        case STATE_TAKE_OFF:
            TakeOff();
            break;
        case STATE_ADVANCE:
            WaypointAdvance();
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
    RLOG << "Waypoint: " << m_unWaypoint << std::endl;
}

/****************************************/
/****************************************/

void CEyeBotAco::Reset() {
    /* Start the behavior */
    m_eState = STATE_START;
}

/****************************************/
/****************************************/

void CEyeBotAco::TakeOff() {
    if(m_eState != STATE_TAKE_OFF) {
        /* State initialization */
        m_eState = STATE_TAKE_OFF;
        m_cTargetPos = m_pcPosSens->GetReading().Position + CVector3(0.0f, REACH, ALTITUDE);
        m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
    } else {
        if(Distance(m_cTargetPos, m_pcPosSens->GetReading().Position) < PROXIMITY_TOLERANCE) {
            /* State transition */
            WaypointAdvance();
        }
    }
}

/****************************************/
/****************************************/

void CEyeBotAco::Land() {
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
void CEyeBotAco::WaypointAdvance() {
    if(m_eState != STATE_ADVANCE) {
        /* State initialization */
        m_eState = STATE_ADVANCE;
        m_unWaypoint = 0;
    } else {
        if(m_cPlantLocList.size() > 0 && m_unWaypoint != m_cPlantLocList.size()) {
            m_cTargetPos = CVector3(m_cPlantLocList[m_unWaypoint][0], m_cPlantLocList[m_unWaypoint][1] - REACH, m_cPlantLocList[m_unWaypoint][2]);
            m_pcPosAct->SetAbsolutePosition(m_cTargetPos);

            if(Distance(m_cTargetPos, m_pcPosSens->GetReading().Position) < PROXIMITY_TOLERANCE) {
                m_unWaypoint++;
            }
        } else if (m_unWaypoint == m_cPlantLocList.size()) {
            /* State transition */
            Land();
        } else if(m_cPlantLocList.size() == 0) {
            LOG << "No targets have been identified." << std::endl;
        }
    }
}

/****************************************/
/****************************************/

void CEyeBotAco::MapArena(bool naive) {
    CVector2 targetLoc;

    if(naive) {
        CSpace::TMapPerType boxes = m_pcSpace->GetEntitiesByType("box");
        CSpace::TMapPerType lights = m_pcSpace->GetEntitiesByType("light");
        
        /* Retrieve the wall object in the arena*/
        CBoxEntity* wall = any_cast<CBoxEntity*>(boxes["wall_north"]);

        /* Retrieve and store the positions of each light in the arena */
        for(size_t l=0; l < lights.size(); l++) {
            std::vector<double> l_vec;
            std::string l_ind = "light" + std::to_string(l);
            CLightEntity* light = any_cast<CLightEntity*>(lights[l_ind]);

            l_vec.push_back(light->GetPosition().GetX());
            l_vec.push_back(light->GetPosition().GetY());
            l_vec.push_back(light->GetPosition().GetZ());
            m_cPlantLocList.push_back(l_vec);
        }
    } else {
        /* Implement target seeking here. Would require SFM abilities? */
        
        m_cTargetPos = m_pcPosSens->GetReading().Position;
        m_cTargetPos.SetZ(3.0f);
        m_pcPosAct->SetAbsolutePosition(m_cTargetPos);

        if(Distance(m_cTargetPos, m_pcPosSens->GetReading().Position) < PROXIMITY_TOLERANCE) {
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
                        * Take the blob distance
                        * With the distance, calculate the global position of each plant
                        */
                        targetLoc = CVector2(sReadings.BlobList[i]->X, sReadings.BlobList[i]->Y);
                        LOG << "Found plant at " << targetLoc << ")";
                        ++unBlobsSeen;
                    }
                }
                LOG << std::endl;
            }
        }

        m_cTargetPos = m_pcPosSens->GetReading().Position;
        m_cTargetPos.SetZ(ALTITUDE);
        m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
    }

    LOG << "Simulator-extracted target locations: " << std::endl;
    for(size_t t=0; t < m_sPlantTargetParams.Quantity; t++) {
        LOG << m_cPlantLocList[t][0] << ", " << m_cPlantLocList[t][1] << ", " << m_cPlantLocList[t][2] << std::endl;
    }

    Swarm swarm(n_ants, m_cPlantLocList, seed, "cm");
    tsp_sol swarm_sol;
    swarm_sol = swarm.optimize();

    LOG << "Aco Tour Distance: " << swarm_sol.tour_length << std::endl;
    LOG << "Shortest Path: ";
    for(size_t n=0; n < swarm_sol.tour.size(); n++) {
        LOG << swarm_sol.tour[n] << " ";
    }
    LOG << std::endl;
    LOG << "Plant target params: " << std::endl;
    LOG << "{ Center : " << m_sPlantTargetParams.Center << " }" << std::endl;
    LOG << "{ Distances : " << m_sPlantTargetParams.Distances << " }" << std::endl;
    LOG << "{ Layout : " << m_sPlantTargetParams.Layout << " }" << std::endl;
    LOG << "{ Quantity : " << m_sPlantTargetParams.Quantity << " }" << std::endl;
}

REGISTER_CONTROLLER(CEyeBotAco, "eyebot_aco_controller")