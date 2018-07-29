#ifndef EYEBOT_ACO_H
#define EYEBOT_ACO_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the quadrotor positioning actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_position_actuator.h>
/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definition of the range-and-bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the eye-bot proximity sensor */
#include <argos3/plugins/robots/eye-bot/control_interface/ci_eyebot_proximity_sensor.h>
/* Definition of the perspective camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_perspective_camera_sensor.h>
/* Definition of the argos space */
#include <argos3/core/simulator/space/space.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CEyeBotAco : public CCI_Controller {

public:
    /* Class constructor. */
    CEyeBotAco();

    /* Class destructor. */
    virtual ~CEyeBotAco() {}

    /*
    * The following variables are used as parameters for
    * turning during navigation. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_flocking_controller><parameters><plant_targets>
    * section.
    */

    virtual void Init(TConfigurationNode& t_node);
    /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
    virtual void ControlStep();

    /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    */
    virtual void Reset();

    /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

    /*
    * The plants layout structure.
    */
   struct SPlantTargetsParams {
       CVector3 Center;
       CVector3 Distances;
       CVector3 Layout;
       Real Quantity;

       void Init(TConfigurationNode& t_node);
    };

private:
    /*
    * Takes off the robot.
    */
    void TakeOff();

    /*
    * Lands the robot.
    */
    void Land();

    /*
    * Move robot to target.
    */
    void WaypointAdvance();

    /*
    * Calculate (naively or via camera vision) the locations of each
    * plant target.
    */
    void MapArena(bool naive);

private:

    /* Current robot state */
    enum EState {
        STATE_START = 0,
        STATE_TAKE_OFF,
        STATE_ADVANCE,
        STATE_LAND
    };

private:

    /* Pointer to the quadrotor position actuator */
    CCI_QuadRotorPositionActuator* m_pcPosAct;
    /* Pointer to the positioning sensor */
    CCI_PositioningSensor* m_pcPosSens;
    /* Pointer to the eye-bot proximity sensor */
    CCI_EyeBotProximitySensor* m_pcProximity;
    /* Pointer to the perspective camera sensor */
    CCI_ColoredBlobPerspectiveCameraSensor* m_pcCamera;
    /* Pointer to the space */
    CSpace* m_pcSpace;

    /* Current robot state */
    EState m_eState;
    /* Current target position */
    CVector3 m_cTargetPos;
    /* Target locations */
    std::vector<std::vector<double>> m_cPlantLocList;
    /* Used to move the robot along the aco trajectory */
    UInt32 m_unWaypoint;

    /* Plant target parameters */
    SPlantTargetsParams m_sPlantTargetParams;
};

#endif