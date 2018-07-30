#ifndef EYEBOT_PSO_H
#define EYEBOT_PSO_H

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
/* Definitions for the argos space */
#include <argos3/core/simulator/space/space.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CEyeBotPso : public CCI_Controller {

public:
    /* Class constructor. */
    CEyeBotPso();

    /* Class destructor. */
    virtual ~CEyeBotPso() {}

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
    * The swarm params.
    */
   struct SSwarmParams {
       int particles;
       double self_trust;
       double past_trust;
       double global_trust;

       void Init(TConfigurationNode& t_node);
    };
    /*
    * The quadcopter launch params.
    */
   struct SQuadLaunchParams {
       /* Altitude to Pso to move along the Pso */
       Real altitude;
       /* Distance to wall to move along the Pso at */
       Real reach;
       /* Tolerance for the distance to a target point to decide to do something else */
       Real proximity_tolerance;

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
    * Compute (naively or via camera vision) the locations of each
    * plant target.
    */
    void MapWaypoints(bool naive, bool add_origin);

    /* Compute (naively or via camera vision) the position, location and size of the wall */
    void MapWall(bool naive);

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

    /* Current robot state */
    EState m_eState;
    /* Current target position */
    CVector3 m_cTargetPos;
    /* Target locations */
    std::vector<std::vector<double>> m_cPlantLocList;
    /* Used to move the robot along the pso trajectory */
    UInt32 m_unWaypoint;

    /* Waypoint variables */
    typedef std::vector<double> wp_loc;
    std::vector<wp_loc> WaypointPositions;
    CVector3 HomePos;

    /**
     * A reference to the simulated space.
     */
    CSpace* m_pcSpace;

    /* simulation parameters */
    SSwarmParams m_sSwarmParams;
    SQuadLaunchParams m_sQuadLaunchParams;
};

#endif