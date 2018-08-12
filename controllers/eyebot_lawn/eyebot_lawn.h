#ifndef EYEBOT_LAWN_H
#define EYEBOT_LAWN_H

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
class CEyeBotLawn : public CCI_Controller {

public:

    /* Class constructor. */
    CEyeBotLawn();

    /* Class destructor. */
    virtual ~CEyeBotLawn() {}

    /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><eyebot_lawn_controller> section.
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

    inline void UpdateWaypoint() {
        m_sStateData.WaypointIndex++;
    }

    inline std::vector<double> GetWaypoint() {
        return (m_sStateData.WaypointMap[m_sStateData.WaypointIndex]);
    }

private:

    /*
    * Takes off the robot.
    */
    void TakeOff();

    /*
    * Moves the robot along the lawn path.
    */
    void Move();

    /*
    * Lands the robot.
    */
    void Land();

private:

    /*
    * Map the wall and generate WaypointMap
    */
    void MapWall();

private:

    /*
    * Waypoint parameters
    */
    struct SWaypointParams {
        /* Horizontal and Vertical step values */
        /* Distance to move vertically along the lawn length at */
        double vstep;
        /* Distance to move horizontally along the lawn length at */
        double hstep;

        void Init(TConfigurationNode& t_node);
    };

    /*
    * Controller state data
    */
    struct SStateData {
        /* Y-axis reach to the wall/target space */
        double Reach;
        /* Current robot state */
        enum EState {
            STATE_START = 0,
            STATE_TAKE_OFF,
            STATE_MOVE,
            STATE_LAND
        } State;

        /* Attitude height above target to hold task execution */
        double attitude;
        /* Launch altitude height */
        double launch_altitude;
        /* Average plane distance to wall to move along the Pso path */
        double global_reach;
        /* Tolerance threshold for the distance to a target point */
        double proximity_tolerance;
        /* The minimum number of steps in holding mode before the eyebot can advance waypoints */
        double minimum_hold_time;
        /* Current robot waypoint location index */
        size_t WaypointIndex;
        /* Time that the drone will hold at target while it performs task */
        size_t HoldTime;
        /* Current robot waypoint/target map */
        std::map<size_t, std::vector<double>> WaypointMap;

        void Init(TConfigurationNode& t_node);
        void Reset();
    };

private:

    /* Pointer to the quadrotor position actuator */
    CCI_QuadRotorPositionActuator* m_pcPosAct;
    /* Pointer to the positioning sensor */
    CCI_PositioningSensor* m_pcPosSens;
    /* Pointer to the range-and-bearing sensor */
    CCI_RangeAndBearingSensor* m_pcRABSens;
    /* Pointer to the eye-bot proximity sensor */
    CCI_EyeBotProximitySensor* m_pcProximity;

    /* Current target position */
    CVector3 m_cTargetPos;
    // The controller state information
    SStateData m_sStateData;
    SWaypointParams m_sWaypointParams;

    /*
     * References to simulated space variables.
     */
    CSpace* m_pcSpace;

    /*
    * Waypoint storage container.
    */
    std::map< size_t, std::vector<double> > WaypointMap;
    /* Index pointer that addresses current robot waypoint */
    size_t WaypointIndex;

    /* Contains the message received from the foot-bot */
    const CCI_RangeAndBearingSensor::SPacket* m_psFBMsg;
};

#endif
