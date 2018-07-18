#ifndef EYEBOT_PSO_H
#define EYEBOT_PSO_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the quadrotor positioning actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_position_actuator.h>
/* Definition of the camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_camera_sensor.h>
/* Definition of the tag detector algorithm */
#include <argos3/plugins/robots/generic/control_interface/ci_camera_sensor_tag_detector_algorithm.h>
/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definition of the range-and-bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the eye-bot proximity sensor */
#include <argos3/plugins/robots/eye-bot/control_interface/ci_eyebot_proximity_sensor.h>

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
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><eyebot_pso_controller> section.
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

private:
    /*
    * Takes off the robot.
    */
    void TakeOff();

    /*
    * Lands the robot.
    */
    void Land();

private:

    /* Current robot state */
    enum EState {
        STATE_START = 0,
        STATE_TAKE_OFF,
        STATE_LAND
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
    /* Pointer to the camera sensor */
    CCI_CameraSensor* m_pcCamera;
    /* Pointer to the tag detector algorithm */
    CCI_CameraSensorTagDetectorAlgorithm* m_pcTags;

    /* Current robot state */
    EState m_eState;
    /* Current target position */
    CVector3 m_cTargetPos;

    /* Contains the message received from the foot-bot */
    const CCI_RangeAndBearingSensor::SPacket* m_psFBMsg;
    /* Contains the message received tag detector */
    const CCI_CameraSensorTagDetectorAlgorithm::SReading* m_psTDMsg;
};

#endif