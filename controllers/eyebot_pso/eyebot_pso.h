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
/* Definition of the eye-bot proximity sensor */
#include <argos3/plugins/robots/eye-bot/control_interface/ci_eyebot_proximity_sensor.h>
/* Definition of the perspective camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_perspective_camera_sensor.h>
/* Definition of the eye-bot light sensor */
#include <argos3/plugins/robots/eye-bot/control_interface/ci_eyebot_light_sensor.h>
/* Definitions for the argos space */
#include <argos3/core/simulator/space/space.h>
/* Include the kalman filter algorithm definitions */
#include <filters/kalman/kalman.h>
/* Definition of the argos entities */
#include <argos3/plugins/simulator/entities/light_entity.h>
/* Include the pso swarm algorithm definitions */
#include <algorithms/pso/swarm.h>
/* Definitions for the eigen library */
#include <Eigen/Dense>

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
    void GenerateWaypoints(bool& naive, bool& add_origin);

    /*
    * Compute (naively or via camera vision) the position
    * location and size of the wall.
    */
    void MapWall(bool& naive);

    /*
    * Perform basic kalman filtering on quadcopter
    * position measurements.
    */
    void UpdatePosition(CVector3 x0 = CVector3(0.,0.,0.));

    /*
    * Update the target light entity to the
    * nearest light led.
    */
    void UpdateNearestLight();

    /*
    * Perform the requisite task on the target plant.
    */
    void ExecuteTask();

    /*
    * Pre-assign tasks to the eyebots in the arena
    */
    void AllocateTasks();

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
    /*
    * Simple Kalman filter struct
    */
    struct SKF {
        int n = 3; // Number of states
        int m = 3; // Number of measurements
        double dt = 10/60; // Time step

        Eigen::MatrixXd A = Eigen::MatrixXd(n, n); // System dynamics matrix
        Eigen::MatrixXd C = Eigen::MatrixXd(m, n); // Output matrix
        Eigen::MatrixXd Q = Eigen::MatrixXd(n, n); // Process noise covariance
        Eigen::MatrixXd R = Eigen::MatrixXd(m, m); // Measurement noise covariance
        Eigen::MatrixXd P = Eigen::MatrixXd(n, n); // Estimate error covariance

        // Construct the state vector
        CVector3 state;
        SKF();
    };

    /*
    * Waypoint parameters
    */
    struct SWaypointParams {
        /* gaussian dist. noise parameters for target sensing */
        double ns_mean;
        double ns_stddev;
        double z_assess;
        bool naive_mapping;
        bool add_origin;

        void Init(TConfigurationNode& t_node);
    };

    /*
    * Noise sources
    */
    struct SGaussDist {
        /* Define random generator with Gaussian distribution for target mapping noise */
        std::default_random_engine* gen;
        std::normal_distribution<double>* nd;

        void Init(double& mean, double& stddev, int& gen_seed);
        double Rand() {
            return (*nd)(*gen);
        };
    };

    struct SUniformIntDist {
        /* Define random generator with Gaussian distribution for target mapping noise */
        std::default_random_engine* gen;
        std::uniform_int_distribution<int>* uid;

        void Init(int min, int max, int seed);
        int Rand() {
            return (*uid)(*gen);
        };
    };

private:

    /* Current robot state */
    enum EState {
        STATE_START = 0,
        STATE_TAKE_OFF,
        STATE_ADVANCE,
        STATE_EXECUTE_TASK,
        STATE_LAND
    };

    /* Eyebot tasks */
    enum ETask {
        EVALUATE_TASK = 0,
        WATER_TASK,
        TREATMENT_TASK,
        NOURISH_TASK
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
    /* Alloted robot task */
    ETask m_eTask;
    /* Current target position */
    CVector3 m_cTargetPos;
    /* Target locations */
    std::vector<std::vector<double>> m_cPlantLocList;
    /* Used to move the robot along the pso trajectory */
    UInt32 m_unWaypoint;
    /* Perspective camera readings variable */
    CCI_ColoredBlobPerspectiveCameraSensor::SReadings m_cSReadings;

    /* Waypoint variables */
    typedef std::vector<double> wp_loc;
    std::vector<wp_loc> WaypointPositions;
    CVector3 HomePos;

    /*
     * References to simulated space variables.
     */
    CSpace* m_pcSpace;
    KalmanFilter* kf;
    CLightEntity* m_cTargetLight;

    /* simulation struct parameters */
    SSwarmParams m_sSwarmParams;
    SQuadLaunchParams m_sQuadLaunchParams;
    SWaypointParams m_sWaypointParams;
    SKF m_sKalmanFilter;
    SGaussDist m_sMappingNoise;
    SUniformIntDist m_sTargetStateShuffle;
    SUniformIntDist m_sTaskCompleted;
    /* swarm solution variable */
    struct tsp_sol swarm_sol;
    /* Eyebot tasks:
    * GRAY      - Undecided
    * GREEN     - Healthy
    * YELLOW    - Diseased
    * RED       - Wilting
    */
    std::vector<CColor> m_pTargetStates{CColor::GREEN, CColor::GRAY50, CColor::BROWN, CColor::YELLOW, CColor::RED};
    std::vector<ETask> m_pTasks{EVALUATE_TASK, WATER_TASK, NOURISH_TASK, TREATMENT_TASK};
    std::map<std::string, ETask> m_mTaskedEyeBots;
};

#endif