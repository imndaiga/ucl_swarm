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
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
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

    inline void UpdateWaypoint() {
        if(m_sStateData.IdleTime > m_sDroneParams.minimum_idle_time) {
            m_sStateData.WaypointIndex++;
            m_sStateData.IdleTime = 0;
        } else {
            m_sStateData.IdleTime++;
        }
    }

    inline std::vector<double> GetWaypoint() {
        return std::get<0>(m_sStateData.WaypointMap[m_sStateData.WaypointIndex]);
    }

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
    void GenerateWaypoints(bool& naive);

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
    * Pre-assign tasks to the eyebots in the arena.
    */
    void AllocateTasks();

    /*
    * Social rule listener
    */
    void ListenToNeighbours();

    /*
    * Based on the assigned tag perform varied tasks.
    * White - reassign tag to plant
    * Green - leave plant alone
    * Yellow - apply medication
    * Red - water the plant
    */
    void EvaluateFunction();
    void WaterFunction();
    void NourishFunction();
    void TreatmentFunction();

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
    * The drone launch and mode params.
    */
    struct SDroneParams {
        /* Altitude at drone takeoff */
        Real launch_altitude;
        /* Attitude height above target to maintain during task execution */
        Real attitude;
        /* Average plane distance to wall to move along the Pso path */
        Real global_reach;
        /* Tolerance threshold for the distance to a target point */
        Real proximity_tolerance;
        /* The minimum number of steps in idling state before the eyebot can advance waypoints */
        Real minimum_idle_time;

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
        bool naive_mapping;

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

        void Init(int min, int max, int& gen_seed);
        int Rand() {
            return (*uid)(*gen);
        };
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
                STATE_ADVANCE,
                STATE_EXECUTE_TASK,
                STATE_LAND
            } State;

            /* Current robot task */
            enum ETask {
                TASK_EVALUATE = 0,
                TASK_WATER,
                TASK_TREATMENT,
                TASK_NOURISH,
                TASK_NULL
            } TaskState;

            /* Current robot waypoint location index */
            size_t WaypointIndex;
            size_t IdleTime;
            /* Current robot waypoint/target map */
            std::map<size_t, std::pair<std::vector<double>, SStateData::ETask>> WaypointMap;
            void Init(double& global_reach);
            void Reset();
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
    /* Pointer to the range and bearing actuator */
    CCI_RangeAndBearingActuator*  m_pcRABA;
    /* Pointer to the range and bearing sensor */
    CCI_RangeAndBearingSensor* m_pcRABS;

    /* The controller state information */
    SStateData m_sStateData;
    /* Current target position */
    CVector3 m_cTargetPos;
    /* Target locations */
    std::vector<std::vector<double>> m_cPlantLocList;
    /* Perspective camera readings variable */
    CCI_ColoredBlobPerspectiveCameraSensor::SReadings m_cSReadings;
    CVector3 HomePos;

    /*
     * References to simulated space variables.
     */
    CSpace* m_pcSpace;
    KalmanFilter* kf;
    CLightEntity* m_cTargetLight;
    // Pointer to task function.
    void (CEyeBotPso::*TaskFunction)();

    /* simulation struct parameters */
    SSwarmParams m_sSwarmParams;
    SDroneParams m_sDroneParams;
    SWaypointParams m_sWaypointParams;
    SKF m_sKalmanFilter;
    SGaussDist m_sMappingNoise;
    SUniformIntDist m_sTargetStateShuffle;
    SUniformIntDist m_sTaskCompleted;

    /* swarm solution variable */
    struct tsp_sol swarm_sol;

    /* Eyebot task mapping:
    * WHITE     - Undecided/Unknown
    * GREEN     - Healthy
    * YELLOW    - Diseased
    * BROWN     - Dry
    * RED       - Wilting
    */
    std::vector<CColor> m_pTargetStates{CColor::WHITE, CColor::GREEN, CColor::BROWN, CColor::YELLOW, CColor::RED};
    std::vector<SStateData::ETask> m_pTaskStates{SStateData::TASK_EVALUATE, SStateData::TASK_WATER, SStateData::TASK_NOURISH, SStateData::TASK_TREATMENT};
    std::map<std::string, SStateData::ETask> m_mTaskedEyeBots;
    std::map<size_t, std::pair<std::vector<double>, SStateData::ETask>> GlobalMap;
};

#endif