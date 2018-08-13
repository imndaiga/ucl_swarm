#ifndef LAWN_H
#define LAWN_H

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
/* Definition of the argos entities */
#include <argos3/plugins/simulator/entities/light_entity.h>
/* Definitions for the eigen library */
#include <Eigen/Dense>
/* Include the kalman filter algorithm definitions */
#include <filters/kalman/kalman.h>

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

    inline CVector3 GetPosition() {
        return (m_sKalmanFilter.state);
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

    /*
    * Perform the requisite task on the target plant.
    */
    void ExecuteTask();

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

private:

    /*
    * Map the wall and generate WaypointMap
    */
    void MapWall();

    /*
    * Perform basic kalman filtering on quadcopter
    * position measurements.
    */
    void UpdatePosition(CVector3 x0 = CVector3(0.,0.,0.));

    /*
    * Update the target light entity to the
    * nearest light led.
    */
    void UpdateNearestTarget();

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
            STATE_EXECUTE_TASK,
            STATE_MOVE,
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
        /* Assigned task at current target */
        ETask TargetTask;

        void Init(TConfigurationNode& t_node);
        void Reset();
    };

    /*
    * Noise sources
    */
    struct SGaussDist {
        /* Define random generator with Gaussian distribution */
        std::default_random_engine* gen;
        std::normal_distribution<double>* nd;

        void Init(double& mean, double& stddev, int& gen_seed);
        double min() {
            return nd->min();
        };
        double max() {
            return nd->max();
        };
        double get() {
            return (*nd)(*gen);
        };
    };

    struct SUniformIntDist {
        /* Define discete random generator with Uniform distribution */
        std::default_random_engine* gen;
        std::uniform_int_distribution<int>* uid;

        void Init(int min, int max, int& gen_seed);
        int min() {
            return uid->min();
        };
        int max() {
            return uid->max();
        };
        int get() {
            return (*uid)(*gen);
        };
    };

    /*
    * Random generators
    */
    struct SRandomGen {
        SGaussDist mapping;
        SUniformIntDist targetshuffle;

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

private:

    /* Pointer to the quadrotor position actuator */
    CCI_QuadRotorPositionActuator* m_pcPosAct;
    /* Pointer to the positioning sensor */
    CCI_PositioningSensor* m_pcPosSens;
    /* Pointer to the eye-bot proximity sensor */
    CCI_EyeBotProximitySensor* m_pcProximity;

    /* Current target position */
    CVector3 m_cTargetPos;
    /* Current robots' home position */
    CVector3 HomePos;
    // The controller state information
    SStateData m_sStateData;
    SWaypointParams m_sWaypointParams;
    SRandomGen m_sRandGen;
    SKF m_sKalmanFilter;

    /*
     * References to simulated space variables.
     */
    CSpace* m_pcSpace;
    KalmanFilter* kf;
    CLightEntity* m_cNearestTarget;

    /*
    * Waypoint storage container.
    */
    std::map< size_t, std::vector<double> > WaypointMap;
    /* Index pointer that addresses current robot waypoint */
    size_t WaypointIndex;

    /* Eyebot task mapping:
    * WHITE     - Undecided/Unknown
    * GREEN     - Healthy
    * YELLOW    - Diseased
    * BROWN     - Dry
    * RED       - Wilting
    */
    std::vector<CColor> m_pTargetStates{CColor::WHITE, CColor::GREEN, CColor::BROWN, CColor::YELLOW, CColor::RED};
    std::vector<SStateData::ETask> m_pTaskStates{SStateData::TASK_EVALUATE, SStateData::TASK_WATER, SStateData::TASK_NOURISH, SStateData::TASK_TREATMENT};
    std::vector<std::string> m_pTaskNames{"evaluate", "water", "nourish", "treatment"};

    /* Contains the message received from the foot-bot */
    const CCI_RangeAndBearingSensor::SPacket* m_psFBMsg;
};

#endif
