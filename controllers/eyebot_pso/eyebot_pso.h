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
        if(m_sStateData.HoldTime > m_sStateData.minimum_hold_time) {
            m_sStateData.WaypointIndex++;
            m_sStateData.HoldTime = 0;
        } else {
            m_sStateData.HoldTime++;
        }
    }

    inline std::vector<double> GetWaypoint() {
        return (m_sStateData.WaypointMap[m_sStateData.WaypointIndex]);
    }

    inline void IncreaseLandingProb() {
        // Increase probability that robot will go into land state.
        m_sStateData.RestToLandProb += m_sStateData.SocialRuleRestToLandDeltaProb;
        // Truncate RestToLand probability value.
        m_sStateData.RestToLandProb = fmax(fmin(m_sStateData.RestToLandProb, m_sRestToLandGen.max()), m_sRestToLandGen.min());
        // Decrease probability that robot will go into move state.
        m_sStateData.RestToMoveProb -= m_sStateData.SocialRuleRestToMoveDeltaProb;
        // Truncate RestToMove probability value.
        m_sStateData.RestToMoveProb = fmax(fmin(m_sStateData.RestToMoveProb, m_sRestToMoveGen.max()), m_sRestToMoveGen.min());
    }

    inline void IncreaseMovingProb() {
        // Increase probability that robot will go into move state.
        m_sStateData.RestToMoveProb += m_sStateData.SocialRuleRestToMoveDeltaProb;
        // Truncate RestToMove probability value.
        m_sStateData.RestToMoveProb = fmax(fmin(m_sStateData.RestToMoveProb, m_sRestToMoveGen.max()), m_sRestToMoveGen.min());
        // Decrease probability that robot will go into land state.
        m_sStateData.RestToLandProb -= m_sStateData.SocialRuleRestToLandDeltaProb;
        // Truncate RestToLand probability value.
        m_sStateData.RestToLandProb = fmax(fmin(m_sStateData.RestToLandProb, m_sRestToLandGen.max()), m_sRestToLandGen.min());
    }

private:

    /*
    * Lands the robot.
    */
    void Land();

    /*
    * Move robot to next waypoint.
    */
    void Move();

    /*
    * Rest time before replanning waypoints.
    */
    void Rest();

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
    * initialize and waypoint targets in the arena: this can be done naively
    * with the passed argos parameters or with the help
    * of the camera sensor.
    */
    void InitializeWaypoints(std::vector< std::vector<double> >& waypoints, bool naive = true);

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

    /*
    * Distribute tasks between available eye-bots
    * in the arena. The initializer must be run before
    * we can generate waypoints.
    */
    void InitializeSwarm();

    /*
    * Social rule listener
    */
    void ListenToNeighbours();

    /*
    * Process and parse received RAB messages
    * into valid waypoint.
    */
    void ProcessWaypoint(UInt8& task_id, UInt8& wp_id, UInt8& agent_id);

    /*
    * Generate optimal path for waypoints listed in UnorderedWaypoints.
    */
    void GenerateMap(std::map<size_t, std::vector<double>>& map, std::vector< std::vector<double> >& unsorted_waypoints, bool verbose = false);

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
    * Random generator seed params.
    */
    struct SSeedParams {
        int mapping;
        int shuffle;
        int success;
        int move;
        int land;

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
        double Rand() {
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
        int Rand() {
            return (*uid)(*gen);
        };
    };

    struct SUniformRealDist {
        /* Define continuous random generator with Uniform distribution */
        std::default_random_engine* gen;
        std::uniform_real_distribution<double>* udd;

        void Init(double min, double max, int& gen_seed);
        double min() {
            return udd->min();
        };
        double max() {
            return udd->max();
        };
        double Rand() {
            return (*udd)(*gen);
        };
    };

    /*
    * Controller state data
    */
    struct SStateData {
            /* Y-axis reach to the wall/target space */
            double Reach;
            /* Boolean to determine if drone is leader */
            bool IsLeader;
            /* Current robot state */
            enum EState {
                STATE_START = 0,
                STATE_MOVE,
                STATE_EXECUTE_TASK,
                STATE_REST,
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
            /* Average plane distance to wall to move along the Pso path */
            double global_reach;
            /* Tolerance threshold for the distance to a target point */
            double proximity_tolerance;
            /* The minimum number of steps in holding mode before the eyebot can advance waypoints */
            double minimum_hold_time;
            /* The minimum number of steps to wait before replanning unordered waypoints */
            double minimum_rest_time;
            /* Initial probability to switch from resting to moving */
            double InitialRestToMoveProb;
            /* Current probability to switch from resting to exploring */
            double RestToMoveProb;
            /* The increase of RestToMoveProb due to the social rule */
            double SocialRuleRestToMoveDeltaProb;
            /* Initial probability to switch from moving to landing */
            double InitialRestToLandProb;
            /* Current probability to switch from moving to landing */
            double RestToLandProb;
            /* The increase of RestToLandProb due to the social rule */
            double SocialRuleRestToLandDeltaProb;
            /* Reach modifiers mapping */
            std::map<SStateData::ETask, double> ReachModifiers{{SStateData::TASK_EVALUATE, 0.8},{SStateData::TASK_WATER, 0.4},{SStateData::TASK_NOURISH, -0.4},{SStateData::TASK_TREATMENT, -0.8}};
            /* Current robot waypoint location index */
            size_t WaypointIndex;
            /* Time that the drone will hold at target while it performs task */
            size_t HoldTime;
            /* Time that the drone will remain in rest mode */
            size_t RestTime;
            /* Current robot waypoint/target map */
            std::map<size_t, std::vector<double>> WaypointMap;
            std::vector<std::vector<double>> UnorderedWaypoints;

            void Init(TConfigurationNode& t_node);
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
    /* Contains the message received from the foot-bot */
    const CCI_RangeAndBearingSensor::SPacket* m_pEBMsg;

    /* Current target position */
    CVector3 m_cTargetPos;
    /* Perspective camera readings variable */
    CCI_ColoredBlobPerspectiveCameraSensor::SReadings m_cSReadings;
    CVector3 HomePos;

    /*
     * References to simulated space variables.
     */
    CSpace* m_pcSpace;
    KalmanFilter* kf;
    CLightEntity* m_cNearestTarget;
    // Pointer to task function.
    void (CEyeBotPso::*TaskFunction)();

    /* simulation struct parameters */
    SSwarmParams m_sSwarmParams;
    // The controller state information
    SStateData m_sStateData;
    SWaypointParams m_sWaypointParams;
    SSeedParams m_sSeedParams;
    SKF m_sKalmanFilter;

    SGaussDist m_sMappingNoiseGen;
    SUniformIntDist m_sTargetStateShuffleGen;
    SUniformIntDist m_sTaskCompletedGen;
    SUniformRealDist m_sRestToMoveGen;
    SUniformRealDist m_sRestToLandGen;

    /* Swarm solution variable */
    struct tsp_sol swarm_sol;
    /* Flag to indicate when the swarm has been initialized */
    bool swarm_initialized;

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
    std::map<std::string, SStateData::ETask> m_mTaskedEyeBots;
    std::map<size_t, std::vector<double>> m_pGlobalMap;
};

#endif